#include "TopoOpt.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <map>
#include <vector>

#include "PassiveGripper.h"
#include "SweptVolume.h"
#include "models/MeshDependentResource.h"
#include "robots/Robots.h"

namespace psg {
namespace core {

static std::vector<Eigen::Vector3i> GetForbiddenVoxels(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const Eigen::Vector3d& lb,
    const Eigen::Vector3d& ub,
    double res,
    Eigen::Vector3i& out_range) {
  igl::embree::EmbreeIntersector intersector;
  intersector.init(V.cast<float>(), F, true);
  std::vector<Eigen::Vector3i> voxels;
  int x, y, z;
  double cx, cy, cz;
  for (x = 0, cx = lb(0); cx < ub(0); x++, cx += res) {
    for (y = 0, cy = lb(1); cy < ub(1); y++, cy += res) {
      for (z = 0, cz = lb(2); cz < ub(2); z++, cz += res) {
        auto position = Eigen::Vector3d(cx, cy, cz);
        position += Eigen::Vector3d(res, res, res) / 2;
        std::vector<igl::Hit> hits;
        int numRays;
        intersector.intersectRay(
            position.cast<float>(), Eigen::RowVector3f::UnitZ(), hits, numRays);
        if (hits.size() % 2 == 0) {
          voxels.push_back(Eigen::Vector3i(x, y, z));
        }
      }
    }
  }
  out_range = Eigen::Vector3i(x, y, z);
  return voxels;
}

static inline Eigen::Vector3i PointToVoxel(const Eigen::Vector3d& p,
                                           const Eigen::Vector3d& lb,
                                           double res) {
  return ((p - lb).array() / res).cast<int>();
}

static inline int VoxelToIndex(const Eigen::Vector3i& v,
                               const Eigen::Vector3i& range) {
  constexpr int X = 0;
  constexpr int Y = 1;
  constexpr int Z = 2;
  return v(X) * range(Y) + (range(Y) - 1 - v(Y)) + v(Z) * range(X) * range(Y) +
         1;
}

static std::vector<int> ConvertToIndices(const std::vector<Eigen::Vector3i>& v,
                                         const Eigen::Vector3i& range) {
  std::vector<int> res(v.size());
  for (size_t i = 0; i < v.size(); i++) {
    res[i] = VoxelToIndex(v[i], range);
  }
  std::sort(res.begin(), res.end());
  res.resize(std::unique(res.begin(), res.end()) - res.begin());
  return res;
}

static bool VoxelValid(const Eigen::Vector3i& v, const Eigen::Vector3i& range) {
  for (int i = 0; i < 3; i++) {
    if (v(i) < 0) return false;
    if (v(i) >= range(i)) return false;
  }
  return true;
}

static Eigen::Vector3i ClosestEmptySpace(
    const Eigen::Vector3i& p,
    const std::vector<int>& forbidden_indices,
    const Eigen::Vector3i& range) {
  int d = 0;
  while (true) {
    for (int dx = -d; dx <= d; dx++) {
      for (int dy = -d; dy < d; dy++) {
        for (int dz = -d; dz < d; dz++) {
          Eigen::Vector3i newP = p + Eigen::Vector3i(dx, dy, dz);
          if (!VoxelValid(newP, range)) continue;
          int index = VoxelToIndex(newP, range);
          auto res = std::lower_bound(
              forbidden_indices.begin(), forbidden_indices.end(), index);
          if (res == forbidden_indices.end() || *res != index) {
            return newP;
          }
        }
      }
    }
    d++;
  }
  return Eigen::Vector3i::Constant(-1);
}

template <class T>
static std::string FormatNodeList(const T& v) {
  std::string result;
  for (auto& point : v) {
    if (result.size() != 0) {
      result += ';';
    }
    result += std::to_string(point);
  }
  return result;
}

static void WriteToFile(std::string fileName,
                        const std::map<std::string, std::string>& config) {
  std::ofstream fout(fileName);
  fout << "[ToPy Problem Definition File v2007]" << std::endl;
  for (auto& pair : config) {
    auto& key = pair.first;
    auto& value = pair.second;
    fout << key << ":" << value << std::endl;
  }
}

void GenerateTopyConfig(const PassiveGripper& psg,
                        const Eigen::MatrixXd& neg_V,
                        const Eigen::MatrixXi& neg_F,
                        const std::string& filename) {
  Eigen::Vector3d lb = psg.GetTopoOptSettings().lower_bound;
  Eigen::Vector3d ub = psg.GetTopoOptSettings().upper_bound;
  double res = psg.GetTopoOptSettings().topo_res;
  Eigen::Vector3i range;

  std::vector<Eigen::Vector3i> forbidden_voxels =
      GetForbiddenVoxels(neg_V, neg_F, lb, ub, res, range);
  std::vector<int> forbidden_indices =
      ConvertToIndices(forbidden_voxels, range);

  std::vector<Eigen::Vector3i> attachment_voxels;
  int sample = psg.GetTopoOptSettings().attachment_samples;
  double size = psg.GetTopoOptSettings().attachment_size;
  for (int x = 0; x < sample; x++) {
    for (int y = 0; y < sample; y++) {
      Eigen::Vector3d attachment(((double)x - sample / 2 + 0.5) * size / sample,
                                 ((double)y - sample / 2 + 0.5) * size / sample,
                                 0);
      attachment_voxels.push_back(PointToVoxel(attachment, lb, res));
    }
  }
  std::vector<int> attachment_indices =
      ConvertToIndices(attachment_voxels, range);

  Eigen::Affine3d finger_trans_inv =
      robots::Forward(psg.GetTrajectory().front()).inverse();
  std::vector<Eigen::Vector3i> contact_voxels;
  for (const auto& point : psg.GetContactPoints()) {
    contact_voxels.push_back(ClosestEmptySpace(
        PointToVoxel(finger_trans_inv * point.position, lb, res),
        forbidden_indices,
        range));
  }
  std::vector<int> contact_indices = ConvertToIndices(contact_voxels, range);

  size_t lastdot = filename.rfind('.');

  std::map<std::string, std::string> config = kTopyConfig;
  config["PROB_NAME"] = filename.substr(0, lastdot);
  config["NUM_ELEM_X"] = std::to_string(range(0));
  config["NUM_ELEM_Y"] = std::to_string(range(1));
  config["NUM_ELEM_Z"] = std::to_string(range(2));
  config["FXTR_NODE_X"] = config["FXTR_NODE_Y"] = config["FXTR_NODE_Z"] =
      FormatNodeList(attachment_indices);
  config["LOAD_NODE_X"] = config["LOAD_NODE_Y"] = config["LOAD_NODE_Z"] =
      FormatNodeList(contact_indices);
  std::vector<double> loadX, loadY, loadZ;
  for (auto& point : psg.GetContactPoints()) {
    Eigen::Vector3d tN = finger_trans_inv.linear() * point.normal;
    loadX.push_back(tN(0));
    loadY.push_back(tN(1));
    loadZ.push_back(tN(2));
  }
  config["LOAD_VALU_X"] = FormatNodeList(loadX);
  config["LOAD_VALU_Y"] = FormatNodeList(loadY);
  config["LOAD_VALU_Z"] = FormatNodeList(loadZ);
  config["PASV_ELEM"] = FormatNodeList(forbidden_indices);

  WriteToFile(filename, config);
}

}  // namespace core
}  // namespace psg
