#include "MeshDependentResource.h"

#include <igl/per_edge_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/signed_distance.h>
#include "../GeometryUtils.h"

namespace psg {
namespace core {
namespace models {
void MeshDependentResource::init(const Eigen::MatrixXd& V_,
                                 const Eigen::MatrixXi& F_) {
  if (initialized) {
    tree.deinit();
    intersector.deinit();
  }
  V = V_;
  F = F_;
  tree.init(V_, F_);
  intersector.init(V.cast<float>(), F, true);
  igl::per_face_normals(V_, F_, FN);
  igl::per_vertex_normals(
      V_, F_, igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_ANGLE, FN, VN);
  igl::per_edge_normals(
      V_, F_, igl::PER_EDGE_NORMALS_WEIGHTING_TYPE_UNIFORM, FN, EN, E, EMAP);

  minimum = V.colwise().minCoeff();
  maximum = V.colwise().maxCoeff();
  size = maximum - minimum;

  center_of_mass = CenterOfMass(V, F);
  SP_valid = false;
}

void MeshDependentResource::init_sp() {
  if (SP_valid) return;
  size_t nV = V.rows();
  size_t nF = F.rows();
  std::vector<std::vector<std::pair<int, double>>> edges(nV);
  SP.resize(nV, nV);
  // SP_par.resize(nV, nV);
  SP.setConstant(std::numeric_limits<double>::max() / 2.);
  // SP_par.setConstant(-1);
  for (size_t i = 0; i < nF; i++) {
    for (size_t k = 0; k < 3; k++) {
      size_t jj = (k + 1) % 3;
      int u = F(i, k);
      int v = F(i, jj);
      edges[u].push_back({v, (V.row(u) - V.row(v)).norm()});
    }
  }

  struct VertexInfo {
    int id;
    double dist;
    bool operator<(const VertexInfo& r) const { return dist > r.dist; }
  };
#pragma omp parallel for
  for (int src = 0; src < nV; src++) {
    std::priority_queue<VertexInfo> q;
    q.push(VertexInfo{src, 0});
    SP(src, src) = 0;
    while (!q.empty()) {
      VertexInfo u = q.top();
      q.pop();
      double curDis;
      for (const auto& v : edges[u.id]) {
        if ((curDis = u.dist + v.second) < SP(v.first, src)) {
          SP(v.first, src) = curDis;
          // SP_par(v.first, src) = u.id;
          q.push(VertexInfo{v.first, curDis});
        }
      }
    }
  }
  SP_valid = true;
}

double MeshDependentResource::ComputeSignedDistance(
    const Eigen::Vector3d& position,
    Eigen::RowVector3d& c,
    double& s) const {
  double sqrd;
  int i;
  Eigen::RowVector3d n;
  igl::signed_distance_pseudonormal(
      tree, V, F, FN, VN, EN, EMAP, position.transpose(), s, sqrd, i, c, n);
  return s * sqrt(sqrd);
}
size_t MeshDependentResource::ComputeClosestFacet(
    const Eigen::Vector3d& position) const {
  Eigen::RowVector3d c;
  int fid;
  tree.squared_distance(V, F, position.transpose(), fid, c);
  return fid;
}
size_t MeshDependentResource::ComputeClosestVertex(
    const Eigen::Vector3d& position) const {
  Eigen::RowVector3d c;
  int fid;
  tree.squared_distance(V, F, position.transpose(), fid, c);
  double dist = std::numeric_limits<double>::max();
  size_t vid = -1;
  for (size_t i = 0; i < 3; i++) {
    int cur_vid = F(fid, i);
    double cur_dist = (V.row(cur_vid) - position.transpose()).squaredNorm();
    if (cur_dist < dist) {
      dist = cur_dist;
      vid = cur_vid;
    }
  }
  return vid;
}

double MeshDependentResource::ComputeRequiredDistance(
    const Eigen::Vector3d& A,
    const Eigen::Vector3d& B) const {
  assert(SP_valid);
  Eigen::RowVector3d dir = B - A;
  double norm = dir.norm();
  dir /= norm;
  std::vector<igl::Hit> hits;
  int numRays;
  intersector.intersectRay(A.cast<float>(), dir.cast<float>(), hits, numRays);
  bool isIn = hits.size() % 2 == 1;
  size_t lastFid = -1;
  double lastT = 0;
  if (isIn) {
    lastFid = ComputeClosestFacet(A);
  }
  double totalDis = 0;
  for (const igl::Hit& hit : hits) {
    if (hit.t >= norm) break;
    if (isIn) {
      size_t fid = hit.id;
      double bestDis = std::numeric_limits<double>::max();
      for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
          int u = F(lastFid, i);
          int v = F(fid, j);
          double dU = ((A.transpose() + dir * lastT) - V.row(u)).norm();
          double dV = ((A.transpose() + dir * hit.t) - V.row(v)).norm();
          bestDis = std::min(bestDis, SP(u, v) + dU + dV);
        }
      }
      totalDis += bestDis;
      // } else {
      // totalDis += hit.t - lastT;
    }
    lastFid = hit.id;
    isIn = !isIn;
    lastT = hit.t;
  }
  if (isIn) {
    // B is in
    size_t fid = ComputeClosestFacet(B);
    double bestDis = std::numeric_limits<double>::max();
    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        int u = F(lastFid, i);
        int v = F(fid, j);
        double dU = ((A.transpose() + dir * lastT) - V.row(u)).norm();
        double dV = (B.transpose() - V.row(v)).norm();
        bestDis = std::min(bestDis, SP(u, v) + dU + dV);
      }
    }
    totalDis += bestDis;
  }
  return totalDis;
}

}  // namespace models
}  // namespace core
}  // namespace psg