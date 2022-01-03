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
}  // namespace models
}  // namespace core
}  // namespace psg