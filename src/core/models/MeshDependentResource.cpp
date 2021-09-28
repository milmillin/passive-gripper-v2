#include "MeshDependentResource.h"

#include <igl/per_edge_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/signed_distance.h>

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

  // TODO: fix center of mass
  center_of_mass =
      (V.colwise().minCoeff() + V.colwise().maxCoeff()).transpose() / 2.;
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
}  // namespace models
}  // namespace core
}  // namespace psg