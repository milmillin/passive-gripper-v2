#pragma once

#include <igl/AABB.h>
#include <igl/embree/EmbreeIntersector.h>
#include <Eigen/Core>

#include "../Constants.h"

namespace psg {
namespace models {

struct MeshDependentResource {
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  Eigen::MatrixXd FN;
  Eigen::MatrixXd VN;
  Eigen::MatrixXd EN;
  Eigen::MatrixXi E;
  Eigen::MatrixXi EMAP;
  Eigen::Vector3d center_of_mass;
  igl::AABB<Eigen::MatrixXd, 3> tree;
  igl::embree::EmbreeIntersector intersector;

  bool initialized = false;
  void init(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

  // out_c: closest point
  // out_s: sign
  double ComputeSignedDistance(const Eigen::Vector3d& position,
                               Eigen::RowVector3d& out_c,
                               double& out_s) const;

  size_t ComputeClosestFacet(const Eigen::Vector3d& position) const;
};

}  // namespace models
}  // namespace psg