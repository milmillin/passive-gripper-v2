#pragma once

#include <igl/AABB.h>
#include <igl/embree/EmbreeIntersector.h>
#include <Eigen/Core>

#include "../../Constants.h"
#include "../Debugger.h"
#include "../serialization/Serialization.h"

namespace psg {
namespace core {
namespace models {

struct MeshDependentResource : psg::core::serialization::Serializable {
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  Eigen::MatrixXd FN;
  Eigen::MatrixXd VN;
  Eigen::MatrixXd EN;
  Eigen::MatrixXi E;
  Eigen::MatrixXi EMAP;
  Eigen::Vector3d center_of_mass;
  Eigen::Vector3d minimum;
  Eigen::Vector3d maximum;
  Eigen::Vector3d size;
  igl::AABB<Eigen::MatrixXd, 3> tree;
  igl::embree::EmbreeIntersector intersector;

  bool initialized = false;
  void init(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

  // All-pair shortest path
  // A proxy for geodesic distance
  bool SP_valid = false;
  Eigen::MatrixXd SP;
  Eigen::MatrixXi SP_par;
  void init_sp();

  // out_c: closest point
  // out_s: sign
  double ComputeSignedDistance(const Eigen::Vector3d& position,
                               Eigen::RowVector3d& out_c,
                               double& out_s) const;

  size_t ComputeClosestFacet(const Eigen::Vector3d& position) const;

  size_t ComputeClosestVertex(const Eigen::Vector3d& position) const;

  // Returns the length of non-intersecting path from A to B
  // minus the displacement from A to B.
  double ComputeRequiredDistance(const Eigen::Vector3d& A,
                                 const Eigen::Vector3d& B,
                                 Debugger *const debugger) const;

  DECL_SERIALIZE() {
    constexpr int version = 1;
    SERIALIZE(version);
    SERIALIZE(V);
    SERIALIZE(F);
  }

  DECL_DESERIALIZE() {
    int version;
    DESERIALIZE(version);
    Eigen::MatrixXd V_;
    Eigen::MatrixXi F_;
    if (version == 1) {
      DESERIALIZE(V_);
      DESERIALIZE(F_);
      init(V_, F_);
    }
  }
};

}  // namespace models
}  // namespace core
}  // namespace psg
