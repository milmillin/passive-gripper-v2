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

// Stores the information derived from the object's mesh
class MeshDependentResource : psg::core::serialization::Serializable {
 public:
  Eigen::MatrixXd V;     // Vertices (number of vertices * dimension)
  Eigen::MatrixXi F;     // Triangles (number of faces * 3)
  Eigen::MatrixXd FN;    // Per-face norm (number of faces * 3)
  Eigen::MatrixXd VN;    // Per-vertex norm (number of vertices * 3)
  Eigen::MatrixXd EN;    // Per-edge norm (number of edges * 3)
  Eigen::MatrixXi E;     // Edges (number of edges * 2)
  Eigen::MatrixXi EMAP;  // Indices from all edges to E (number of edges * 1)
  Eigen::Vector3d center_of_mass;      // Estimated center of mass
  Eigen::Vector3d minimum;             // Minimum coordinate
  Eigen::Vector3d maximum;             // Maximum coordinate
  Eigen::Vector3d size;                // Range of coordinate (max - min)
  igl::AABB<Eigen::MatrixXd, 3> tree;  // AABB tree for the mesh
  igl::embree::EmbreeIntersector intersector;  // Intersection for the mesh

  // curvature
  Eigen::MatrixXd PD1, PD2;
  Eigen::VectorXd PV1, PV2;

 private:
  // All-pair shortest path
  // A proxy for geodesic distance
  mutable bool SP_valid_ = false;
  mutable Eigen::MatrixXd SP_;
  mutable Eigen::MatrixXi SP_par_;
  mutable std::mutex SP_mutex_;
  void init_sp() const;

  // Curvature
  /*
  mutable bool curvature_valid_ = false;
  mutable Eigen::VectorXd curvature_;
  mutable std::mutex curvature_mutex_;
  void init_curvature() const;
  */

  bool initialized = false;

 public:
  void init(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
  void init(const MeshDependentResource& other);

  // Return signed distance
  // position: the point to compute
  // out_c: closest point on mesh
  // out_s: sign
  double ComputeSignedDistance(const Eigen::Vector3d& position,
                               Eigen::RowVector3d& out_c,
                               double& out_s) const;

  // position: the point to compute
  // out_c: closest point on mesh
  // out_fid: index of face that contains the closest point
  void ComputeClosestPoint(const Eigen::Vector3d& position,
                           Eigen::RowVector3d& out_c,
                           int& out_fid) const;

  // Returns the index of the closest face
  size_t ComputeClosestFacet(const Eigen::Vector3d& position) const;

  // Returns the index of the closest vertex
  size_t ComputeClosestVertex(const Eigen::Vector3d& position) const;

  // Returns the length of non-intersecting path from A to B
  // minus the displacement from A to B.
  double ComputeRequiredDistance(const Eigen::Vector3d& A,
                                 const Eigen::Vector3d& B,
                                 Debugger* const debugger) const;

  // Returns whether the box intersects with the mesh
  bool Intersects(const Eigen::AlignedBox3d box) const;

  // Getters
  const Eigen::MatrixXd& GetSP() const;
  const Eigen::MatrixXi& GetSPPar() const;
  // const Eigen::VectorXd& GetCurvature() const;

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
