#pragma once

#include <Eigen/Core>
#include <functional>
#include <vector>

#include "models/ContactPoint.h"
#include "models/GripperSettings.h"
#include "models/MeshDependentResource.h"
#include "models/GripperParams.h"

namespace psg {
namespace core {

using namespace models;

class PassiveGripper {
 public:
  enum class InvalidatedReason { kMesh, kContactPoints, kFingers, kTrajectory };
  typedef std::function<void(InvalidatedReason)>
      InvalidatedDelegate;

  PassiveGripper();

  inline void RegisterInvalidatedDelegate(const InvalidatedDelegate& d) {
    Invalidated_ = d;
  }

  // Mesh
  void SetMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
  inline void GetMesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F) const {
    V = mdr_.V;
    F = mdr_.F;
  }
  inline const Eigen::Vector3d& GetCenterOfMass() const {
    return mdr_.center_of_mass;
  }
  inline bool IsMeshLoaded() const { return mesh_loaded_; }

  // Contact Point
  void AddContactPoint(const ContactPoint& contact_point);
  void RemoveContactPoint(size_t index);
  void ClearContactPoint();
  inline const std::vector<ContactPoint>& GetContactPoints() const {
    return params_.contact_points;
  }
  inline const std::vector<ContactPoint>& GetContactCones() const {
    return contact_cones_;
  }

  // Fingers
  inline const std::vector<Eigen::MatrixXd>& GetFingers() const {
    return params_.fingers;
  }

  // Trajectory
  void AddKeyframe(const Pose& pose);
  void EditKeyframe(size_t index, const Pose& pose);
  void RemoveKeyframe(size_t index);
  void ClearKeyframe();
  inline const Trajectory& GetTrajectory() const { return params_.trajectory; }

  // Settings
  void SetContactSettings(const ContactSettings& settings);
  void SetFingerSettings(const FingerSettings& finger_settings);
  void SetTrajectorySettings(const TrajectorySettings& finger_settings);
  void SetOptSettings(const OptSettings& finger_settings);
  void SetTopoOptSettings(const TopoOptSettings& finger_settings);
  void SetCostSettings(const CostSettings& finger_settings);
  inline const ContactSettings& GetContactSettings() const {
    return settings_.contact;
  }
  inline const FingerSettings& GetFingerSettings() const {
    return settings_.finger;
  }
  inline const TrajectorySettings& GetTrajectorySettings() const {
    return settings_.trajectory;
  }
  inline const OptSettings& GetOptSettings() const { return settings_.opt; }
  inline const TopoOptSettings& GetTopoOptSettings() const {
    return settings_.topo_opt;
  }
  inline const CostSettings& GetCostSettings() const { return settings_.cost; }

  // Quality Metric
  inline bool GetIsForceClosure() const { return is_force_closure_; }
  inline bool GetIsPartialClosure() const { return is_partial_closure_; }
  inline double GetMinWrench() const { return min_wrench_; }
  inline double GetPartialMinWrench() const { return partial_min_wrench_; }

  // Cost
  inline double GetCost() const { return cost_; }

  bool reinit_trajectory = true;

 private:
  GripperParams params_;
  GripperSettings settings_;
  MeshDependentResource mdr_;
  std::vector<ContactPoint> contact_cones_;

  bool mesh_loaded_ = false;

  bool is_force_closure_;
  bool is_partial_closure_;
  double min_wrench_;
  double partial_min_wrench_;
  double cost_;

  // state dependency
  bool mesh_changed_ = false;
  bool contact_settings_changed_ = false;
  bool finger_settings_changed_ = false;
  bool trajectory_settings_changed_ = false;
  bool opt_settings_changed_ = false;
  bool topo_opt_settings_changed_ = false;
  bool cost_settings_changed_ = false;
  bool contact_changed_ = false;
  bool finger_changed_ = false;
  bool trajectory_changed_ = false;
  bool quality_changed_ = false;
  bool cost_changed_ = false;

  InvalidatedDelegate Invalidated_;

  void Invalidate();
  void ForceInvalidateAll();
  void InvokeInvalidated(InvalidatedReason reason);

  // To be called by Invalidate()
  void InvalidateMesh();
  void InvalidateContactSettings();
  void InvalidateFingerSettings();
  void InvalidateTrajectorySettings();
  void InvalidateCostSettings();
  void InvalidateContact();
  void InvalidateFinger();
  void InvalidateTrajectory();
  void InvalidateQuality();
  void InvalidateCost();
};

}  // namespace core
}  // namespace psg