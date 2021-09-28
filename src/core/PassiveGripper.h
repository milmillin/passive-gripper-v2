#pragma once

#include <Eigen/Core>
#include <functional>
#include <vector>

#include "../core/models/ContactPoint.h"
#include "../core/models/GripperSettings.h"
#include "../core/models/MeshDependentResource.h"

namespace psg {

using namespace models;

class PassiveGripper {
 public:
  typedef std::function<void(Layers)> LayerInvalidatedDelegate;

  PassiveGripper();

  inline void RegisterLayerInvalidatedDelegate(
      const LayerInvalidatedDelegate& d) {
    LayerInvalidated_ = d;
    ForceInvalidateAll();
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
    return gripper_.contact_points;
  }
  inline const std::vector<ContactPoint>& GetContactCones() const {
    return contact_cones_;
  }

  // Fingers
  inline const std::vector<Eigen::MatrixXd>& GetFingers() const {
    return gripper_.params.fingers;
  }

  // Trajectory
  void AddKeyframe(const Pose& pose);
  void EditKeyframe(size_t index, const Pose& pose);
  void RemoveKeyframe(size_t index);
  void ClearKeyframe();
  inline const Trajectory& GetTrajectory() const {
    return gripper_.params.trajectory;
  }

  // Settings
  void SetContactSettings(const ContactSettings& settings);
  void SetFingerSettings(const FingerSettings& finger_settings);
  void SetTrajectorySettings(const TrajectorySettings& finger_settings);
  void SetOptSettings(const OptSettings& finger_settings);
  void SetTopoOptSettings(const TopoOptSettings& finger_settings);
  void SetCostSettings(const CostSettings& finger_settings);
  inline const ContactSettings& GetContactSettings() const {
    return gripper_.contact_settings;
  }
  inline const FingerSettings& GetFingerSettings() const {
    return gripper_.finger_settings;
  }
  inline const TrajectorySettings& GetTrajectorySettings() const {
    return gripper_.trajectory_settings;
  }
  inline const OptSettings& GetOptSettings() const {
    return gripper_.opt_settings;
  }
  inline const TopoOptSettings& GetTopoOptSettings() const {
    return gripper_.topo_opt_settings;
  }
  inline const CostSettings& GetCostSettings() const {
    return gripper_.cost_settings;
  }

  // Quality Metric
  inline bool GetIsForceClosure() const { return is_force_closure_; }
  inline bool GetIsPartialClosure() const { return is_partial_closure_; }
  inline double GetMinWrench() const { return min_wrench_; }
  inline double GetPartialMinWrench() const { return partial_min_wrench_; }

  // Cost
  inline double GetCost() const { return cost_; }

  // Robot Pose
  inline const Pose& GetCurrentPose() const { return current_pose_; }

  bool reinit_trajectory = true;

 private:
  Gripper gripper_;
  MeshDependentResource mdr_;
  LayerInvalidatedDelegate LayerInvalidated_;
  std::vector<ContactPoint> contact_cones_;

  bool mesh_loaded_ = false;

  bool is_force_closure_;
  bool is_partial_closure_;
  double min_wrench_;
  double partial_min_wrench_;
  double cost_;

  Pose current_pose_ = kInitPose;

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

  void Invalidate();
  void ForceInvalidateAll();
  void InvokeLayerInvalidated(Layers layer);

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

}  // namespace psg