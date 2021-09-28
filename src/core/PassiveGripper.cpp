#include "PassiveGripper.h"

#include "CostFunctions.h"
#include "GeometryUtils.h"
#include "Initialization.h"
#include "QualityMetric.h"
#include "robots/Robots.h"

namespace psg {

// Mesh

ViewModel::ViewModel() {
  gripper_.params.trajectory.push_back(kInitPose);
}

void ViewModel::SetMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
  mdr_.init(V, F);
  mesh_changed_ = true;
  mesh_loaded_ = true;
  Invalidate();
}

// Contact Points

void ViewModel::AddContactPoint(const ContactPoint& contact_point) {
  gripper_.contact_points.push_back(contact_point);

  Eigen::Affine3d effector_pos =
      robots::Forward(gripper_.params.trajectory.front());

  Eigen::MatrixXd&& finger =
      InitializeFinger(contact_point,
                       mdr_,
                       effector_pos.translation(),
                       gripper_.finger_settings.n_finger_joints);
  gripper_.params.fingers.push_back(finger);
  std::vector<ContactPoint>&& cone =
      GenerateContactCone(contact_point,
                          gripper_.contact_settings.cone_res,
                          gripper_.contact_settings.friction);
  contact_cones_.insert(contact_cones_.end(), cone.begin(), cone.end());

  contact_changed_ = true;
  Invalidate();
}

void ViewModel::RemoveContactPoint(size_t index) {
  gripper_.contact_points.erase(gripper_.contact_points.begin() + index);
  gripper_.params.fingers.erase(gripper_.params.fingers.begin() + index);
  contact_cones_.erase(
      contact_cones_.begin() + (index * gripper_.contact_settings.cone_res),
      contact_cones_.begin() +
          ((index + 1) * gripper_.contact_settings.cone_res));
  contact_changed_ = true;
  Invalidate();
}

void ViewModel::ClearContactPoint() {
  gripper_.contact_points.clear();
  gripper_.params.fingers.clear();
  contact_cones_.clear();
  contact_changed_ = true;
  Invalidate();
}

// Trajectory
void ViewModel::AddKeyframe(const Pose& pose) {
  gripper_.params.trajectory.push_back(pose);
  trajectory_changed_ = true;
  Invalidate();
}
void ViewModel::EditKeyframe(size_t index, const Pose& pose) {
  gripper_.params.trajectory[index] = pose;
  trajectory_changed_ = true;
  Invalidate();
}
void ViewModel::RemoveKeyframe(size_t index) {
  if (gripper_.params.trajectory.size() <= 1) return;
  gripper_.params.trajectory.erase(gripper_.params.trajectory.begin() + index);
  trajectory_changed_ = true;
  Invalidate();
}
void ViewModel::ClearKeyframe() {
  Pose front = gripper_.params.trajectory.front();
  gripper_.params.trajectory.clear();
  gripper_.params.trajectory.push_back(front);
  trajectory_changed_ = true;
  Invalidate();
}

void ViewModel::SetContactSettings(const ContactSettings& settings) {
  gripper_.contact_settings = settings;
  contact_settings_changed_ = true;
  Invalidate();
}

void ViewModel::SetFingerSettings(const FingerSettings& settings) {
  gripper_.finger_settings = settings;
  finger_settings_changed_ = true;
  Invalidate();
}

void ViewModel::SetTrajectorySettings(const TrajectorySettings& settings) {
  gripper_.trajectory_settings = settings;
  trajectory_settings_changed_ = true;
  Invalidate();
}

void ViewModel::SetOptSettings(const OptSettings& settings) {
  gripper_.opt_settings = settings;
  opt_settings_changed_ = true;
  Invalidate();
}

void ViewModel::SetTopoOptSettings(const TopoOptSettings& settings) {
  gripper_.topo_opt_settings = settings;
  topo_opt_settings_changed_ = true;
  Invalidate();
}

void ViewModel::SetCostSettings(const CostSettings& settings) {
  gripper_.cost_settings = settings;
  cost_settings_changed_ = true;
  Invalidate();
}

// State Invalidation

// [] -> [Mesh]
// [] -> [Settings]
// [Mesh, ContactSettings] -> [Contact]
// [Contact] -> [Quality]
// [Contact, FingerSettings] -> [Finger]
// [Finger, TrajectorySettings] -> [Trajectory]
// [Finger, Trajectory, CostSettings] -> [Cost]

void ViewModel::Invalidate() {
  if (mesh_changed_) InvalidateMesh();
  if (contact_settings_changed_) InvalidateContactSettings();
  if (contact_changed_) InvalidateContact();
  if (finger_settings_changed_) InvalidateFingerSettings();
  if (trajectory_settings_changed_) InvalidateTrajectorySettings();
  if (cost_settings_changed_) InvalidateCostSettings();
  if (finger_changed_) InvalidateFinger();
  if (trajectory_changed_) InvalidateTrajectory();
  if (quality_changed_) InvalidateQuality();
  if (cost_changed_) InvalidateCost();
}

void ViewModel::ForceInvalidateAll() {
  mesh_changed_ = true;
  finger_settings_changed_ = true;
  trajectory_settings_changed_ = true;
  cost_settings_changed_ = true;
  Invalidate();
}

void ViewModel::InvokeLayerInvalidated(Layers layer) {
  if (LayerInvalidated_) LayerInvalidated_(layer);
}

void ViewModel::InvalidateMesh() {
  mesh_changed_ = false;
  gripper_.contact_points.clear();
  gripper_.params.fingers.clear();
  InvokeLayerInvalidated(Layers::kMesh);
  InvokeLayerInvalidated(Layers::kCenterOfMass);
  contact_changed_ = true;
}

void ViewModel::InvalidateContactSettings() {
  contact_settings_changed_ = false;
  contact_cones_.clear();
  contact_cones_.reserve(gripper_.contact_settings.cone_res *
                         gripper_.contact_points.size());
  for (size_t i = 0; i < gripper_.contact_points.size(); i++) {
    std::vector<ContactPoint>&& cones =
        GenerateContactCone(gripper_.contact_points[i],
                            gripper_.contact_settings.cone_res,
                            gripper_.contact_settings.friction);  
    contact_cones_.insert(contact_cones_.end(), cones.begin(), cones.end());
  }
  contact_changed_ = true;
}

void ViewModel::InvalidateFingerSettings() {
  finger_settings_changed_ = false;
  // Re-initialize finger
  Eigen::Affine3d effector_pos =
      robots::Forward(gripper_.params.trajectory.front());
  for (size_t i = 0; i < gripper_.contact_points.size(); i++) {
    Eigen::MatrixXd&& finger =
        InitializeFinger(gripper_.contact_points[i],
                         mdr_,
                         effector_pos.translation(),
                         gripper_.finger_settings.n_finger_joints);
    gripper_.params.fingers[i] = finger;
  }
  finger_changed_ = true;
}

void ViewModel::InvalidateTrajectorySettings() {
  trajectory_settings_changed_ = false;
  if (reinit_trajectory) {
    trajectory_changed_ = true;
  }
}

void ViewModel::InvalidateCostSettings() {
  cost_settings_changed_ = false;
  cost_changed_ = true;
}

void ViewModel::InvalidateContact() {
  contact_changed_ = false;
  InvokeLayerInvalidated(Layers::kContactPoints);
  finger_changed_ = true;
  quality_changed_ = true;
}

void ViewModel::InvalidateFinger() {
  finger_changed_ = false;
  InvokeLayerInvalidated(Layers::kFingers);
  if (reinit_trajectory) {
    trajectory_changed_ = true;
  }
  cost_changed_ = true;
}

void ViewModel::InvalidateTrajectory() {
  trajectory_changed_ = false;
  if (reinit_trajectory) {
    // Re-initialize trajectory
    gripper_.params.trajectory =
        InitializeTrajectory(gripper_.params.fingers,
                             gripper_.params.trajectory.front(),
                             gripper_.trajectory_settings.n_keyframes);
  }
  InvokeLayerInvalidated(Layers::kTrajectory);
  cost_changed_ = true;
}

void ViewModel::InvalidateQuality() {
  quality_changed_ = false;
  is_force_closure_ = CheckForceClosureQP(contact_cones_, mdr_.center_of_mass);
  is_partial_closure_ = CheckPartialClosureQP(contact_cones_,
                                              mdr_.center_of_mass,
                                              -Eigen::Vector3d::UnitY(),
                                              Eigen::Vector3d::Zero());
  min_wrench_ = ComputeMinWrenchQP(contact_cones_, mdr_.center_of_mass);
  partial_min_wrench_ = ComputePartialMinWrenchQP(contact_cones_,
                                                  mdr_.center_of_mass,
                                                  -Eigen::Vector3d::UnitY(),
                                                  Eigen::Vector3d::Zero());
}

void ViewModel::InvalidateCost() {
  cost_changed_ = false;
  cost_ = ComputeCost(gripper_, mdr_);
}

}  // namespace psg