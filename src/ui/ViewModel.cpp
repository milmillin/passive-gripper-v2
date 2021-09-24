#include "ViewModel.h"

#include "../core/CostFunctions.h"
#include "../core/GeometryUtils.h"
#include "../core/Initialization.h"
#include "../core/QualityMetric.h"
#include "../core/robots/Robots.h"

namespace psg {

// Mesh

ViewModel::ViewModel() {
  gripper_.params.trajectory.push_back(kInitPose);
}

void ViewModel::SetMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
  mdr_.init(V, F);
  mesh_changed_ = true;
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
                       gripper_.settings.n_finger_joints);
  gripper_.params.fingers.push_back(finger);
  std::vector<ContactPoint>&& cone = GenerateContactCone(
      contact_point, gripper_.settings.cone_res, gripper_.settings.friction);
  contact_cones_.insert(contact_cones_.end(), cone.begin(), cone.end());

  contact_changed_ = true;
  Invalidate();
}

void ViewModel::RemoveContactPoint(size_t index) {
  gripper_.contact_points.erase(gripper_.contact_points.begin() + index);
  gripper_.params.fingers.erase(gripper_.params.fingers.begin() + index);
  contact_cones_.erase(
      contact_cones_.begin() + (index * gripper_.settings.cone_res),
      contact_cones_.begin() + ((index + 1) * gripper_.settings.cone_res));
  contact_changed_= true;
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

// Settings
void ViewModel::SetSettings(const Settings& settings) {
  gripper_.settings = settings;
  settings_changed_ = true;
  Invalidate();
}

// State Invalidation

// [] -> [Mesh]
// [] -> [Settings]
// [Mesh] -> [Contact]
// [Contact] -> [Quality]
// [Contact, Settings] -> [Finger]
// [Finger] -> [Trajectory]
// [Finger, Trajectory] -> [Cost]

void ViewModel::Invalidate() {
  if (mesh_changed_) InvalidateMesh();
  if (contact_changed_) InvalidateContact();
  if (settings_changed_) InvalidateSettings();
  if (finger_changed_) InvalidateFinger();
  if (trajectory_changed_) InvalidateTrajectory();
  if (quality_changed_) InvalidateQuality();
  if (cost_changed_) InvalidateCost();
}

void ViewModel::ForceInvalidateAll() {
  mesh_changed_ = true;
  settings_changed_ = true;
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
  contact_changed_ = true;
}

void ViewModel::InvalidateSettings() {
  settings_changed_ = false;
  // Re-initialize finger
  Eigen::Affine3d effector_pos =
      robots::Forward(gripper_.params.trajectory.front());
  for (size_t i = 0; i < gripper_.contact_points.size(); i++) {
    Eigen::MatrixXd&& finger =
        InitializeFinger(gripper_.contact_points[i],
                         mdr_,
                         effector_pos.translation(),
                         gripper_.settings.n_finger_joints);
    gripper_.params.fingers[i] = finger;
  }
  finger_changed_ = true;
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
    // Re-initialize trajectory
    InitializeTrajectory(gripper_.params.fingers,
                         gripper_.params.trajectory.front(),
                         gripper_.settings.n_keyframes);
    trajectory_changed_ = true;
  }
  cost_changed_ = true;
}

void ViewModel::InvalidateTrajectory() {
  trajectory_changed_ = false;
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
  min_wrench_ = ComputePartialMinWrenchQP(contact_cones_,
                                          mdr_.center_of_mass,
                                          -Eigen::Vector3d::UnitY(),
                                          Eigen::Vector3d::Zero());
}

void ViewModel::InvalidateCost() {
  cost_changed_ = false;
  cost_ = ComputeCost(gripper_, mdr_);
}

}  // namespace psg