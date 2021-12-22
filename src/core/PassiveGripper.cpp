#include "PassiveGripper.h"

#include "CostFunctions.h"
#include "GeometryUtils.h"
#include "Initialization.h"
#include "QualityMetric.h"
#include "robots/Robots.h"

namespace psg {
namespace core {

// Mesh

PassiveGripper::PassiveGripper() {
  params_.trajectory.push_back(kInitPose);
}

void PassiveGripper::SetMesh(const Eigen::MatrixXd& V,
                             const Eigen::MatrixXi& F,
                             bool invalidate) {
  mdr_.init(V, F);
  mesh_changed_ = true;
  mesh_loaded_ = true;
  if (invalidate) Invalidate();
}

// Contact Points

void PassiveGripper::SetMeshTrans(const Eigen::Affine3d& trans) {
  mesh_trans_ = trans;
}

void PassiveGripper::TransformMesh(const Eigen::Affine3d& trans) {
  Eigen::MatrixXd V = mdr_.V;
  Eigen::MatrixXi F = mdr_.F;
  V = (trans * V.transpose().colwise().homogeneous()).transpose();
  mdr_.init(V, F);
  mesh_trans_ = trans * mesh_trans_;
  mesh_changed_ = true;
  Invalidate();
}

void PassiveGripper::AddContactPoint(const ContactPoint& contact_point) {
  params_.contact_points.push_back(contact_point);

  Eigen::Affine3d effector_pos = robots::Forward(params_.trajectory.front());

  Eigen::MatrixXd&& finger = InitializeFinger(contact_point,
                                              mdr_,
                                              effector_pos.translation(),
                                              settings_.finger.n_finger_joints);
  params_.fingers.push_back(finger);
  std::vector<ContactPoint>&& cone = GenerateContactCone(
      contact_point, settings_.contact.cone_res, settings_.contact.friction);
  contact_cones_.insert(contact_cones_.end(), cone.begin(), cone.end());

  contact_changed_ = true;
  Invalidate();
}

void PassiveGripper::SetContactPoints(
    const std::vector<ContactPoint>& contact_points) {
  params_.contact_points = contact_points;
  params_.fingers.clear();
  contact_cones_.clear();
  Eigen::Affine3d effector_pos = robots::Forward(params_.trajectory.front());

  for (size_t i = 0; i < contact_points.size(); i++) {
    Eigen::MatrixXd&& finger =
        InitializeFinger(contact_points[i],
                         mdr_,
                         effector_pos.translation(),
                         settings_.finger.n_finger_joints);
    params_.fingers.push_back(finger);
    std::vector<ContactPoint>&& cone =
        GenerateContactCone(contact_points[i],
                            settings_.contact.cone_res,
                            settings_.contact.friction);
    contact_cones_.insert(contact_cones_.end(), cone.begin(), cone.end());
  }

  contact_changed_ = true;
  Invalidate();
}

void PassiveGripper::RemoveContactPoint(size_t index) {
  params_.contact_points.erase(params_.contact_points.begin() + index);
  params_.fingers.erase(params_.fingers.begin() + index);
  contact_cones_.erase(
      contact_cones_.begin() + (index * settings_.contact.cone_res),
      contact_cones_.begin() + ((index + 1) * settings_.contact.cone_res));
  contact_changed_ = true;
  Invalidate();
}

void PassiveGripper::ClearContactPoint() {
  params_.contact_points.clear();
  params_.fingers.clear();
  contact_cones_.clear();
  contact_changed_ = true;
  Invalidate();
}

// Trajectory
void PassiveGripper::AddKeyframe(const Pose& pose) {
  params_.trajectory.push_back(pose);
  FixTrajectory(params_.trajectory);
  trajectory_changed_ = true;
  Invalidate();
}
void PassiveGripper::EditKeyframe(size_t index, const Pose& pose) {
  params_.trajectory[index] = pose;
  FixTrajectory(params_.trajectory);
  trajectory_changed_ = true;
  if (index == 0) finger_settings_changed_ = true;
  Invalidate();
}
void PassiveGripper::RemoveKeyframe(size_t index) {
  if (params_.trajectory.size() <= 1) return;
  params_.trajectory.erase(params_.trajectory.begin() + index);
  FixTrajectory(params_.trajectory);
  trajectory_changed_ = true;
  Invalidate();
}
void PassiveGripper::ClearKeyframe() {
  Pose front = params_.trajectory.front();
  params_.trajectory.clear();
  params_.trajectory.push_back(front);
  trajectory_changed_ = true;
  Invalidate();
}

void PassiveGripper::SetContactSettings(const ContactSettings& settings) {
  settings_.contact = settings;
  contact_settings_changed_ = true;
  Invalidate();
}

void PassiveGripper::SetFingerSettings(const FingerSettings& settings) {
  settings_.finger = settings;
  finger_settings_changed_ = true;
  Invalidate();
}

void PassiveGripper::SetTrajectorySettings(const TrajectorySettings& settings) {
  settings_.trajectory = settings;
  trajectory_settings_changed_ = true;
  Invalidate();
}

void PassiveGripper::SetOptSettings(const OptSettings& settings) {
  settings_.opt = settings;
  opt_settings_changed_ = true;
  Invalidate();
}

void PassiveGripper::SetTopoOptSettings(const TopoOptSettings& settings) {
  settings_.topo_opt = settings;
  topo_opt_settings_changed_ = true;
  Invalidate();
}

void PassiveGripper::SetCostSettings(const CostSettings& settings) {
  settings_.cost = settings;
  cost_settings_changed_ = true;
  Invalidate();
}

void PassiveGripper::SetSettings(const GripperSettings& settings,
                                 bool invalidate) {
  settings_ = settings;
  contact_settings_changed_ = true;
  finger_settings_changed_ = true;
  trajectory_settings_changed_ = true;
  opt_settings_changed_ = true;
  topo_opt_settings_changed_ = true;
  cost_settings_changed_ = true;
  if (invalidate) Invalidate();
}

void PassiveGripper::SetParams(const GripperParams& params, bool invalidate) {
  bool tmp_reinit_finger = reinit_fingers;
  bool tmp_reinit_trajectory = reinit_trajectory;
  reinit_fingers = false;
  reinit_trajectory = false;
  params_ = params;
  contact_cones_.clear();
  for (size_t i = 0; i < params_.contact_points.size(); i++) {
    std::vector<ContactPoint>&& cone =
        GenerateContactCone(params_.contact_points[i],
                            settings_.contact.cone_res,
                            settings_.contact.friction);
    contact_cones_.insert(contact_cones_.end(), cone.begin(), cone.end());
  }
  contact_changed_ = true;
  finger_changed_ = true;
  trajectory_changed_ = true;
  if (invalidate) Invalidate();
  reinit_fingers = tmp_reinit_finger;
  reinit_trajectory = tmp_reinit_trajectory;
}

void PassiveGripper::InitGripperBound() {
  Eigen::Vector3d lb;
  Eigen::Vector3d ub;
  InitializeGripperBound(*this, lb, ub);
  settings_.topo_opt.lower_bound = lb;
  settings_.topo_opt.upper_bound = ub;
  topo_opt_settings_changed_ = true;
  Invalidate();
}

// State Invalidation

// [] -> [Mesh]
// [] -> [Settings]
// [Mesh, ContactSettings] -> [Contact]
// [Contact] -> [Quality]
// [Contact, FingerSettings] -> [Finger]
// [Finger, TrajectorySettings] -> [Trajectory]
// [] -> [TopoOptSettings]
// [Finger, Trajectory, CostSettings] -> [Cost]

void PassiveGripper::Invalidate() {
  if (mesh_changed_) InvalidateMesh();
  if (contact_settings_changed_) InvalidateContactSettings();
  if (contact_changed_) InvalidateContact();
  if (finger_settings_changed_) InvalidateFingerSettings();
  if (trajectory_settings_changed_) InvalidateTrajectorySettings();
  if (cost_settings_changed_) InvalidateCostSettings();
  if (finger_changed_) InvalidateFinger();
  if (trajectory_changed_) InvalidateTrajectory();
  if (topo_opt_settings_changed_) InvalidateTopoOptSettings();
  if (quality_changed_) InvalidateQuality();
  if (cost_changed_) InvalidateCost();
}

void PassiveGripper::ForceInvalidateAll(bool disable_reinit) {
  bool tmp_reinit_finger;
  bool tmp_reinit_trajectory;
  if (disable_reinit) {
    tmp_reinit_finger = reinit_fingers;
    tmp_reinit_trajectory = reinit_trajectory;
  }
  mesh_changed_ = true;
  finger_settings_changed_ = true;
  trajectory_settings_changed_ = true;
  cost_settings_changed_ = true;
  Invalidate();
  if (disable_reinit) {
    reinit_fingers = tmp_reinit_finger;
    reinit_trajectory = tmp_reinit_trajectory;
  }
}

void PassiveGripper::InvokeInvalidated(InvalidatedReason reason) {
  if (Invalidated_) Invalidated_(reason);
}

void PassiveGripper::InvalidateMesh() {
  mesh_changed_ = false;
  params_.contact_points.clear();
  params_.fingers.clear();
  InvokeInvalidated(InvalidatedReason::kMesh);
  contact_changed_ = true;
}

void PassiveGripper::InvalidateContactSettings() {
  contact_settings_changed_ = false;
  contact_cones_.clear();
  contact_cones_.reserve(settings_.contact.cone_res *
                         params_.contact_points.size());
  for (size_t i = 0; i < params_.contact_points.size(); i++) {
    std::vector<ContactPoint>&& cones =
        GenerateContactCone(params_.contact_points[i],
                            settings_.contact.cone_res,
                            settings_.contact.friction);
    contact_cones_.insert(contact_cones_.end(), cones.begin(), cones.end());
  }
  contact_changed_ = true;
}

void PassiveGripper::InvalidateFingerSettings() {
  finger_settings_changed_ = false;
  // Re-initialize finger
  if (reinit_fingers) {
    Eigen::Affine3d effector_pos = robots::Forward(params_.trajectory.front());
    for (size_t i = 0; i < params_.contact_points.size(); i++) {
      Eigen::MatrixXd&& finger =
          InitializeFinger(params_.contact_points[i],
                           mdr_,
                           effector_pos.translation(),
                           settings_.finger.n_finger_joints);
      params_.fingers[i] = finger;
    }
    finger_changed_ = true;
  }
}

void PassiveGripper::InvalidateTrajectorySettings() {
  trajectory_settings_changed_ = false;
  if (reinit_trajectory) {
    trajectory_changed_ = true;
  }
}

void PassiveGripper::InvalidateCostSettings() {
  cost_settings_changed_ = false;
  cost_changed_ = true;
}

void PassiveGripper::InvalidateContact() {
  contact_changed_ = false;
  InvokeInvalidated(InvalidatedReason::kContactPoints);
  finger_changed_ = true;
  quality_changed_ = true;
}

void PassiveGripper::InvalidateFinger() {
  finger_changed_ = false;
  InvokeInvalidated(InvalidatedReason::kFingers);
  if (reinit_trajectory) {
    trajectory_changed_ = true;
  }
  cost_changed_ = true;
}

void PassiveGripper::InvalidateTrajectory() {
  trajectory_changed_ = false;
  if (reinit_trajectory) {
    // Re-initialize trajectory
    params_.trajectory = InitializeTrajectory(params_.fingers,
                                              params_.trajectory.front(),
                                              settings_.trajectory.n_keyframes);
  }
  InvokeInvalidated(InvalidatedReason::kTrajectory);
  cost_changed_ = true;
}

void PassiveGripper::InvalidateTopoOptSettings() {
  topo_opt_settings_changed_ = false;
  InvokeInvalidated(InvalidatedReason::kTopoOptSettings);
}

void PassiveGripper::InvalidateQuality() {
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

void PassiveGripper::InvalidateCost() {
  cost_changed_ = false;
  cost_ = ComputeCost(params_, settings_, mdr_, dCost_dParam_);
  min_dist_ = MinDistance(params_, settings_, mdr_);
  intersecting_ = Intersects(params_, settings_, mdr_);
  InvokeInvalidated(InvalidatedReason::kCost);
}

}  // namespace core
}  // namespace psg