#include "ViewModel.h"

#include "../core/robots/Robots.h"
#include "../core/GeometryUtils.h"

namespace psg {
namespace ui {

ViewModel::ViewModel() {
  using namespace std::placeholders;
  psg_.RegisterInvalidatedDelegate(
      std::bind(&ViewModel::OnPsgInvalidated, this, _1));

  SetCurrentPose(kInitPose);
}

void ViewModel::SetCurrentPose(const Pose& pose) {
  current_pose_ = pose;
  Eigen::Affine3d trans = robots::Forward(current_pose_);
  eff_position_ = trans.translation();
  eff_angles_ = trans.linear().eulerAngles(1, 0, 2);
  std::swap(eff_angles_(1), eff_angles_(0));
  ComputeIK();
  PoseChanged();
}

void ViewModel::SetCurrentPose(const Eigen::Affine3d& trans) {
  eff_position_ = trans.translation();
  eff_angles_ = trans.linear().eulerAngles(1, 0, 2);
  std::swap(eff_angles_(1), eff_angles_(0));
  ComputeIK();
  PoseChanged();
}

void ViewModel::SetCurrentPose(const Eigen::Vector3d& pos,
                               const Eigen::Vector3d& ang) {
  eff_position_ = pos;
  eff_angles_ = ang;
  ComputeIK();
  PoseChanged();
}

void ViewModel::ComputeIK() {
  Eigen::Affine3d trans =
      Eigen::Translation3d(eff_position_) *
      Eigen::AngleAxisd(eff_angles_(1), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(eff_angles_(0), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(eff_angles_(2), Eigen::Vector3d::UnitZ());

  if (robots::Inverse(trans, ik_sols_)) {
    // Find solution closest to previous solution
    const Pose& prev = current_pose_;
    size_t best = 0;
    double diff = std::numeric_limits<double>::max();
    for (size_t i = 0; i < ik_sols_.size(); i++) {
      double curDiff = SumSquaredAngularDistance(prev, ik_sols_[i]);
      if (curDiff < diff) {
        diff = curDiff;
        best = i;
      }
    }
    current_pose_ = ik_sols_[best];
    ik_sols_index_ = best;
  } else {
    ik_sols_index_ = -1;  
  }
}

void ViewModel::InvokeLayerInvalidated(Layer layer) {
  if (LayerInvalidated_) LayerInvalidated_(layer);
}

void ViewModel::OnPsgInvalidated(PassiveGripper::InvalidatedReason reason) {
  switch (reason) {
    case PassiveGripper::InvalidatedReason::kMesh:
      InvokeLayerInvalidated(Layer::kMesh);
      InvokeLayerInvalidated(Layer::kCenterOfMass);
      break;
    case PassiveGripper::InvalidatedReason::kContactPoints:
      InvokeLayerInvalidated(Layer::kContactPoints);
      break;
    case PassiveGripper::InvalidatedReason::kFingers:
      InvokeLayerInvalidated(Layer::kFingers);
      break;
    case PassiveGripper::InvalidatedReason::kTrajectory:
      InvokeLayerInvalidated(Layer::kTrajectory);
      break;
  }
}

void ViewModel::PoseChanged() {
  InvokeLayerInvalidated(Layer::kFingers);
  InvokeLayerInvalidated(Layer::kRobot);
}

}  // namespace ui
}  // namespace psg
