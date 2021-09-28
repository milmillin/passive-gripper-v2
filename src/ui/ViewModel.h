#pragma once

#include "../core/PassiveGripper.h"
#include "Layer.h"

#define DECLARE_GETTER(x, y) \
  inline const decltype(y)& x() const { return y; }

using namespace psg::core;

namespace psg {
namespace ui {

class ViewModel {
 public:
  typedef std::function<void(Layer)> LayerInvalidatedDelegate;

  ViewModel();

  inline void RegisterInvalidatedDelegate(const LayerInvalidatedDelegate& d) {
    LayerInvalidated_ = d;
    PoseChanged();
  }

  inline PassiveGripper& PSG() { return psg_; }

  void SetMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
  void SetCurrentPose(const Pose& pose);
  void SetCurrentPose(const Eigen::Affine3d& trans);
  void SetCurrentPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& ang);

 private:
  LayerInvalidatedDelegate LayerInvalidated_;
  void InvokeLayerInvalidated(Layer layer);

  PassiveGripper psg_;
  void OnPsgInvalidated(PassiveGripper::InvalidatedReason reason);

  Pose current_pose_;

  std::vector<Pose> ik_sols_;
  size_t ik_sols_index_;
  Eigen::Vector3d eff_position_;
  Eigen::Vector3d eff_angles_;

  void ComputeIK();
  void PoseChanged();

 public:
  DECLARE_GETTER(PSG, psg_)
  DECLARE_GETTER(GetEffPosition, eff_position_)
  DECLARE_GETTER(GetEffAngles, eff_angles_)
  DECLARE_GETTER(GetCurrentPose, current_pose_)
};

}  // namespace ui
}  // namespace psg