#pragma once

#include "../core/PassiveGripper.h"
#include "Layer.h"

using namespace psg::core;

namespace psg {
namespace ui {

class ViewModel {
 public:
  typedef std::function<void(Layer)> LayerInvalidatedDelegate;

  ViewModel();

  inline void RegisterInvalidatedDelegate(const LayerInvalidatedDelegate& d) {
    LayerInvalidated_ = d;
  }

  inline PassiveGripper& PSG() { return psg_; }
  inline const PassiveGripper& PSG() const { return psg_; }

  inline const Pose& GetCurrentPose() const { return current_pose_; }

 private:
  LayerInvalidatedDelegate LayerInvalidated_;
  void InvokeLayerInvalidated(Layer layer);

  PassiveGripper psg_;
  void OnPsgInvalidated(PassiveGripper::InvalidatedReason reason);
  
  Pose current_pose_ = kInitPose;
};

}  // namespace ui
}  // namespace psg