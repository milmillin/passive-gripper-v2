#include "ViewModel.h"

namespace psg {
namespace ui {

ViewModel::ViewModel() {
  using namespace std::placeholders;
  psg_.RegisterInvalidatedDelegate(
      std::bind(&ViewModel::OnPsgInvalidated, this, _1));
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

}  // namespace ui
}  // namespace psg
