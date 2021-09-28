#pragma once

namespace psg {
namespace ui {

enum class Layers : int {
  kMesh = 0,
  kCenterOfMass,
  kContactPoints,
  kFingers,
  kTrajectory,
  kAxis,
  kSweptSurface,
  kRobot,
  kMax
};

}
}  // namespace psg