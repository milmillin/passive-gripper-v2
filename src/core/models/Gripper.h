#pragma once

#include <vector>

#include "../Constants.h"
#include "GripperParams.h"
#include "ContactPoint.h"
#include "FingerSettings.h"
#include "TrajectorySettings.h"
#include "OptSettings.h"
#include "TopoOptSettings.h"
#include "CostSettings.h"

namespace psg {
namespace models {

struct Gripper {
  GripperParams params;
  std::vector<ContactPoint> contact_points;
  FingerSettings finger_settings;
  TrajectorySettings trajectory_settings;
  OptSettings opt_settings;
  TopoOptSettings topo_opt_settings;
  CostSettings cost_settings;
};

}
}