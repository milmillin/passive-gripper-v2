#pragma once

#include <vector>

#include "ContactSettings.h"
#include "CostSettings.h"
#include "FingerSettings.h"
#include "OptSettings.h"
#include "TopoOptSettings.h"
#include "TrajectorySettings.h"

namespace psg {
namespace core {
namespace models {

struct GripperSettings {
  ContactSettings contact;
  FingerSettings finger;
  TrajectorySettings trajectory;
  OptSettings opt;
  TopoOptSettings topo_opt;
  CostSettings cost;
};

}  // namespace models
}  // namespace core
}  // namespace psg