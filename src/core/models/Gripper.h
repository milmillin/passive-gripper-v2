#pragma once

#include <vector>

#include "../Constants.h"
#include "GripperParams.h"
#include "Settings.h"
#include "ContactPoint.h"

namespace psg {
namespace models {

struct Gripper {
  Settings settings;
  std::vector<ContactPoint> contact_points;
  GripperParams params;
};

}
}