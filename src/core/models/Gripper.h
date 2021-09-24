#pragma once

#include "../Constants.h"
#include "GripperParams.h"
#include "Settings.h"

namespace psg {
namespace models {

struct Gripper {
  Settings settings;
  GripperParams params;
};

}
}