#pragma once

#include "../Constants.h"

namespace psg {
namespace models {

struct FingerSettings {
  size_t n_finger_joints = 4;
  double friction = 0.5;
  size_t cone_res = 4;
};

}  // namespace models
}  // namespace psg
