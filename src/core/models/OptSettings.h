#pragma once

#include "../Constants.h"

namespace psg {
namespace models {

struct OptSettings {
  double opt_max_runtime = 0;  // seconds
  double opt_finger_wiggle = 0.01;
  Pose opt_trajectory_wiggle = {4. * kDegToRad,
                                4. * kDegToRad,
                                4. * kDegToRad,
                                8. * kDegToRad,
                                8. * kDegToRad,
                                8. * kDegToRad};
  double opt_tolerance = 0;  // run forever
};

}  // namespace models
}  // namespace psg