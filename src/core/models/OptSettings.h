#pragma once

#include <nlopt.h>

#include "../../Constants.h"

namespace psg {
namespace core {
namespace models {

struct OptSettings {
  double max_runtime = 0;  // seconds
  double finger_wiggle = 0.01;
  Pose trajectory_wiggle = {4. * kDegToRad,
                                4. * kDegToRad,
                                4. * kDegToRad,
                                8. * kDegToRad,
                                8. * kDegToRad,
                                8. * kDegToRad};
  double tolerance = 0;  // run forever
  nlopt_algorithm algorithm = NLOPT_GN_CRS2_LM;
  size_t population = 20000;
};

}  // namespace models
}  // namespace core
}  // namespace psg