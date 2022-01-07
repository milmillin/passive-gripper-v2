#pragma once

#include "../../Constants.h"

namespace psg {
namespace core {
namespace models {

struct ContactPointFilter {
  double hole = 0.006;
  double curvature_radius = 0;
  double angle = kPi;  // 180 deg
};

}  // namespace models
}  // namespace core
}  // namespace psg