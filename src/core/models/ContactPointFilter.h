#pragma once

#include "../../Constants.h"

namespace psg {
namespace core {
namespace models {

struct ContactPointFilter {
  double hole = 0.005;
  double curvature_radius = 0.002;
  double angle = kDegToRad * 45;
};

}  // namespace models
}  // namespace core
}  // namespace psg