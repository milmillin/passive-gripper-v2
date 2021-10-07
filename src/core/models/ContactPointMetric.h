#pragma once

#include <vector>

#include "ContactPoint.h"

namespace psg {
namespace core {
namespace models {

struct ContactPointMetric {
  std::vector<ContactPoint> contact_points;
  double min_wrench;
  double partial_min_wrench;
};

}
}
}