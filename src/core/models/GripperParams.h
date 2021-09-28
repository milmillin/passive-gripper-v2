#pragma once

#include "../../Constants.h"
#include "ContactPoint.h"

namespace psg {
namespace core {
namespace models {

struct GripperParams {
  std::vector<Eigen::MatrixXd> fingers;
  Trajectory trajectory;
  std::vector<ContactPoint> contact_points;
};

}  // namespace models
}  // namespace core
}  // namespace psg