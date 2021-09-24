#pragma once

#include "../Constants.h"

namespace psg {
namespace models {

struct GripperParams {
  std::vector<Eigen::MatrixXd> fingers;
  Trajectory trajectory;
};

}
}