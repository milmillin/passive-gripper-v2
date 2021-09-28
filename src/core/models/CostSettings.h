#pragma once

namespace psg {
namespace core {
namespace models {

struct CostSettings {
  double floor = 0.01;
  size_t n_trajectory_steps = 32;
  size_t n_finger_steps = 32;
};

}  // namespace models
}  // namespace core
}  // namespace psg
