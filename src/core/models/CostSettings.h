#pragma once

#include "../serialization/Serialization.h"

namespace psg {
namespace core {
namespace models {

struct CostSettings : psg::core::serialization::Serializable {
  double floor = 0.0075;
  size_t n_trajectory_steps = 32;
  size_t n_finger_steps = 32;
  double ang_velocity = kDegToRad * 60.;
  CostFunctionEnum cost_function = CostFunctionEnum::kSP;
  double regularization = 1e-6;

  DECL_SERIALIZE() {
    constexpr int version = 4;
    SERIALIZE(version);
    SERIALIZE(floor);
    SERIALIZE(n_trajectory_steps);
    SERIALIZE(n_finger_steps);
    SERIALIZE(ang_velocity);
    SERIALIZE(cost_function);
    SERIALIZE(regularization);
  }

  DECL_DESERIALIZE() {
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(floor);
      DESERIALIZE(n_trajectory_steps);
      DESERIALIZE(n_finger_steps);
    } else if (version == 2) {
      DESERIALIZE(floor);
      DESERIALIZE(n_trajectory_steps);
      DESERIALIZE(n_finger_steps);
      DESERIALIZE(ang_velocity);
    } else if (version == 3) {
      DESERIALIZE(floor);
      DESERIALIZE(n_trajectory_steps);
      DESERIALIZE(n_finger_steps);
      DESERIALIZE(ang_velocity);
      DESERIALIZE(cost_function);
    } else if (version == 4) {
      DESERIALIZE(floor);
      DESERIALIZE(n_trajectory_steps);
      DESERIALIZE(n_finger_steps);
      DESERIALIZE(ang_velocity);
      DESERIALIZE(cost_function);
      DESERIALIZE(regularization);
    }
  }
};

}  // namespace models
}  // namespace core
}  // namespace psg
