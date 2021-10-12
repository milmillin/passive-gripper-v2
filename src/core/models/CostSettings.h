#pragma once

#include "../serialization/Serialization.h"

namespace psg {
namespace core {
namespace models {

struct CostSettings : psg::core::serialization::Serializable {
  double floor = 0.01;
  size_t n_trajectory_steps = 32;
  size_t n_finger_steps = 32;

  SERIALIZE_MEMBER() {
    constexpr int version = 1;
    SERIALIZE(version);
    SERIALIZE(floor);
    SERIALIZE(n_trajectory_steps);
    SERIALIZE(n_finger_steps);
  }

  DESERIALIZE_MEMBER() {
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(floor);
      DESERIALIZE(n_trajectory_steps);
      DESERIALIZE(n_finger_steps);
    }
  }
};

}  // namespace models
}  // namespace core
}  // namespace psg
