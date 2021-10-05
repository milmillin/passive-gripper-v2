#pragma once

#include "../serialization/Serialization.h"

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

DECL_SERIALIZE(psg::core::models::CostSettings, obj) {
  constexpr int version = 1;
  SERIALIZE(version);
  SERIALIZE(obj.floor);
  SERIALIZE(obj.n_trajectory_steps);
  SERIALIZE(obj.n_finger_steps);
}

DECL_DESERIALIZE(psg::core::models::CostSettings, obj) {
  int version;
  DESERIALIZE(version);
  if (version == 1) {
    DESERIALIZE(obj.floor);
    DESERIALIZE(obj.n_trajectory_steps);
    DESERIALIZE(obj.n_finger_steps);
  }
}
