#pragma once

#include "../serialization/Serialization.h"

namespace psg {
namespace core {
namespace models {

struct TrajectorySettings {
  size_t n_keyframes = 4;
};

}  // namespace models
}  // namespace core
}  // namespace psg

DECL_SERIALIZE(psg::core::models::TrajectorySettings, obj) {
  constexpr int version = 1;
  SERIALIZE(version);
  SERIALIZE(obj.n_keyframes);
}

DECL_DESERIALIZE(psg::core::models::TrajectorySettings, obj) {
  int version;
  DESERIALIZE(version);
  if (version == 1) {
    DESERIALIZE(obj.n_keyframes);
  }
}