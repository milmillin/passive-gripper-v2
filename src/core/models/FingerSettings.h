#pragma once

#include "../serialization/Serialization.h"

namespace psg {
namespace core {
namespace models {

struct FingerSettings {
  size_t n_finger_joints = 4;
};

}  // namespace models
}  // namespace core
}  // namespace psg

DECL_SERIALIZE(psg::core::models::FingerSettings, obj) {
  constexpr int version = 1;
  SERIALIZE(version);
  SERIALIZE(obj.n_finger_joints);
}

DECL_DESERIALIZE(psg::core::models::FingerSettings, obj) {
  int version;
  DESERIALIZE(version);
  if (version == 1) {
    DESERIALIZE(obj.n_finger_joints);
  }
}
