#pragma once

#include "../serialization/Serialization.h"

namespace psg {
namespace core {
namespace models {

struct FingerSettings : psg::core::serialization::Serializable {
  size_t n_finger_joints = 4;  // Number of joints in a finger

  DECL_SERIALIZE() {
    constexpr int version = 1;
    SERIALIZE(version);
    SERIALIZE(n_finger_joints);
  }

  DECL_DESERIALIZE() {
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(n_finger_joints);
    }
  }
};

}  // namespace models
}  // namespace core
}  // namespace psg
