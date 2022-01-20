#pragma once

#include "../serialization/Serialization.h"

namespace psg {
namespace core {
namespace models {

struct ContactSettings : psg::core::serialization::Serializable {
  double friction = 0.5;
  size_t cone_res = 4;
  double floor = 0.01;

  DECL_SERIALIZE() {
    constexpr int version = 2;
    SERIALIZE(version);
    SERIALIZE(friction);
    SERIALIZE(cone_res);
    SERIALIZE(floor);
  }

  DECL_DESERIALIZE() {
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(friction);
      DESERIALIZE(cone_res);
    } else if (version == 2) {
      DESERIALIZE(friction);
      DESERIALIZE(cone_res);
      DESERIALIZE(floor);
    }
  }
};

}  // namespace models
}  // namespace core
}  // namespace psg
