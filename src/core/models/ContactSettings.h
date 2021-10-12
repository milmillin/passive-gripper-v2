#pragma once

#include "../serialization/Serialization.h"

namespace psg {
namespace core {
namespace models {

struct ContactSettings : psg::core::serialization::Serializable {
  double friction = 0.5;
  size_t cone_res = 4;

  SERIALIZE_MEMBER() {
    constexpr int version = 1;
    SERIALIZE(version);
    SERIALIZE(friction);
    SERIALIZE(cone_res);
  }

  DESERIALIZE_MEMBER() {
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(friction);
      DESERIALIZE(cone_res);
    }
  }
};

}  // namespace models
}  // namespace core
}  // namespace psg
