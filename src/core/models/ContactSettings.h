#pragma once

#include "../serialization/Serialization.h"

namespace psg {
namespace core {
namespace models {

struct ContactSettings {
  double friction = 0.5;
  size_t cone_res = 4;
};


}  // namespace models
}  // namespace core
}  // namespace psg

DECL_SERIALIZE(psg::core::models::ContactSettings, obj) {
  constexpr int version = 1;
  SERIALIZE(version);
  SERIALIZE(obj.friction);
  SERIALIZE(obj.cone_res);
}

DECL_DESERIALIZE(psg::core::models::ContactSettings, obj) {
  int version;
  DESERIALIZE(version);
  if (version == 1) {
    DESERIALIZE(obj.friction);
    DESERIALIZE(obj.cone_res);
  }
}
