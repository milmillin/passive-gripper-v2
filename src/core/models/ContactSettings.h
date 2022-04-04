#pragma once

#include "../serialization/Serialization.h"

namespace psg {
namespace core {
namespace models {

// Settings for contact points
struct ContactSettings : psg::core::serialization::Serializable {
  double friction = 0.5;  // Friction coefficient (\mu)
  size_t cone_res = 4;    // Edges in a pyramid to approximate friction cone
  double floor = 0.01;    // Minimum height that is reachable
  double max_angle =
      kDegToRad * 80;  // Maximum angle between the approach direction and the
                       // contact normal to be considered as reachable

  DECL_SERIALIZE() {
    constexpr int version = 3;
    SERIALIZE(version);
    SERIALIZE(friction);
    SERIALIZE(cone_res);
    SERIALIZE(floor);
    SERIALIZE(max_angle);
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
    } else if (version == 3) {
      DESERIALIZE(friction);
      DESERIALIZE(cone_res);
      DESERIALIZE(floor);
      DESERIALIZE(max_angle);
    }
  }
};

}  // namespace models
}  // namespace core
}  // namespace psg
