#pragma once

#include <nlopt.h>

#include "../../Constants.h"
#include "../serialization/Serialization.h"

namespace psg {
namespace core {
namespace models {

struct OptSettings : psg::core::serialization::Serializable {
  double max_runtime = 0;  // seconds
  double finger_wiggle = 0.01;
  Pose trajectory_wiggle = (Pose() << 4. * kDegToRad,
                            4. * kDegToRad,
                            4. * kDegToRad,
                            8. * kDegToRad,
                            8. * kDegToRad,
                            8. * kDegToRad)
                               .finished();
  double tolerance = 0;  // run forever
  nlopt_algorithm algorithm = NLOPT_GN_CRS2_LM;
  size_t population = 20000;

  SERIALIZE_MEMBER() {
    constexpr int version = 1;
    SERIALIZE(version);
    SERIALIZE(max_runtime);
    SERIALIZE(finger_wiggle);
    SERIALIZE(trajectory_wiggle);
    SERIALIZE(tolerance);
    SERIALIZE(algorithm);
    SERIALIZE(population);
  }

  DESERIALIZE_MEMBER() {
    int version = 1;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(max_runtime);
      DESERIALIZE(finger_wiggle);
      DESERIALIZE(trajectory_wiggle);
      DESERIALIZE(tolerance);
      DESERIALIZE(algorithm);
      DESERIALIZE(population);
    }
  }
};

}  // namespace models
}  // namespace core
}  // namespace psg
