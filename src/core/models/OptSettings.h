#pragma once

#include <nlopt.h>

#include "../../Constants.h"
#include "../serialization/Serialization.h"

namespace psg {
namespace core {
namespace models {

struct OptSettings {
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
};

}  // namespace models
}  // namespace core
}  // namespace psg

DECL_SERIALIZE(psg::core::models::OptSettings, obj) {
  constexpr int version = 1;
  SERIALIZE(version);
  SERIALIZE(obj.max_runtime);
  SERIALIZE(obj.finger_wiggle);
  SERIALIZE(obj.trajectory_wiggle);
  SERIALIZE(obj.tolerance);
  SERIALIZE(obj.algorithm);
  SERIALIZE(obj.population);
}

DECL_DESERIALIZE(psg::core::models::OptSettings, obj) {
  int version = 1;
  DESERIALIZE(version);
  if (version == 1) {
    DESERIALIZE(obj.max_runtime);
    DESERIALIZE(obj.finger_wiggle);
    DESERIALIZE(obj.trajectory_wiggle);
    DESERIALIZE(obj.tolerance);
    DESERIALIZE(obj.algorithm);
    DESERIALIZE(obj.population);
  }
}