#pragma once

#include <vector>

#include "../serialization/Serialization.h"
#include "ContactSettings.h"
#include "CostSettings.h"
#include "FingerSettings.h"
#include "OptSettings.h"
#include "TopoOptSettings.h"
#include "TrajectorySettings.h"

namespace psg {
namespace core {
namespace models {

struct GripperSettings : psg::core::serialization::Serializable {
  ContactSettings contact;
  FingerSettings finger;
  TrajectorySettings trajectory;
  OptSettings opt;
  TopoOptSettings topo_opt;
  CostSettings cost;

  SERIALIZE_MEMBER() {
    constexpr int version = 1;
    SERIALIZE(version);
    SERIALIZE(contact);
    SERIALIZE(finger);
    SERIALIZE(trajectory);
    SERIALIZE(opt);
    SERIALIZE(topo_opt);
    SERIALIZE(cost);
  }

  DESERIALIZE_MEMBER() {
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(contact);
      DESERIALIZE(finger);
      DESERIALIZE(trajectory);
      DESERIALIZE(opt);
      DESERIALIZE(topo_opt);
      DESERIALIZE(cost);
    }
  }
};

}  // namespace models
}  // namespace core
}  // namespace psg
