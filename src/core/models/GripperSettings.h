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

struct GripperSettings {
  ContactSettings contact;
  FingerSettings finger;
  TrajectorySettings trajectory;
  OptSettings opt;
  TopoOptSettings topo_opt;
  CostSettings cost;
};

}  // namespace models
}  // namespace core
}  // namespace psg

DECL_SERIALIZE(psg::core::models::GripperSettings, obj) {
  constexpr int version = 1;
  SERIALIZE(version);
  SERIALIZE(obj.contact);
  SERIALIZE(obj.finger);
  SERIALIZE(obj.trajectory);
  SERIALIZE(obj.opt);
  SERIALIZE(obj.topo_opt);
  SERIALIZE(obj.cost);
}

DECL_DESERIALIZE(psg::core::models::GripperSettings, obj) {
  int version;
  DESERIALIZE(version);
  if (version == 1) {
    DESERIALIZE(obj.contact);
    DESERIALIZE(obj.finger);
    DESERIALIZE(obj.trajectory);
    DESERIALIZE(obj.opt);
    DESERIALIZE(obj.topo_opt);
    DESERIALIZE(obj.cost);
  }
}