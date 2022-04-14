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

struct _DebugHit {
  size_t traj_cnt_missing = 0;
  size_t traj_cnt_extra = 0;
  size_t gripper_cnt_missing = 0;
  size_t gripper_cnt_extra = 0;
  mutable std::mutex lock;

  _DebugHit(){}

  _DebugHit(const psg::core::models::_DebugHit& other) {
    *this = other;
  }

  _DebugHit &operator=(const _DebugHit& other) {
    traj_cnt_missing = other.traj_cnt_missing;
    traj_cnt_extra = other.traj_cnt_extra;
    gripper_cnt_missing = other.gripper_cnt_missing;
    gripper_cnt_extra = other.gripper_cnt_extra;
  }

};

struct GripperSettings : psg::core::serialization::Serializable {
  ContactSettings contact;
  FingerSettings finger;
  TrajectorySettings trajectory;
  OptSettings opt;
  TopoOptSettings topo_opt;
  CostSettings cost;
  mutable _DebugHit debug_hit;

  DECL_SERIALIZE() {
    constexpr int version = 1;
    SERIALIZE(version);
    SERIALIZE(contact);
    SERIALIZE(finger);
    SERIALIZE(trajectory);
    SERIALIZE(opt);
    SERIALIZE(topo_opt);
    SERIALIZE(cost);
  }

  DECL_DESERIALIZE() {
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
