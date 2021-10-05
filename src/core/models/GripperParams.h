#pragma once

#include "../../Constants.h"
#include "../serialization/Serialization.h"
#include "ContactPoint.h"

namespace psg {
namespace core {
namespace models {

struct GripperParams {
  std::vector<Eigen::MatrixXd> fingers;
  Trajectory trajectory;
  std::vector<ContactPoint> contact_points;
};

}  // namespace models
}  // namespace core
}  // namespace psg

DECL_SERIALIZE(psg::core::models::GripperParams, obj) {
  constexpr int version = 1;
  SERIALIZE(version);
  SERIALIZE(obj.fingers);
  SERIALIZE(obj.trajectory);
  SERIALIZE(obj.contact_points);
}

DECL_DESERIALIZE(psg::core::models::GripperParams, obj) {
  int version;
  DESERIALIZE(version);
  if (version == 1) {
    DESERIALIZE(obj.fingers);
    DESERIALIZE(obj.trajectory);
    DESERIALIZE(obj.contact_points);
  }
}