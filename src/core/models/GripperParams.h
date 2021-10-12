#pragma once

#include "ContactPoint.h"

#include "../../Constants.h"
#include "../serialization/Serialization.h"

namespace psg {
namespace core {
namespace models {

struct GripperParams : psg::core::serialization::Serializable {
  std::vector<Eigen::MatrixXd> fingers;
  Trajectory trajectory;
  std::vector<ContactPoint> contact_points;

  SERIALIZE_MEMBER() {
    constexpr int version = 1;
    SERIALIZE(version);
    SERIALIZE(fingers);
    SERIALIZE(trajectory);
    SERIALIZE(contact_points);
  }

  DESERIALIZE_MEMBER() {
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(fingers);
      DESERIALIZE(trajectory);
      DESERIALIZE(contact_points);
    }
  }
};

}  // namespace models
}  // namespace core
}  // namespace psg
