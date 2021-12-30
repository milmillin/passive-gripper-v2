#pragma once

#include <vector>
#include <Eigen/Geometry>

#include "../serialization/Serialization.h"
#include "ContactPoint.h"

namespace psg {
namespace core {
namespace models {

struct ContactPointMetric : psg::core::serialization::Serializable {
  std::vector<ContactPoint> contact_points;
  double min_wrench;
  double partial_min_wrench;
  Eigen::Affine3d trans;

  DECL_SERIALIZE() {
    constexpr int version = 1;
    SERIALIZE(version);
    SERIALIZE(contact_points);
    SERIALIZE(min_wrench);
    SERIALIZE(partial_min_wrench);
  }

  DECL_DESERIALIZE() {
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(contact_points);
      DESERIALIZE(min_wrench);
      DESERIALIZE(partial_min_wrench);
    }
  }

  bool operator<(const ContactPointMetric& r) const {
    if (min_wrench == r.min_wrench)
      return partial_min_wrench > r.partial_min_wrench;
    return min_wrench > r.min_wrench;
  }
};

}  // namespace models
}  // namespace core
}  // namespace psg