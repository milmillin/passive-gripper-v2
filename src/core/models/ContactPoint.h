#pragma once

#include <Eigen/Core>

#include "../serialization/Serialization.h"

namespace psg {
namespace core {
namespace models {

struct ContactPoint {
  Eigen::Vector3d position;
  Eigen::Vector3d normal;  // pointing out of mesh
  int fid;
};

}  // namespace models
}  // namespace core
}  // namespace psg

DECL_SERIALIZE(psg::core::models::ContactPoint, obj) {
  constexpr int version = 1;
  SERIALIZE(version);
  SERIALIZE(obj.position);
  SERIALIZE(obj.normal);
  SERIALIZE(obj.fid);
}

DECL_DESERIALIZE(psg::core::models::ContactPoint, obj) {
  int version;
  DESERIALIZE(version);
  if (version == 1) {
    DESERIALIZE(obj.position);
    DESERIALIZE(obj.normal);
    DESERIALIZE(obj.fid);
  }
}
