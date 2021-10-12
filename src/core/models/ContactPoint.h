#pragma once

#include <Eigen/Core>

#include "../serialization/Serialization.h"


namespace psg {
namespace core {
namespace models {

struct ContactPoint : psg::core::serialization::Serializable {
  Eigen::Vector3d position;
  Eigen::Vector3d normal;  // pointing out of mesh
  int fid;

  SERIALIZE_MEMBER() {
    constexpr int version = 1;
    SERIALIZE(version);
    SERIALIZE(position);
    SERIALIZE(normal);
    SERIALIZE(fid);
  }

  DESERIALIZE_MEMBER() {
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(position);
      DESERIALIZE(normal);
      DESERIALIZE(fid);
    }
  }
};

}  // namespace models
}  // namespace core
}  // namespace psg

