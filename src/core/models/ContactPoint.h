#pragma once

#include <Eigen/Core>


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

// DECL_SERIALIZE(psg::core::models::ContactPoint, obj) {
namespace psg {
namespace core {
namespace serialization {
inline void Serialize(const psg::core::models::ContactPoint& obj, std::ofstream& f);
inline void Deserialize(psg::core::models::ContactPoint& obj, std::ifstream& f);
}
}
}

#include "../serialization/Serialization.h"

void psg::core::serialization::Serialize(const psg::core::models::ContactPoint& obj, std::ofstream& f) {
  constexpr int version = 1;
  SERIALIZE(version);
  SERIALIZE(obj.position);
  SERIALIZE(obj.normal);
  SERIALIZE(obj.fid);
}

void psg::core::serialization::Deserialize(psg::core::models::ContactPoint& obj, std::ifstream& f) {
  int version;
  DESERIALIZE(version);
  if (version == 1) {
    DESERIALIZE(obj.position);
    DESERIALIZE(obj.normal);
    DESERIALIZE(obj.fid);
  }
}
