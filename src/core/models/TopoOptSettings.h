#pragma once

#include "../serialization/Serialization.h"

namespace psg {
namespace core {
namespace models {

struct TopoOptSettings : psg::core::serialization::Serializable {
  Eigen::Vector3d lower_bound = Eigen::Vector3d(-0.2, -0.05, 0.5);
  Eigen::Vector3d upper_bound = Eigen::Vector3d(0.05, 0.2, 0.8);
  double neg_vol_res = 0.005;
  double topo_res = 0.005;
  double attachment_size = 0.04;
  int attachment_samples = 30;

  double contact_point_size = 0.01;
  double base_thickness = 0.01;

  SERIALIZE_MEMBER() {
    constexpr int version = 1;
    SERIALIZE(version);
    SERIALIZE(lower_bound);
    SERIALIZE(upper_bound);
    SERIALIZE(neg_vol_res);
    SERIALIZE(topo_res);
    SERIALIZE(attachment_size);
    SERIALIZE(attachment_samples);
    SERIALIZE(contact_point_size);
    SERIALIZE(base_thickness);
  }

  DESERIALIZE_MEMBER() {
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(lower_bound);
      DESERIALIZE(upper_bound);
      DESERIALIZE(neg_vol_res);
      DESERIALIZE(topo_res);
      DESERIALIZE(attachment_size);
      DESERIALIZE(attachment_samples);
      DESERIALIZE(contact_point_size);
      DESERIALIZE(base_thickness);
    }
  }
};

}  // namespace models
}  // namespace core
}  // namespace psg
