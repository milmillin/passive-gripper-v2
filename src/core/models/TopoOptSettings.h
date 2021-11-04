#pragma once

#include "../serialization/Serialization.h"

namespace psg {
namespace core {
namespace models {

struct TopoOptSettings : psg::core::serialization::Serializable {
  Eigen::Vector3d lower_bound = Eigen::Vector3d(-0.2, -0.05, 0.5);
  Eigen::Vector3d upper_bound = Eigen::Vector3d(0.05, 0.2, 0.8);
  double neg_vol_res = 0.002;
  double topo_res = 0.002;
  double attachment_size = 0.038; // diameter
  double vol_frac = 0.02; // percentage wrt pi bound

  double contact_point_size = 0.01;
  double base_thickness = 0.01;

  DECL_SERIALIZE() {
    constexpr int version = 2;
    SERIALIZE(version);
    SERIALIZE(lower_bound);
    SERIALIZE(upper_bound);
    SERIALIZE(neg_vol_res);
    SERIALIZE(topo_res);
    SERIALIZE(attachment_size);
    SERIALIZE(vol_frac);
    SERIALIZE(contact_point_size);
    SERIALIZE(base_thickness);
  }

  DECL_DESERIALIZE() {
    int __unused;
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(lower_bound);
      DESERIALIZE(upper_bound);
      DESERIALIZE(neg_vol_res);
      DESERIALIZE(topo_res);
      DESERIALIZE(attachment_size);
      DESERIALIZE(__unused); // int
      DESERIALIZE(contact_point_size);
      DESERIALIZE(base_thickness);
    } else if (version == 2) {
      DESERIALIZE(lower_bound);
      DESERIALIZE(upper_bound);
      DESERIALIZE(neg_vol_res);
      DESERIALIZE(topo_res);
      DESERIALIZE(attachment_size);
      DESERIALIZE(vol_frac);
      DESERIALIZE(contact_point_size);
      DESERIALIZE(base_thickness);
    }
  }
};

}  // namespace models
}  // namespace core
}  // namespace psg
