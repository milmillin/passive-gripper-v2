#pragma once

#include "../serialization/Serialization.h"

namespace psg {
namespace core {
namespace models {

struct TopoOptSettings {
  Eigen::Vector3d lower_bound = Eigen::Vector3d(-0.2, -0.05, 0.5);
  Eigen::Vector3d upper_bound = Eigen::Vector3d(0.05, 0.2, 0.8);
  double neg_vol_res = 0.005;
  double topo_res = 0.005;
  double attachment_size = 0.04;
  int attachment_samples = 30;

  double contact_point_size = 0.01;
  double base_thickness = 0.01;
};

}  // namespace models
}  // namespace core
}  // namespace psg

DECL_SERIALIZE(psg::core::models::TopoOptSettings, obj) {
  constexpr int version = 1;
  SERIALIZE(version);
  SERIALIZE(obj.lower_bound);
  SERIALIZE(obj.upper_bound);
  SERIALIZE(obj.neg_vol_res);
  SERIALIZE(obj.topo_res);
  SERIALIZE(obj.attachment_size);
  SERIALIZE(obj.attachment_samples);
  SERIALIZE(obj.contact_point_size);
  SERIALIZE(obj.base_thickness);
}

DECL_DESERIALIZE(psg::core::models::TopoOptSettings, obj) {
  int version;
  DESERIALIZE(version);
  if (version == 1) {
    DESERIALIZE(obj.lower_bound);
    DESERIALIZE(obj.upper_bound);
    DESERIALIZE(obj.neg_vol_res);
    DESERIALIZE(obj.topo_res);
    DESERIALIZE(obj.attachment_size);
    DESERIALIZE(obj.attachment_samples);
    DESERIALIZE(obj.contact_point_size);
    DESERIALIZE(obj.base_thickness);
  }
}