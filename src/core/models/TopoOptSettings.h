#pragma once

#include "../Constants.h"

namespace psg {
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
}  // namespace psg