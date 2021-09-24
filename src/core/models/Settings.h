#pragma once

#include "../Constants.h"

namespace psg {
namespace models {

struct Settings {
  // ContactPoints
  size_t n_finger_joints = 4;
  double friction = 0.5;
  size_t cone_res = 4;

  // Trajectory
  size_t n_keyframes = 4;

  // Floor
  double floor = 0.01;

  // Compute Cost
  size_t n_trajectory_steps = 32;
  size_t n_finger_steps = 32;

  // TopologyOpt
  Eigen::Vector3d lower_bound = Eigen::Vector3d(-0.2, -0.05, 0.5);
  Eigen::Vector3d upper_bound = Eigen::Vector3d(0.05, 0.2, 0.8);
  double neg_vol_res = 0.005;
  double topo_res = 0.005;
  double attachment_size = 0.04;
  int attachment_samples = 30;

  double contact_point_size = 0.01;
  double base_thickness = 0.01;

  // Optimization
  double opt_max_runtime = 0;  // seconds
  double opt_finger_wiggle = 0.01;
  Pose opt_trajectory_wiggle = {4. * kDegToRad,
                                4. * kDegToRad,
                                4. * kDegToRad,
                                8. * kDegToRad,
                                8. * kDegToRad,
                                8. * kDegToRad};
  double opt_tolerance = 0;  // run forever
};

}  // namespace models
}  // namespace psg