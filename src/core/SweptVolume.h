#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "PassiveGripper.h"

namespace psg {
namespace core {

// Compute Swept Volume of the box minus swept volume of mesh and floor
// V, F        : Mesh
// Transformations  : List of transformations
// boxLB, boxUB: Lower and upper bound of the box
// floor       : y-coordinate of the floor
void NegativeSweptVolume(const PassiveGripper& psg,
                         Eigen::MatrixXd& out_V,
                         Eigen::MatrixXi& out_F,
                         const int num_seeds = 100);

void SweptVolume(const PassiveGripper& psg,
                 Eigen::MatrixXd& out_V,
                 Eigen::MatrixXi& out_F,
                 const int num_seeds = 100);

}  // namespace core
}  // namespace psg