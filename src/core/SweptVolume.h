#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "PassiveGripper.h"

namespace psg {
namespace core {

// Compute Swept Volume of the box minus swept volume of mesh and floor
// V, F        : Mesh
// out_V, out_F : Neg Vol
// out_SV_V, out_SV_F : Swept Vol
void NegativeSweptVolume(const PassiveGripper& psg,
                         Eigen::MatrixXd& out_V,
                         Eigen::MatrixXi& out_F,
                         Eigen::MatrixXd& out_SV_V,
                         Eigen::MatrixXi& out_SV_F,
                         const int num_seeds = 100);

void SweptVolume(const PassiveGripper& psg,
                 Eigen::MatrixXd& out_V,
                 Eigen::MatrixXi& out_F,
                 const int num_seeds = 100);

}  // namespace core
}  // namespace psg