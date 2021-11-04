#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

#include "PassiveGripper.h"

namespace psg {
namespace core {

void GenerateTopyConfig(const PassiveGripper& psg,
                        const Eigen::MatrixXd& neg_V,
                        const Eigen::MatrixXi& neg_F,
                        const std::string& filename);

void LoadResultBin(const PassiveGripper& psg,
                   const std::string& filename,
                   Eigen::MatrixXd& out_V,
                   Eigen::MatrixXi& out_F);

// Refine gripper mesh
// V, F    : gripper mesh
// sv_     : swept volume
// out_    : refined gripper
void RefineGripper(const PassiveGripper& psg,
                   const Eigen::MatrixXd& V,
                   const Eigen::MatrixXi& F,
                   const Eigen::MatrixXd& neg_V,
                   const Eigen::MatrixXi& neg_F,
                   Eigen::MatrixXd& out_V,
                   Eigen::MatrixXi& out_F);

}
}  // namespace psg