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
                        const std::string& filename,
                        std::vector<Eigen::Vector3i>& out_attachment_voxels,
                        std::vector<Eigen::Vector3i>& out_contact_voxels,
                        std::vector<Eigen::Vector3i>& out_forbidden_voxels);

}
}  // namespace psg