#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "../../Constants.h"

namespace psg {
namespace core {
namespace robots {

Eigen::Affine3d Forward(const Pose& jointConfig);

bool Inverse(Eigen::Affine3d trans, std::vector<Pose>& out_jointConfigs);

void ForwardIntermediate(const Pose& jointConfig,
                         std::vector<Eigen::Affine3d>& out_trans);

}  // namespace robots
}  // namespace core
}  // namespace psg
