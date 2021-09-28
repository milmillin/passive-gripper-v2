#pragma once

#include "Constants.h"
#include "models/Gripper.h"
#include "models/MeshDependentResource.h"

namespace psg {

using namespace models;

double EvalAt(const Eigen::Vector3d& p,
              const CostSettings& settings,
              const MeshDependentResource& mdr);

double ComputeCost(const Gripper& gripper, const MeshDependentResource& mdr);

}  // namespace psg