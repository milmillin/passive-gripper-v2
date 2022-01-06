#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "../Constants.h"
#include "models/GripperParams.h"
#include "models/GripperSettings.h"
#include "models/MeshDependentResource.h"

namespace psg {
namespace core {

using namespace models;

double GetDist(const Eigen::Vector3d& p,
               const CostSettings& settings,
               const MeshDependentResource& mdr,
               Eigen::RowVector3d& out_ds_dp);

double EvalAt(const Eigen::Vector3d& p,
              const CostSettings& settings,
              const MeshDependentResource& mdr,
              Eigen::RowVector3d& out_dc_dp);

double ComputeCost(const GripperParams& params,
                   const GripperSettings& settings,
                   const MeshDependentResource& mdr,
                   GripperParams& out_dCost_dParam);

double MinDistanceAtPose(const std::vector<Eigen::MatrixXd>& fingers,
                         const Eigen::Affine3d& finger_trans_inv,
                         const MeshDependentResource& mdr,
                         const GripperSettings& settings,
                         const Pose& current_pose);

double MinDistance(const GripperParams& params,
                   const GripperSettings& settings,
                   const MeshDependentResource& mdr);

bool Intersects(const GripperParams& params,
                const GripperSettings& settings,
                const MeshDependentResource& mdr);

}  // namespace core
}  // namespace psg