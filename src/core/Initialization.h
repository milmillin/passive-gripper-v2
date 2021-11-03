#pragma once

#include <Eigen/Core>
#include "PassiveGripper.h"
#include "models/ContactPoint.h"
#include "models/ContactPointMetric.h"
#include "models/GripperSettings.h"
#include "models/MeshDependentResource.h"

namespace psg {
namespace core {

using namespace models;

Eigen::MatrixXd InitializeFinger(const ContactPoint& contact_point,
                                 const MeshDependentResource& mdr,
                                 const Eigen::Vector3d& effector_pos,
                                 size_t num_finger_joints);

Trajectory InitializeTrajectory(const std::vector<Eigen::MatrixXd>& fingers,
                                const Pose& init_pose,
                                size_t n_keyframes);

std::vector<ContactPointMetric> InitializeContactPoints(
    const MeshDependentResource& mdr,
    const GripperSettings& settings,
    size_t num_candidates,
    size_t num_seeds);

void InitializeGripperBound(const PassiveGripper& psg,
                            Eigen::Vector3d& out_lb,
                            Eigen::Vector3d& out_ub);

void InitializeConservativeBound(const PassiveGripper& psg,
                                 Eigen::Vector3d& out_lb,
                                 Eigen::Vector3d& out_ub);

}  // namespace core
}  // namespace psg