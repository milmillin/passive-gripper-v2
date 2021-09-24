#pragma once

#include <Eigen/Core>
#include "models/ContactPoint.h"
#include "models/MeshDependentResource.h"

namespace psg {

using namespace models;

Eigen::MatrixXd InitializeFinger(const ContactPoint& contact_point,
                                 const MeshDependentResource& mdr,
                                 const Eigen::Vector3d& effector_pos,
                                 size_t num_finger_joints);

Trajectory InitializeTrajectory(const std::vector<Eigen::MatrixXd>& fingers,
                                const Pose& init_pose,
                                size_t n_keyframes);

}  // namespace psg