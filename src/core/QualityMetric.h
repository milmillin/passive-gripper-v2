#pragma once

#include <Eigen/Core>
#include <vector>

#include "../Constants.h"
#include "models/ContactPoint.h"
#include "models/ContactPointMetric.h"
#include "models/GripperSettings.h"
#include "models/MeshDependentResource.h"

namespace psg {
namespace core {

using namespace models;

// Source:
// https://github.com/BerkeleyAutomation/dex-net/blob/master/src/dexnet/grasping/quality.py

Eigen::MatrixXd CreateGraspMatrix(
    const std::vector<ContactPoint>& contact_cones,
    const Eigen::Vector3d& center_of_mass);

// Checks force closure by solving a quadratic program
// (whether or not zero is in the convex hull)
bool CheckForceClosureQP(const Eigen::MatrixXd& grasp_matrix);

// Evalutes partial closure: whether or not the forces and torques
// can resist a specific wrench. Estimates resistance by solving a quadratic
// program (whether or not the target wrench is in the convex hull).
bool CheckPartialClosureQP(const Eigen::MatrixXd& grasp_matrix,
                           const Eigen::Vector3d& extForce,
                           const Eigen::Vector3d& extTorque);

// Ferrari & Canny's L1 metric. Also known as the epsilon metric.
double ComputeMinWrenchQP(const Eigen::MatrixXd& grasp_matrix);

//
double ComputePartialMinWrenchQP(const Eigen::MatrixXd& grasp_matrix,
                                 const Eigen::Vector3d& extForce,
                                 const Eigen::Vector3d& extTorque);

bool ComputeRobustQualityMetric(const std::vector<ContactPoint>& contact_points,
                                const MeshDependentResource& mdr,
                                const GripperSettings& settings,
                                const Eigen::Vector3d& ext_force,
                                const Eigen::Vector3d& ext_torque,
                                ContactPointMetric& out_metric);

}  // namespace core
}  // namespace psg
