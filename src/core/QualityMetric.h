#pragma once

#include <Eigen/Core>
#include <vector>

#include "Constants.h"
#include "models/ContactPoint.h"

namespace psg {

using namespace models;

// Source:
// https://github.com/BerkeleyAutomation/dex-net/blob/master/src/dexnet/grasping/quality.py

// Checks force closure by solving a quadratic program
// (whether or not zero is in the convex hull)
bool CheckForceClosureQP(const std::vector<ContactPoint>& contactCones,
                         const Eigen::Vector3d& centerOfMass);

// Evalutes partial closure: whether or not the forces and torques
// can resist a specific wrench. Estimates resistance by solving a quadratic
// program (whether or not the target wrench is in the convex hull).
bool CheckPartialClosureQP(const std::vector<ContactPoint>& contactCones,
                           const Eigen::Vector3d& centerOfMass,
                           const Eigen::Vector3d& extForce,
                           const Eigen::Vector3d& extTorque);

// Ferrari & Canny's L1 metric. Also known as the epsilon metric.
double ComputeMinWrenchQP(const std::vector<ContactPoint>& contactCones,
                          const Eigen::Vector3d& centerOfMass);

//
double ComputePartialMinWrenchQP(const std::vector<ContactPoint>& contactCones,
                                 const Eigen::Vector3d& centerOfMass,
                                 const Eigen::Vector3d& extForce,
                                 const Eigen::Vector3d& extTorque);

}  // namespace psg
