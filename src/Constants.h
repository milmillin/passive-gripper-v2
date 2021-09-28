#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <vector>

namespace psg {

constexpr double kPi = EIGEN_PI;
constexpr double kTwoPi = EIGEN_PI * 2.;
constexpr double kHalfPi = EIGEN_PI / 2.;
constexpr float kDegToRad = EIGEN_PI / 180.;
constexpr float kRadToDeg = 180. / EIGEN_PI;

constexpr size_t kNumDOFs = 6;
const double kRobotA[] = {0, -0.425, -0.39225, 0, 0, 0};
const double kRobotD[] = {0.089159, 0, 0, 0.10915, 0.09465, 0.0823};
const double kRobotAlpha[] = {kHalfPi, 0, 0, kHalfPi, -kHalfPi, 0};

const double kArmRadius = 0.0465;
const Eigen::Affine3d kLocalTrans[6] = {
    Eigen::Translation3d(-kArmRadius, -0.0892, -kArmRadius) *
        Eigen::Scaling(2 * kArmRadius, kArmRadius + 0.0892, 2 * kArmRadius),
    Eigen::Translation3d(-kArmRadius, -kArmRadius, kArmRadius) *
        Eigen::Scaling(2 * kArmRadius + 0.425, 2 * kArmRadius, 2 * kArmRadius),
    Eigen::Translation3d(-kArmRadius, -kArmRadius, 0.02 - kArmRadius) *
        Eigen::Scaling(2 * kArmRadius + 0.39225,
                       2 * kArmRadius,
                       2 * kArmRadius),
    Eigen::Translation3d(-kArmRadius, -kArmRadius, -kArmRadius) *
        Eigen::Scaling(2 * kArmRadius, 2 * kArmRadius, 0.09465),
    Eigen::Translation3d(-kArmRadius, -kArmRadius, -kArmRadius) *
        Eigen::Scaling(2 * kArmRadius,
                       2 * kArmRadius,
                       kArmRadius + 0.0825 - 0.01),
    Eigen::Translation3d(-kArmRadius, -kArmRadius, -0.01) *
        Eigen::Scaling(2 * kArmRadius, 2 * kArmRadius, 0.01)};

typedef std::array<double, kNumDOFs> Pose;
typedef std::vector<Pose> Trajectory;

const Pose kInitPose = {-kHalfPi, -2., -2., 4., -kHalfPi, 0.};

// Quality Metric
// small float to make quadratic program positive semidefinite
static constexpr double kWrenchReg = 1e-10;
// zero threshold
static constexpr double kWrenchNormThresh = 1e-5;

namespace colors {
const Eigen::RowVector3d kPurple = Eigen::RowVector3d(219, 76, 178) / 255;
const Eigen::RowVector3d kOrange = Eigen::RowVector3d(239, 126, 50) / 255;
const Eigen::RowVector3d kRed = Eigen::RowVector3d(192, 35, 35) / 255;
const Eigen::RowVector3d kBrown = Eigen::RowVector3d(130, 4, 1) / 255;
const Eigen::RowVector3d kDarkBlue = Eigen::RowVector3d(20, 36, 89) / 255;
const Eigen::RowVector3d kBlue = Eigen::RowVector3d(0, 0, 255) / 255;
const Eigen::RowVector3d kGold = Eigen::RowVector3d(252, 181, 9) / 255;
const Eigen::RowVector3d kWhite = Eigen::RowVector3d(255, 255, 255) / 255;
}  // namespace colors
}  // namespace psg
