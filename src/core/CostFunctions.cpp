#include "CostFunctions.h"

#include "GeometryUtils.h"
#include "robots/Robots.h"

namespace psg {

// See CHOMP paper page 4
static double PotentialSDF(double s) {
  static constexpr double epsilon = 0.001;
  if (s > epsilon) return 0;
  if (s < 0) {
    return -s + (epsilon / 2.);
  }
  double tmp = s - epsilon;
  return tmp * tmp / (2. * epsilon);
}

double EvalAt(const Eigen::Vector3d& p,
              const CostSettings& settings,
              const MeshDependentResource& mdr) {
  Eigen::RowVector3d c;
  double sign;
  double s = mdr.ComputeSignedDistance(p, c, sign);
  double sFloor = p.y() - settings.floor;
  return PotentialSDF(std::min(s, sFloor));
}

double ComputeCost(const Gripper& gripper, const MeshDependentResource& mdr) {
  static const size_t nTrajectorySteps = gripper.cost_settings.n_trajectory_steps;
  static const size_t nFingerSteps = gripper.cost_settings.n_finger_steps;
  static const double trajectoryStep = 1. / nTrajectorySteps;
  static const double fingerStep = 1. / nFingerSteps;

  const size_t nKeyframes = gripper.params.trajectory.size();
  const size_t nFingers = gripper.params.fingers.size();
  const size_t nFingerJoints = gripper.finger_settings.n_finger_joints;
  const size_t nEvalsPerFingerPerFrame = (nFingerJoints - 1) * nFingerSteps + 1;

  std::vector<std::vector<double>> lastEval(
      nFingers, std::vector<double>(nEvalsPerFingerPerFrame, 0));
  std::vector<std::vector<Eigen::Vector3d>> lastFinger(
      nFingers, std::vector<Eigen::Vector3d>(nEvalsPerFingerPerFrame));
  std::vector<std::vector<double>> curEval(
      nFingers, std::vector<double>(nEvalsPerFingerPerFrame, 0));
  std::vector<std::vector<Eigen::Vector3d>> curFinger(
      nFingers, std::vector<Eigen::Vector3d>(nEvalsPerFingerPerFrame));

  Eigen::Affine3d fingerTransInv =
      robots::Forward(gripper.params.trajectory.front()).inverse();

  {
    Eigen::Affine3d curTrans =
        robots::Forward(gripper.params.trajectory.front()) * fingerTransInv;
    for (size_t i = 0; i < nFingers; i++) {
      const Eigen::MatrixXd& finger = gripper.params.fingers[i];
      Eigen::MatrixXd transformedFinger =
          (curTrans * finger.transpose().colwise().homogeneous()).transpose();
      lastFinger[i][0] = transformedFinger.row(0).transpose();
      lastEval[i][0] =
          EvalAt(transformedFinger.row(0).transpose(), gripper.cost_settings, mdr);
      for (size_t joint = 1; joint < nFingerJoints; joint++) {
        for (size_t kk = 1; kk <= nFingerSteps; kk++) {
          double fingerT = kk * fingerStep;
          Eigen::RowVector3d lerpedFinger =
              transformedFinger.row(joint - 1) * (1. - fingerT) +
              transformedFinger.row(joint) * fingerT;
          lastFinger[i][(joint - 1) * nFingerSteps + kk] =
              lerpedFinger.transpose();
          lastEval[i][(joint - 1) * nFingerSteps + kk] =
              EvalAt(lerpedFinger.transpose(), gripper.cost_settings, mdr);
        }
      }
    }
  }

  double totalCost = 0.;
  for (size_t iKf = 1; iKf < nKeyframes; iKf++) {
    Pose t_lerpedKeyframe;
    for (long long j = 1; j <= nTrajectorySteps; j++) {
      double trajectoryT = j * trajectoryStep;
      for (size_t k = 0; k < kNumDOFs; k++) {
        t_lerpedKeyframe[k] =
            gripper.params.trajectory[iKf - 1][k] * (1 - trajectoryT) +
            gripper.params.trajectory[iKf][k] * trajectoryT;
      }
      Eigen::Affine3d curTrans =
          robots::Forward(t_lerpedKeyframe) * fingerTransInv;
      for (size_t i = 0; i < nFingers; i++) {
        const Eigen::MatrixXd& finger = gripper.params.fingers[i];
        Eigen::MatrixXd transformedFinger =
            (curTrans * finger.transpose().colwise().homogeneous()).transpose();
        curFinger[i][0] = transformedFinger.row(0).transpose();
        curEval[i][0] =
            EvalAt(transformedFinger.row(0).transpose(), gripper.cost_settings, mdr);
#pragma omp parallel for
        for (long long jj = 1; jj < nEvalsPerFingerPerFrame; jj++) {
          long long kk = (jj - 1) % nFingerSteps + 1;
          long long joint = (jj - 1) / nFingerSteps + 1;
          double fingerT = kk * fingerStep;
          Eigen::RowVector3d lerpedFinger =
              transformedFinger.row(joint - 1) * (1. - fingerT) +
              transformedFinger.row(joint) * fingerT;
          curFinger[i][jj] = lerpedFinger.transpose();
          curEval[i][jj] =
              EvalAt(lerpedFinger.transpose(), gripper.cost_settings, mdr);
        }
        double curCost = 0.;
#pragma omp parallel for reduction(+ : curCost)
        for (long long jj = 1; jj < nEvalsPerFingerPerFrame; jj++) {
          curCost +=
              (curEval[i][jj - 1] + curEval[i][jj] + lastEval[i][jj - 1]) *
              DoubleTriangleArea(curFinger[i][jj - 1],
                                 curFinger[i][jj],
                                 lastFinger[i][jj - 1]);
          curCost +=
              (lastEval[i][jj] + curEval[i][jj] + lastEval[i][jj - 1]) *
              DoubleTriangleArea(
                  lastFinger[i][jj], curFinger[i][jj], lastFinger[i][jj - 1]);
        }
        totalCost += curCost;
      }
      std::swap(curEval, lastEval);
      std::swap(curFinger, lastFinger);
    }
  }
  totalCost /= 6.;
  return totalCost;
}
}  // namespace psg