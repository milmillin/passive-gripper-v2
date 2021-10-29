#include "CostFunctions.h"

#include "GeometryUtils.h"
#include "robots/Robots.h"

namespace psg {
namespace core {

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

static double GetDist(const Eigen::Vector3d& p,
                      const CostSettings& settings,
                      const MeshDependentResource& mdr) {
  Eigen::RowVector3d c;
  double sign;
  double s = mdr.ComputeSignedDistance(p, c, sign);
  double sFloor = p.y() - settings.floor;
  return std::min(s, sFloor);
}

double EvalAt(const Eigen::Vector3d& p,
              const CostSettings& settings,
              const MeshDependentResource& mdr) {
  return PotentialSDF(GetDist(p, settings, mdr));
}

double ComputeCost(const GripperParams& params,
                   const GripperSettings& settings,
                   const MeshDependentResource& mdr) {
  const size_t nTrajectorySteps = settings.cost.n_trajectory_steps;
  const size_t nFingerSteps = settings.cost.n_finger_steps;
  const double trajectoryStep = 1. / nTrajectorySteps;
  const double fingerStep = 1. / nFingerSteps;

  const size_t nKeyframes = params.trajectory.size();
  const size_t nFingers = params.fingers.size();
  const size_t nFingerJoints = settings.finger.n_finger_joints;
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
      robots::Forward(params.trajectory.front()).inverse();

  {
    Eigen::Affine3d curTrans =
        robots::Forward(params.trajectory.front()) * fingerTransInv;
    for (size_t i = 0; i < nFingers; i++) {
      const Eigen::MatrixXd& finger = params.fingers[i];
      Eigen::MatrixXd transformedFinger =
          (curTrans * finger.transpose().colwise().homogeneous()).transpose();
      lastFinger[i][0] = transformedFinger.row(0).transpose();
      lastEval[i][0] =
          EvalAt(transformedFinger.row(0).transpose(), settings.cost, mdr);
      for (size_t joint = 1; joint < nFingerJoints; joint++) {
        for (size_t kk = 1; kk <= nFingerSteps; kk++) {
          double fingerT = kk * fingerStep;
          Eigen::RowVector3d lerpedFinger =
              transformedFinger.row(joint - 1) * (1. - fingerT) +
              transformedFinger.row(joint) * fingerT;
          lastFinger[i][(joint - 1) * nFingerSteps + kk] =
              lerpedFinger.transpose();
          lastEval[i][(joint - 1) * nFingerSteps + kk] =
              EvalAt(lerpedFinger.transpose(), settings.cost, mdr);
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
            params.trajectory[iKf - 1][k] * (1 - trajectoryT) +
            params.trajectory[iKf][k] * trajectoryT;
      }
      Eigen::Affine3d curTrans =
          robots::Forward(t_lerpedKeyframe) * fingerTransInv;
      for (size_t i = 0; i < nFingers; i++) {
        const Eigen::MatrixXd& finger = params.fingers[i];
        Eigen::MatrixXd transformedFinger =
            (curTrans * finger.transpose().colwise().homogeneous()).transpose();
        curFinger[i][0] = transformedFinger.row(0).transpose();
        curEval[i][0] =
            EvalAt(transformedFinger.row(0).transpose(), settings.cost, mdr);
#pragma omp parallel for
        for (long long jj = 1; jj < nEvalsPerFingerPerFrame; jj++) {
          long long kk = (jj - 1) % nFingerSteps + 1;
          long long joint = (jj - 1) / nFingerSteps + 1;
          double fingerT = kk * fingerStep;
          Eigen::RowVector3d lerpedFinger =
              transformedFinger.row(joint - 1) * (1. - fingerT) +
              transformedFinger.row(joint) * fingerT;
          curFinger[i][jj] = lerpedFinger.transpose();
          curEval[i][jj] = EvalAt(lerpedFinger.transpose(), settings.cost, mdr);
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

double MinDistance(const GripperParams& params,
                   const GripperSettings& settings,
                   const MeshDependentResource& mdr) {
  const size_t nTrajectorySteps = settings.cost.n_trajectory_steps;
  const size_t nFingerSteps = settings.cost.n_finger_steps;
  const double trajectoryStep = 1. / nTrajectorySteps;
  const double fingerStep = 1. / nFingerSteps;

  const size_t nKeyframes = params.trajectory.size();
  const size_t nFingers = params.fingers.size();
  const size_t nFingerJoints = settings.finger.n_finger_joints;
  const size_t nEvalsPerFingerPerFrame = (nFingerJoints - 1) * nFingerSteps + 1;
  const size_t nFrames = (nKeyframes - 1) * nTrajectorySteps + 1;

  if (nKeyframes <= 1) return 0;

  double min_dist = 0;

  Eigen::Affine3d fingerTransInv =
      robots::Forward(params.trajectory.front()).inverse();

#pragma omp parallel
  {
    double t_min_dist = 0;
#pragma omp for
    for (long long ii = 0; ii < nFrames; ii++) {
      size_t a = ii / nTrajectorySteps;
      size_t b = ii % nTrajectorySteps;
      if (a == nKeyframes - 1) {
        a--;
        b = nTrajectorySteps;
      }
      double t = (double)b / nTrajectorySteps;
      Pose l_pose =
          params.trajectory[a] * (1. - t) + params.trajectory[a + 1] * (t);

      Eigen::Affine3d cur_trans = robots::Forward(l_pose) * fingerTransInv;
      for (size_t i = 0; i < nFingers; i++) {
        const Eigen::MatrixXd& finger = params.fingers[i];
        Eigen::MatrixXd t_finger =
            (cur_trans * finger.transpose().colwise().homogeneous())
                .transpose();
        for (long long jj = 1; jj < nEvalsPerFingerPerFrame; jj++) {
          long long kk = (jj - 1) % nFingerSteps + 1;
          long long joint = (jj - 1) / nFingerSteps + 1;
          double fingerT = kk * fingerStep;
          Eigen::RowVector3d lerpedFinger =
              t_finger.row(joint - 1) * (1. - fingerT) +
              t_finger.row(joint) * fingerT;
          t_min_dist =
              std::min(t_min_dist,
                       GetDist(lerpedFinger.transpose(), settings.cost, mdr));
        }
      }
    }
#pragma omp critical
    min_dist = std::min(min_dist, t_min_dist);
  }
  return min_dist;
}

}  // namespace core
}  // namespace psg