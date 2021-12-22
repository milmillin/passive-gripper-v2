#include "CostFunctions.h"

#include <igl/copyleft/cgal/intersect_other.h>
#include "GeometryUtils.h"
#include "robots/Robots.h"

namespace psg {
namespace core {

// See CHOMP paper page 4
static double PotentialSDF(double s, double& out_dP_ds) {
  static constexpr double epsilon = 0.001;
  if (s > epsilon) {
    out_dP_ds = 0;
    return 0;
  }
  if (s < 0) {
    out_dP_ds = -1;
    return -s + (epsilon / 2.);
  }
  double tmp = s - epsilon;
  out_dP_ds = s / epsilon - 1;
  return tmp * tmp / (2. * epsilon);
}

static double GetDist(const Eigen::Vector3d& p,
                      const CostSettings& settings,
                      const MeshDependentResource& mdr,
                      Eigen::RowVector3d& out_ds_dp) {
  Eigen::RowVector3d c;
  double sign;
  double s = mdr.ComputeSignedDistance(p, c, sign);
  double sFloor = p.y() - settings.floor;
  if (s < sFloor) {
    if (s < 0)
      out_ds_dp = (c - p.transpose()).normalized();
    else
      out_ds_dp = (p.transpose() - c).normalized();
    return s;
  } else {
    out_ds_dp = Eigen::RowVector3d::UnitY();
    return sFloor;
  }
}

static double Norm(const Eigen::Vector3d& x1,
                   const Eigen::Vector3d& x2,
                   Eigen::RowVector3d& out_dNorm_dx1) {
  Eigen::RowVector3d x12 = (x1 - x2).transpose();
  double norm = x12.norm();
  out_dNorm_dx1 = x12 / norm;
  return norm;
}

double EvalAt(const Eigen::Vector3d& p,
              const CostSettings& settings,
              const MeshDependentResource& mdr,
              Eigen::RowVector3d& out_dc_dp) {
  Eigen::RowVector3d ds_dp;
  double dP_ds;
  double result = PotentialSDF(GetDist(p, settings, mdr, ds_dp), dP_ds);
  out_dc_dp = dP_ds * ds_dp;
  return result;
}

double ComputeCost(const GripperParams& params,
                   const GripperSettings& settings,
                   const MeshDependentResource& mdr,
                   std::vector<Eigen::MatrixXd>& out_dCost_dFinger) {
  const size_t nTrajectorySteps = settings.cost.n_trajectory_steps;
  const long long nFingerSteps = settings.cost.n_finger_steps;
  const double trajectoryStep = 1. / nTrajectorySteps;
  const double fingerStep = 1. / nFingerSteps;

  const size_t nKeyframes = params.trajectory.size();
  const size_t nFingers = params.fingers.size();
  const size_t nFingerJoints = settings.finger.n_finger_joints;
  const size_t nEvalsPerFingerPerFrame = (nFingerJoints - 1) * nFingerSteps + 1;

  static struct _Data {
    double eval;
    Eigen::Vector3d lerpedJoint;
    Eigen::RowVector3d dEval_dLerpedJoint;
    Eigen::Matrix3d dLerpedJoint_dJoint1;
    Eigen::Matrix3d dLerpedJoint_dJoint2;
    size_t iJoint;  // iJoint -- (iJoint + 1)
  };

  std::vector<std::vector<_Data>> lastData(
      nFingers, std::vector<_Data>(nEvalsPerFingerPerFrame));
  std::vector<std::vector<_Data>> curData(
      nFingers, std::vector<_Data>(nEvalsPerFingerPerFrame));

  // Finger in effector space
  std::vector<std::vector<Eigen::Vector3d>> effFingers(
      nFingers, std::vector<Eigen::Vector3d>(nEvalsPerFingerPerFrame));

  Eigen::Affine3d lastH;
  Eigen::Affine3d curH;

  std::vector<Eigen::MatrixXd> dCost_dFinger(
      nFingers, Eigen::MatrixXd::Zero(nFingerJoints, 3));

  // std::vector<Pose> dCost_dTheta(nKeyframes, Pose::Zero());

  Eigen::Affine3d fingerTransInv =
      robots::Forward(params.trajectory.front()).inverse();

  {
    curH = robots::Forward(params.trajectory.front());
    for (size_t i = 0; i < nFingers; i++) {
      const Eigen::MatrixXd& finger = params.fingers[i];
      Eigen::MatrixXd effFinger =
          (fingerTransInv * finger.transpose().colwise().homogeneous())
              .transpose();
#pragma omp parallel for
      for (long long jj = 0; jj < nEvalsPerFingerPerFrame; jj++) {
        _Data& data = lastData[i][jj];
        long long kk = (jj - 1) % nFingerSteps + 1;
        long long joint = (jj - 1) / nFingerSteps + 1;
        double fingerT = kk * fingerStep;

        // precompute finger
        Eigen::Vector3d lerpedJoint =
            curH *
            (effFingers[i][jj] = effFinger.row(joint - 1) * (1. - fingerT) +
                                 effFinger.row(joint) * fingerT);

        data.lerpedJoint = lerpedJoint;
        data.dLerpedJoint_dJoint1 =
            (1. - fingerT) * Eigen::Matrix3d::Identity();
        data.dLerpedJoint_dJoint2 = (fingerT)*Eigen::Matrix3d::Identity();
        data.eval = EvalAt(lerpedJoint,
                           settings.cost,
                           mdr,
                           data.dEval_dLerpedJoint);
        data.iJoint = joint - 1;
      }
    }
    lastH = curH;
  }

  double totalCost = 0.;
  for (size_t iKf = 1; iKf < nKeyframes; iKf++) {
    Pose t_lerpedKeyframe;
    for (long long j = 1; j <= nTrajectorySteps; j++) {
      double trajectoryT = j * trajectoryStep;
      t_lerpedKeyframe = params.trajectory[iKf - 1] * (1 - trajectoryT) +
                         params.trajectory[iKf] * trajectoryT;
      curH = robots::Forward(t_lerpedKeyframe);
      Eigen::Affine3d curTrans = curH * fingerTransInv;
      for (size_t i = 0; i < nFingers; i++) {
        const Eigen::MatrixXd& finger = params.fingers[i];
        Eigen::MatrixXd transformedFinger =
            (curTrans * finger.transpose().colwise().homogeneous()).transpose();
#pragma omp parallel for
        for (long long jj = 0; jj < nEvalsPerFingerPerFrame; jj++) {
          _Data& data = curData[i][jj];
          long long kk = (jj - 1) % nFingerSteps + 1;
          long long joint = (jj - 1) / nFingerSteps + 1;
          double fingerT = kk * fingerStep;

          Eigen::Vector3d lerpedJoint = curH * effFingers[i][jj];

          data.lerpedJoint = lerpedJoint;
          data.dLerpedJoint_dJoint1 = (1. - fingerT) * curTrans.linear();
          data.dLerpedJoint_dJoint2 = (fingerT)*curTrans.linear();
          data.eval = EvalAt(lerpedJoint,
                             settings.cost,
                             mdr,
                             data.dEval_dLerpedJoint);
          data.iJoint = joint - 1;
        }

#pragma omp parallel
        {
          double t_curCost = 0.;
          Eigen::MatrixXd t_dCost_dFinger =
              Eigen::MatrixXd::Zero(nFingerJoints, 3);

          auto t_ApplyGradient = [&t_curCost, &t_dCost_dFinger](
                                     const _Data& data, double factor) {
            t_dCost_dFinger.row(data.iJoint) +=
                data.dEval_dLerpedJoint * data.dLerpedJoint_dJoint1 * factor;
            t_dCost_dFinger.row(data.iJoint + 1) +=
                data.dEval_dLerpedJoint * data.dLerpedJoint_dJoint2 * factor;
          };
#pragma omp for
          for (long long jj = 1; jj < nEvalsPerFingerPerFrame; jj++) {
            Eigen::RowVector3d dFingerLen_dLerpedJoint1;
            // dFingerLen_dLerpedJoint2 = -dFingerLen_dLerpedJoint1
            double finger_len = Norm(curData[i][jj - 1].lerpedJoint,
                                     curData[i][jj].lerpedJoint,
                                     dFingerLen_dLerpedJoint1);
            double totalEval =
                curData[i][jj - 1].eval + 2 * curData[i][jj].eval +
                2 * lastData[i][jj - 1].eval + lastData[i][jj].eval;
            t_curCost += totalEval * finger_len * trajectoryStep;

            // dEval/dFinger * fingerLen * trajectoryStep
            t_ApplyGradient(curData[i][jj - 1], finger_len * trajectoryStep);
            t_ApplyGradient(curData[i][jj], 2 * finger_len * trajectoryStep);
            t_ApplyGradient(lastData[i][jj - 1],
                            2 * finger_len * trajectoryStep);
            t_ApplyGradient(lastData[i][jj], finger_len * trajectoryStep);

            // totalEval * dFingerLen/dFinger * trajectoryStep
            t_dCost_dFinger.row(curData[i][jj - 1].iJoint) +=
                totalEval * dFingerLen_dLerpedJoint1 *
                curData[i][jj - 1].dLerpedJoint_dJoint1 * trajectoryStep;
            t_dCost_dFinger.row(curData[i][jj - 1].iJoint + 1) +=
                totalEval * dFingerLen_dLerpedJoint1 *
                curData[i][jj - 1].dLerpedJoint_dJoint2 * trajectoryStep;
            t_dCost_dFinger.row(curData[i][jj].iJoint) +=
                totalEval * -dFingerLen_dLerpedJoint1 *
                curData[i][jj].dLerpedJoint_dJoint1 * trajectoryStep;
            t_dCost_dFinger.row(curData[i][jj].iJoint + 1) +=
                totalEval * -dFingerLen_dLerpedJoint1 *
                curData[i][jj].dLerpedJoint_dJoint2 * trajectoryStep;
          }

#pragma omp critical
          {
            totalCost += t_curCost;
            dCost_dFinger[i] += t_dCost_dFinger;
          }
        }
      }
      lastH = curH;
      std::swap(curData, lastData);
    }
  }
  totalCost /= 6.;
  for (size_t i = 0; i < nFingers; i++) {
    dCost_dFinger[i] /= 6.;
  }
  out_dCost_dFinger = dCost_dFinger;
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
    Eigen::RowVector3d t_dP_ds;  // unused
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
          t_min_dist = std::min(
              t_min_dist,
              GetDist(lerpedFinger.transpose(), settings.cost, mdr, t_dP_ds));
        }
      }
    }
#pragma omp critical
    min_dist = std::min(min_dist, t_min_dist);
  }
  return min_dist;
}

bool Intersects(const GripperParams& params,
                const GripperSettings& settings,
                const MeshDependentResource& mdr) {
  const size_t nTrajectorySteps = settings.cost.n_trajectory_steps;
  const double trajectoryStep = 1. / nTrajectorySteps;

  const size_t nKeyframes = params.trajectory.size();
  const size_t nFingers = params.fingers.size();
  const size_t nFingerJoints = settings.finger.n_finger_joints;
  const size_t nFrames = (nKeyframes - 1) * nTrajectorySteps + 1;

  if (params.trajectory.size() <= 1llu) return false;

  std::vector<Eigen::MatrixXd> sv_V(
      nFingers, Eigen::MatrixXd(nFingerJoints * nFrames, 3));
  Eigen::MatrixXi sv_F((nFingerJoints - 1) * (nFrames - 1) * 2, 3);

  Eigen::Affine3d finger_trans_inv =
      robots::Forward(params.trajectory.front()).inverse();

  Eigen::MatrixXi sv_F_template((nFingerJoints - 1) * 2, 3);
  for (size_t i = 0; i < nFingerJoints - 1; i++) {
    sv_F_template.row(i * 2) = Eigen::RowVector3i(i, i + nFingerJoints, i + 1);
    sv_F_template.row(i * 2 + 1) =
        Eigen::RowVector3i(i + 1, i + nFingerJoints, i + nFingerJoints + 1);
  }
  for (size_t j = 0; j < nFrames - 1; j++) {
    sv_F.block(j * (nFingerJoints - 1) * 2, 0, (nFingerJoints - 1) * 2, 3) =
        sv_F_template.array() + (j * nFingerJoints);
  }
  for (size_t i = 0; i < nFingers; i++) {
    for (size_t j = 0; j < nFrames; j++) {
      size_t a = j / nTrajectorySteps;
      size_t b = j % nTrajectorySteps;
      if (a == nKeyframes - 1) {
        a--;
        b = nTrajectorySteps;
      }
      double t = (double)b / nTrajectorySteps;

      Pose l_pose =
          params.trajectory[a] * (1. - t) + params.trajectory[a + 1] * (t);

      Eigen::Affine3d cur_trans = robots::Forward(l_pose) * finger_trans_inv;
      sv_V[i].block(j * nFingerJoints, 0, nFingerJoints, 3) =
          (cur_trans * params.fingers[i].transpose().colwise().homogeneous())
              .transpose();
    }
    Eigen::MatrixXi IF;
    bool intersect = igl::copyleft::cgal::intersect_other(
        mdr.V, mdr.F, sv_V[i], sv_F, true, IF);
    if (intersect) return true;
  }
  return false;
}

}  // namespace core
}  // namespace psg