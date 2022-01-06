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

double GetDist(const Eigen::Vector3d& p,
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

double ComputeDuration(const Pose& p1,
                       const Pose& p2,
                       double ang_velocity,
                       size_t& out_idx,
                       bool& out_flip) {
  Pose dp = (p2 - p1) / ang_velocity;
  double duration = -1;
  out_idx = -1;
  for (size_t i = 0; i < kNumDOFs; i++) {
    double d = dp(i);
    bool flip = d < 0;
    if (flip) d = -d;
    if (d > duration) {
      duration = d;
      out_idx = i;
      out_flip = flip;
    }
  }
  return duration;
}

double ComputeCost(const GripperParams& params,
                   const GripperSettings& settings,
                   const MeshDependentResource& mdr,
                   GripperParams& out_dCost_dParam) {
  const size_t nTrajectorySteps = settings.cost.n_trajectory_steps;
  const long long nFingerSteps = settings.cost.n_finger_steps;
  const double angVelocity = settings.cost.ang_velocity;
  const double trajectoryStep = 1. / nTrajectorySteps;
  const double fingerStep = 1. / nFingerSteps;

  const size_t nKeyframes = params.trajectory.size();
  const size_t nFingers = params.fingers.size();
  const size_t nFingerJoints = settings.finger.n_finger_joints;
  const size_t nEvalsPerFingerPerFrame = (nFingerJoints - 1) * nFingerSteps + 1;

  struct _Data {
    double eval;
    Eigen::Vector3d lerpedJoint;
    Eigen::RowVector3d dEval_dLerpedJoint;
    Eigen::Matrix3d dLerpedJoint_dJoint1;
    Eigen::Matrix3d dLerpedJoint_dJoint2;
    size_t iJoint;  // iJoint -- (iJoint + 1)
    Jacobian dpos_dtheta;
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

  std::vector<Pose> dCost_dTheta(nKeyframes, Pose::Zero());

  Eigen::Affine3d fingerTrans = robots::Forward(params.trajectory.front());
  Eigen::Affine3d fingerTransInv = fingerTrans.inverse();

  {
    curH = robots::Forward(params.trajectory.front());
    JacobianFunc J = robots::ComputeJacobian(params.trajectory.front());
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
        data.dLerpedJoint_dJoint2 = fingerT * Eigen::Matrix3d::Identity();
        data.eval =
            EvalAt(lerpedJoint, settings.cost, mdr, data.dEval_dLerpedJoint);
        data.iJoint = joint - 1;
        data.dpos_dtheta = J(effFingers[i][jj]);
      }
    }
    lastH = curH;
  }

  double totalCost = 0.;
  for (size_t iKf = 1; iKf < nKeyframes; iKf++) {
    size_t duration_idx;
    bool duration_flip;
    double duration = ComputeDuration(params.trajectory[iKf - 1],
                                      params.trajectory[iKf],
                                      angVelocity,
                                      duration_idx,
                                      duration_flip);
    // std::cout << std::setprecision(12) << duration << " " << duration_idx <<
    // " "
    //<< duration_flip << std::endl;
    Pose t_lerpedKeyframe;
    for (long long j = 1; j <= nTrajectorySteps; j++) {
      double trajectoryT = j * trajectoryStep;
      double lastTrajectoryT = (j - 1) * trajectoryStep;
      t_lerpedKeyframe = params.trajectory[iKf - 1] * (1 - trajectoryT) +
                         params.trajectory[iKf] * trajectoryT;
      curH = robots::Forward(t_lerpedKeyframe);
      JacobianFunc J = robots::ComputeJacobian(t_lerpedKeyframe);
      Eigen::Affine3d curTrans = curH * fingerTransInv;
      for (size_t i = 0; i < nFingers; i++) {
#pragma omp parallel for
        for (long long jj = 0; jj < nEvalsPerFingerPerFrame; jj++) {
          _Data& data = curData[i][jj];
          long long kk = (jj - 1) % nFingerSteps + 1;
          long long joint = (jj - 1) / nFingerSteps + 1;
          double fingerT = kk * fingerStep;

          Eigen::Vector3d lerpedJoint = curH * effFingers[i][jj];

          data.lerpedJoint = lerpedJoint;
          data.dLerpedJoint_dJoint1 = (1. - fingerT) * curTrans.linear();
          data.dLerpedJoint_dJoint2 = fingerT * curTrans.linear();
          data.eval =
              EvalAt(lerpedJoint, settings.cost, mdr, data.dEval_dLerpedJoint);
          data.iJoint = joint - 1;
          data.dpos_dtheta = J(effFingers[i][jj]);
        }

#pragma omp parallel
        {
          double t_curCost = 0.;
          Eigen::MatrixXd t_dCost_dFinger =
              Eigen::MatrixXd::Zero(nFingerJoints, 3);
          Pose t_dCost_dTheta_0 = Pose::Zero();
          Pose t_dCost_dTheta_1 = Pose::Zero();

          auto t_ApplyGradient =
              [&t_dCost_dTheta_0,
               &t_dCost_dTheta_1,
               &t_dCost_dFinger,
               &curH,
               &lastH,
               iKf,
               lastTrajectoryT,
               trajectoryT](const _Data& data, double factor, bool last) {
                Eigen::RowVector3d dEval_dJoint1 =
                    data.dEval_dLerpedJoint * data.dLerpedJoint_dJoint1;
                Eigen::RowVector3d dEval_dJoint2 =
                    data.dEval_dLerpedJoint * data.dLerpedJoint_dJoint2;

                Eigen::Matrix<double, 1, 6> dEval_dTheta =
                    data.dEval_dLerpedJoint * (last ? lastH : curH).linear() *
                    data.dpos_dtheta;

                // dEval/dFinger * factor
                t_dCost_dFinger.row(data.iJoint) += dEval_dJoint1 * factor;
                t_dCost_dFinger.row(data.iJoint + 1) += dEval_dJoint2 * factor;

                // dEval/dTheta * factor
                double t = last ? lastTrajectoryT : trajectoryT;
                t_dCost_dTheta_0 += dEval_dTheta.array() * ((1. - t) * factor);
                t_dCost_dTheta_1 += dEval_dTheta.array() * (t * factor);
              };
#pragma omp for
          for (long long jj = 1; jj < nEvalsPerFingerPerFrame; jj++) {
            Eigen::RowVector3d dFingerLen_dLerpedJoint1;
            // dFingerLen_dLerpedJoint2 = -dFingerLen_dLerpedJoint1
            double finger_len = Norm(curData[i][jj - 1].lerpedJoint,
                                     curData[i][jj].lerpedJoint,
                                     dFingerLen_dLerpedJoint1);
            double total_eval =
                curData[i][jj - 1].eval + 2 * curData[i][jj].eval +
                2 * lastData[i][jj - 1].eval + lastData[i][jj].eval;
            double non_eval_factor = finger_len * trajectoryStep * duration;

            // The cost
            t_curCost += total_eval * non_eval_factor;

            // Apply eval part of gradient
            t_ApplyGradient(curData[i][jj - 1], non_eval_factor, false);
            t_ApplyGradient(curData[i][jj], 2 * non_eval_factor, false);
            t_ApplyGradient(lastData[i][jj - 1], 2 * non_eval_factor, true);
            t_ApplyGradient(lastData[i][jj], non_eval_factor, true);

            // total_eval * dFingerLen/dFinger * trajectoryStep * duration
            double non_finger_len_factor =
                total_eval * trajectoryStep * duration;
            t_dCost_dFinger.row(curData[i][jj - 1].iJoint) +=
                dFingerLen_dLerpedJoint1 *
                curData[i][jj - 1].dLerpedJoint_dJoint1 * non_finger_len_factor;
            t_dCost_dFinger.row(curData[i][jj - 1].iJoint + 1) +=
                dFingerLen_dLerpedJoint1 *
                curData[i][jj - 1].dLerpedJoint_dJoint2 * non_finger_len_factor;
            t_dCost_dFinger.row(curData[i][jj].iJoint) +=
                -dFingerLen_dLerpedJoint1 *
                curData[i][jj].dLerpedJoint_dJoint1 * non_finger_len_factor;
            t_dCost_dFinger.row(curData[i][jj].iJoint + 1) +=
                -dFingerLen_dLerpedJoint1 *
                curData[i][jj].dLerpedJoint_dJoint2 * non_finger_len_factor;

            // total_eval * finger_len * trajectoryStep * dDuration/dTheta
            double ddTheta =
                total_eval * finger_len * trajectoryStep / angVelocity;
            if (duration_flip) ddTheta = -ddTheta;
            t_dCost_dTheta_0(duration_idx) -= ddTheta;
            t_dCost_dTheta_1(duration_idx) += ddTheta;
          }

#pragma omp critical
          {
            totalCost += t_curCost;
            dCost_dFinger[i] += t_dCost_dFinger;
            dCost_dTheta[iKf - 1] += t_dCost_dTheta_0;
            dCost_dTheta[iKf] += t_dCost_dTheta_1;
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
  for (size_t i = 0; i < nKeyframes; i++) {
    dCost_dTheta[i] /= 6;
  }
  out_dCost_dParam.fingers = dCost_dFinger;
  out_dCost_dParam.trajectory = dCost_dTheta;
  return totalCost;
}

double MinDistanceAtPose(const std::vector<Eigen::MatrixXd>& fingers,
                         const Eigen::Affine3d& finger_trans_inv,
                         const MeshDependentResource& mdr,
                         const GripperSettings& settings,
                         const Pose& current_pose) {
  const size_t n_finger_steps = settings.cost.n_finger_steps;
  const size_t n_finger_joints = settings.finger.n_finger_joints;
  const size_t n_evals_per_finger = (n_finger_joints - 1) * n_finger_steps + 1;
  const double finger_step = 1. / n_finger_steps;

  double t_min_dist = 0;
  Eigen::RowVector3d t_dP_ds;  // unused
  Eigen::Affine3d trans = robots::Forward(current_pose) * finger_trans_inv;
  for (const auto& finger : fingers) {
    Eigen::MatrixXd t_finger =
        (trans * finger.transpose().colwise().homogeneous()).transpose();
    for (long long jj = 1; jj < n_evals_per_finger; jj++) {
      long long kk = (jj - 1) % n_finger_steps + 1;
      long long joint = (jj - 1) / n_finger_steps + 1;
      double fingerT = kk * finger_step;
      Eigen::RowVector3d lerpedFinger =
          t_finger.row(joint - 1) * (1. - fingerT) +
          t_finger.row(joint) * fingerT;
      t_min_dist = std::min(
          t_min_dist,
          GetDist(lerpedFinger.transpose(), settings.cost, mdr, t_dP_ds));
    }
  }
  return t_min_dist;
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

      t_min_dist =
          std::min(t_min_dist,
                   MinDistanceAtPose(
                       params.fingers, fingerTransInv, mdr, settings, l_pose));
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