#include "CostFunctions.h"

#include <igl/copyleft/cgal/intersect_other.h>
#include "../easy_profiler_headers.h"
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
  // EASY_FUNCTION();

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
                   const GripperParams& init_params,
                   const GripperSettings& settings,
                   const MeshDependentResource& mdr,
                   GripperParams& out_dCost_dParam,
                   Debugger* const debugger) {
  EASY_FUNCTION();

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

double ComputeCost1(const GripperParams& params,
                    const GripperParams& init_params,
                    const GripperSettings& settings,
                    const MeshDependentResource& mdr,
                    GripperParams& out_dCost_dParam,
                    Debugger* const debugger) {
  constexpr double precision = 0.001;  // 1mm

  Eigen::Affine3d finger_trans_inv =
      robots::Forward(params.trajectory.front()).inverse();
  std::vector<Eigen::MatrixXd> fingers =
      TransformFingers(params.fingers, finger_trans_inv);

  // discretize fingers
  struct _Data {
    Eigen::Vector3d finger;
    size_t finger_idx;
    size_t joint_idx;
    double t;
    double factor;
    // finger[finger_idx].row(joint_idx - 1) * (1 - t)
    // + finger[finger_idx].row(joint_idx) * t
  };

  // 1/2 -- 1 -- 1 -- 1/2 * length / subs

  std::vector<_Data> d_fingers;
  for (size_t i = 0; i < fingers.size(); i++) {
    for (size_t j = 1; j < fingers[i].rows(); j++) {
      double norm = (fingers[i].row(j) - fingers[i].row(j - 1)).norm();
      size_t subs = std::max<size_t>(std::ceil(norm / precision), 1);
      size_t iters = subs;
      double factor = norm / (subs * 2);
      if (j == 1) {
        d_fingers.push_back(_Data{fingers[i].row(j - 1), i, j - 1, 0, 0});
      }
      for (size_t k = 1; k <= subs; k++) {
        double t = (double)k / subs;
        d_fingers.back().factor += factor;
        d_fingers.push_back(
            _Data{fingers[i].row(j - 1) * (1. - t) + fingers[i].row(j) * t,
                  i,
                  j - 1,
                  t,
                  factor});
      }
    }
  }
  Eigen::MatrixXd D_fingers(d_fingers.size(), 3);
#pragma omp parallel for
  for (long long i = 0; i < d_fingers.size(); i++) {
    D_fingers.row(i) = d_fingers[i].finger;
  }

  // discretize time
  Trajectory new_trajectory;
  std::vector<std::vector<Eigen::MatrixXd>> new_fingers;
  std::vector<std::pair<int, double>> traj_contrib;
  AdaptiveSubdivideTrajectory(params.trajectory,
                              params.fingers,
                              precision,
                              new_trajectory,
                              new_fingers,
                              traj_contrib);
  size_t n_trajectory = new_trajectory.size();
  std::vector<bool> traj_skip(n_trajectory - 1, false);
  std::vector<size_t> traj_subs(n_trajectory - 1, 0);

  // #pragma omp parallel for
  for (long long i = 0; i < n_trajectory - 1; i++) {
    double max_deviation = 0;
    bool intersects = false;
    for (size_t j = 0; j < new_fingers[i].size(); j++) {
      double dev = (new_fingers[i + 1][j] - new_fingers[i][j])
                       .rowwise()
                       .norm()
                       .maxCoeff();
      max_deviation = std::max(max_deviation, dev);
      Eigen::RowVector3d p_min =
          new_fingers[i][j].colwise().minCoeff().cwiseMin(
              new_fingers[i + 1][j].colwise().minCoeff());
      Eigen::RowVector3d p_max =
          new_fingers[i][j].colwise().maxCoeff().cwiseMax(
              new_fingers[i + 1][j].colwise().maxCoeff());
      p_min.array() -= precision;
      p_max.array() += precision;
      intersects =
          intersects || mdr.Intersects(Eigen::AlignedBox3d(p_min, p_max));

      if (debugger != nullptr) {
        for (size_t k = 1; k < new_fingers[i][j].rows(); k++) {
          debugger->AddEdge(new_fingers[i][j].row(k - 1),
                            new_fingers[i][j].row(k),
                            colors::kOrange);
        }
      }
    }

    traj_skip[i] = !intersects;
    traj_subs[i] =
        intersects ? std::max<size_t>(std::ceil(max_deviation / precision), 1)
                   : 1;
  }

  for (size_t i = 0; i < traj_skip.size(); i++) {
    std::cout << traj_skip[i];
  }
  std::cout << std::endl;
  for (size_t i = 0; i < traj_skip.size(); i++) {
    std::cout << traj_subs[i] << " ";
  }
  std::cout << std::endl;

  struct _TrajData {
    size_t traj_idx;
    double t;
    Pose pose;

    double factor;
    double dFactor_dJointi;
    size_t jointi_idx;
  };

  const double ang_velocity = settings.cost.ang_velocity;
  std::vector<_TrajData> traj_data;
  for (size_t i = 0; i < n_trajectory - 1; i++) {
    if (traj_skip[i]) continue;
    size_t subs = traj_subs[i];

    size_t duration_idx;
    bool duration_flip;
    double duration = ComputeDuration(new_trajectory[i],
                                      new_trajectory[i + 1],
                                      ang_velocity,
                                      duration_idx,
                                      duration_flip);

    double factor = duration / (subs * 2);
    double dFactor_dJointi = 1. / (subs * 2 * ang_velocity);
    if (duration_flip) dFactor_dJointi = -dFactor_dJointi;

    size_t traj_idx = traj_contrib[i].first;
    double t2 = traj_contrib[i + 1].second;
    if (traj_contrib[i + 1].first != traj_idx) t2 = 1;

    traj_data.push_back(_TrajData{traj_idx,
                                  traj_contrib[i].second,
                                  new_trajectory[i],
                                  0,
                                  0,
                                  duration_idx});

    for (size_t j = 1; j <= subs; j++) {
      double t = (double)j / subs;
      Pose pose = new_trajectory[i] * (1. - t) + new_trajectory[i + 1] * t;

      traj_data.back().factor += factor;
      traj_data.back().dFactor_dJointi += dFactor_dJointi;
      traj_data.push_back(_TrajData{traj_idx,
                                    traj_contrib[i].second * (1 - t) + t2 * t,
                                    pose,
                                    factor,
                                    dFactor_dJointi,
                                    duration_idx});
    }
  }

  std::cout << "dfinger: " << d_fingers.size()
            << ", traj_data: " << traj_data.size() << std::endl;

  const size_t n_fingers = params.fingers.size();
  const size_t n_joints = settings.finger.n_finger_joints;

  // dEval_dDFingers and dEval_dPose assumed to have the right size and zerod
  auto EvalAtPose =
      [&D_fingers, &d_fingers, &settings, &mdr, n_fingers, n_joints](
          const Pose pose,
          const JacobianFunc& J,
          Fingers& dEval_dDFingers,
          Eigen::Matrix<double, 1, kNumDOFs>& dEval_dPose) {
        auto f = TransformMatrix(D_fingers, robots::Forward(pose));
        double eval = 0;
        for (long long k = 0; k < f.rows(); k++) {
          const auto& data = d_fingers[k];
          Eigen::RowVector3d dEval_dPos;
          eval +=
              data.factor * EvalAt(f.row(k), settings.cost, mdr, dEval_dPos);

          Jacobian dPos_dPose = J(f.row(k));
          dEval_dPose += data.factor * (dEval_dPos * dPos_dPose);
          dEval_dDFingers[data.finger_idx].row(data.joint_idx) +=
              (data.factor * (1 - data.t)) * dEval_dPos;
          dEval_dDFingers[data.finger_idx].row(data.joint_idx + 1) +=
              (data.factor * data.t) * dEval_dPos;
        }
        return eval;
      };

  Fingers dCost_dFingers(n_fingers, Eigen::MatrixXd::Zero(n_joints, 3));
  Trajectory dCost_dTrajectory(params.trajectory.size(), Pose::Zero());
  double cost = 0;

#pragma omp parallel
  {
    Fingers t_dCost_dFingers(n_fingers, Eigen::MatrixXd::Zero(n_joints, 3));
    Trajectory t_dCost_dTrajectory(params.trajectory.size(), Pose::Zero());
    double t_cost = 0;

#pragma omp for nowait
    for (long long i = 0; i < traj_data.size(); i++) {
      const auto& data = traj_data[i];
      Eigen::Affine3d trans = robots::Forward(data.pose);
      Eigen::Matrix3d dDFinger_dFinger = (trans * finger_trans_inv).linear();
      JacobianFunc J = robots::ComputeJacobian(data.pose);
      Fingers dEval_dDFingers(n_fingers, Eigen::MatrixXd::Zero(n_joints, 3));
      Eigen::Matrix<double, 1, kNumDOFs> dEval_dPose;
      dEval_dPose.setZero();
      double eval = EvalAtPose(data.pose, J, dEval_dDFingers, dEval_dPose);

      t_cost += data.factor * eval;

      for (size_t j = 0; j < n_fingers; j++) {
        for (size_t k = 0; k < t_dCost_dFingers[j].rows(); k++) {
          t_dCost_dFingers[j].row(k) +=
              data.factor * (dEval_dDFingers[j].row(k) * dDFinger_dFinger);
        }
      }
      t_dCost_dTrajectory[data.traj_idx] +=
          (data.factor * (1 - data.t)) * dEval_dPose.array();
      t_dCost_dTrajectory[data.traj_idx + 1] +=
          (data.factor * data.t) * dEval_dPose.array();
      t_dCost_dTrajectory[data.traj_idx](data.jointi_idx) -=
          data.dFactor_dJointi * eval;
      t_dCost_dTrajectory[data.traj_idx + 1](data.jointi_idx) +=
          data.dFactor_dJointi * eval;
    }

#pragma omp critical
    {
      cost += t_cost;
      for (size_t j = 0; j < n_fingers; j++) {
        dCost_dFingers[j] += t_dCost_dFingers[j];
      }
      for (size_t j = 0; j < params.trajectory.size(); j++) {
        dCost_dTrajectory[j] += t_dCost_dTrajectory[j];
      }
    }
  }

  out_dCost_dParam.fingers = dCost_dFingers;
  out_dCost_dParam.trajectory = dCost_dTrajectory;
  return cost;
}

double MinDistance(const GripperParams& params,
                   const GripperSettings& settings,
                   const MeshDependentResource& mdr) {
  EASY_FUNCTION();

  constexpr double precision = 0.001;  // 1mm

  Eigen::Affine3d finger_trans_inv =
      robots::Forward(params.trajectory.front()).inverse();
  std::vector<Eigen::MatrixXd> fingers =
      TransformFingers(params.fingers, finger_trans_inv);

  // discretize fingers
  EASY_BLOCK("discretize fingers")
  std::vector<Eigen::Vector3d> d_fingers;
  for (size_t i = 0; i < fingers.size(); i++) {
    for (size_t j = 1; j < fingers[i].rows(); j++) {
      double norm = (fingers[i].row(j) - fingers[i].row(j - 1)).norm();
      size_t subs = std::ceil(norm / precision);
      size_t iters = subs;
      for (size_t k = (j == 1) ? 0 : 1; k <= subs; k++) {
        double t = (double)k / subs;
        d_fingers.push_back(fingers[i].row(j - 1) * (1. - t) +
                            fingers[i].row(j) * t);
      }
    }
  }
  Eigen::MatrixXd D_fingers(d_fingers.size(), 3);
#pragma omp parallel for
  for (long long i = 0; i < d_fingers.size(); i++) {
    D_fingers.row(i) = d_fingers[i];
  }
  EASY_END_BLOCK;

  // discretize time
  EASY_BLOCK("discretize time")
  Trajectory new_trajectory;
  std::vector<std::vector<Eigen::MatrixXd>> new_fingers;
  std::vector<std::pair<int, double>> traj_contrib;
  AdaptiveSubdivideTrajectory(params.trajectory,
                              params.fingers,
                              precision,
                              new_trajectory,
                              new_fingers,
                              traj_contrib);
  size_t n_trajectory = new_trajectory.size();

  double min_dist = 0;

  for (size_t i = 0; i < n_trajectory - 1; i++) {
    double max_deviation = 0;
    for (size_t j = 0; j < new_fingers[i].size(); j++) {
      double dev = (new_fingers[i + 1][j] - new_fingers[i][j])
                       .rowwise()
                       .norm()
                       .maxCoeff();
      max_deviation = std::max(max_deviation, dev);
    }
    size_t cur_sub = std::max<size_t>(std::ceil(max_deviation / precision), 1);

    size_t iters = cur_sub;
    if (i == n_trajectory - 2) iters++;

    for (size_t j = 0; j < iters; j++) {
      double t = (double)j / cur_sub;
      Pose pose = new_trajectory[i] * (1. - t) + new_trajectory[i + 1] * t;
      Eigen::MatrixXd f = TransformMatrix(D_fingers, robots::Forward(pose));
#pragma omp parallel
      {
        double t_min = 0;
        Eigen::RowVector3d ds_dp;  // unused
#pragma omp for nowait
        for (long long k = 0; k < f.rows(); k++) {
          t_min = std::min(t_min, GetDist(f.row(k), settings.cost, mdr, ds_dp));
        }
#pragma omp critical
        min_dist = std::min(min_dist, t_min);
      }
    }
  }
  EASY_END_BLOCK;
  return min_dist;
}

struct _SegState {
  bool is_first;
  bool is_in;
  size_t last_pos_vid;
  double last_pos_vid_dis;
  Eigen::Vector3d last_pos;  // last intersection entering the mesh
};

#define soft_assert(x) \
  if (!(x)) fprintf(stderr, "Assertion Error: " #x "" __FILE__ ":%d", __LINE__)

static double ComputeCollisionPenaltySegment(const Eigen::Vector3d& A,
                                             const Eigen::Vector3d& B,
                                             const MeshDependentResource& mdr,
                                             double geodesic_contrib,
                                             double inner_dis_contrib,
                                             _SegState& state,
                                             Debugger* const debugger,
                                             const Eigen::RowVector3d& color) {
  EASY_FUNCTION();

  const Eigen::MatrixXd& SP_ = mdr.GetSP();
  const Eigen::MatrixXi& SP_par_ = mdr.GetSPPar();
  Eigen::RowVector3d dir = B - A;
  double norm = dir.norm();
  if (norm < 1e-12 || isnan(norm)) return 0;
  dir /= norm;
  // std::cout << dir << std::endl;
  std::vector<igl::Hit> hits;
  int num_rays;

  EASY_BLOCK("intersectRay");
  mdr.intersector.intersectRay(
      A.cast<float>(), dir.cast<float>(), hits, num_rays);
  bool is_A_in = hits.size() % 2 == 1;
  EASY_END_BLOCK;
  EASY_BLOCK("After intersectRay");

  if (state.is_first) {
    state.is_in = is_A_in;
  }

  Eigen::RowVector3d color_inv = Eigen::RowVector3d::Ones() - color;

  Eigen::Vector3d last_hit = A;

  double total_dis = 0;
  for (const auto& hit : hits) {
    EASY_BLOCK("ProcessHit");
    if (hit.t >= norm) break;
    Eigen::RowVector3d P = A.transpose() + dir * hit.t;

    // find closest vertex for P
    size_t vid = -1;
    double best_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < 3; i++) {
      int u = mdr.F(hit.id, i);
      double d = (P - mdr.V.row(u)).squaredNorm();
      if (d < best_dist) {
        best_dist = d;
        vid = u;
      }
    }
    best_dist = sqrt(best_dist);

    if (state.is_in) {
      // --|P
      if (inner_dis_contrib != 0.) {
        total_dis += (P - last_hit.transpose()).norm() * inner_dis_contrib;
        if (debugger) {
          debugger->AddEdge(last_hit, P.transpose(), color);
        }
      }

      if (!state.is_first) {
        // state.last_pos |----| P
        soft_assert(state.last_pos_vid != -1llu);

        if (geodesic_contrib != 0.) {
          total_dis += (state.last_pos_vid_dis + SP_(state.last_pos_vid, vid) +
                        best_dist) *
                       geodesic_contrib;
          if (debugger) {
            debugger->AddEdge(
                state.last_pos, mdr.V.row(state.last_pos_vid), colors::kRed);
            debugger->AddEdge(mdr.V.row(vid), P, colors::kBrown);
            size_t cur = vid;
            while (cur != state.last_pos_vid) {
              size_t par = SP_par_(cur, state.last_pos_vid);
              debugger->AddEdge(
                  mdr.V.row(par), mdr.V.row(cur), colors::kPurple);
              cur = par;
            }
          }
        }
      }
      state.last_pos_vid = -1llu;
    } else {
      // P|--
      state.last_pos = P;
      state.last_pos_vid = vid;
      state.last_pos_vid_dis = best_dist;

      if (debugger) {
        debugger->AddEdge(last_hit, P.transpose(), color_inv);
      }
    }
    state.is_in = !state.is_in;
    state.is_first = false;
    last_hit = P;
  }

  if (state.is_in) {
    // |-- B
    if (inner_dis_contrib != 0.) {
      total_dis += (B - last_hit).norm() * inner_dis_contrib;
      if (debugger) {
        debugger->AddEdge(B, last_hit, color);
      }
    }
  } else {
    if (debugger) {
      debugger->AddEdge(B, last_hit, color_inv);
    }
  }
  return total_dis;
}

double ComputeFloorCost(Eigen::RowVector3d p0,
                        Eigen::RowVector3d p1,
                        double floor) {
  EASY_FUNCTION();

  if (p0.y() >= floor && p1.y() >= floor) return 0;
  Eigen::RowVector3d p01 = p1 - p0;
  if (p0.y() < floor != p1.y() < floor) {
    if (p1.y() < floor) std::swap(p0, p1);
    p01 *= (floor - p0.y()) / (p1.y() - p0.y());
  }
  double cost = p01.norm();
  p01.y() = 0;
  return cost + p01.norm();
}

double ComputeCost_SP(const GripperParams& params,
                      const GripperParams& init_params,
                      const GripperSettings& settings,
                      const MeshDependentResource& remeshed_mdr,
                      GripperParams& out_dCost_dParam,
                      Debugger* const debugger) {
  EASY_FUNCTION();

  struct _SubInfo {
    // Pose pose;
    Fingers fingers;
    std::vector<Eigen::VectorXd> costs;
  };

  if (params.fingers.empty()) return 0;

  Eigen::Affine3d finger_trans_inv =
      robots::Forward(params.trajectory.front()).inverse();
  std::vector<Eigen::MatrixXd> fingers =
      TransformFingers(params.fingers, finger_trans_inv);

  // std::cout << params.fingers[0] << std::endl;

  // Cost between two points
  const double floor = settings.cost.floor;
  const double inner_dis_contrib = settings.cost.inner_dis_contrib;
  const double geodesic_contrib = settings.cost.geodesic_contrib;
  auto MyCost =
      [floor, debugger, inner_dis_contrib, geodesic_contrib, &remeshed_mdr](
          const Eigen::RowVector3d& p0,
          const Eigen::RowVector3d& p1,
          _SegState& state,
          const Eigen::RowVector3d& color) -> double {
    EASY_BLOCK("MyCost", profiler::EasyBlockStatus::OFF_RECURSIVE);
    return ComputeCollisionPenaltySegment(p0,
                                          p1,
                                          remeshed_mdr,
                                          geodesic_contrib,
                                          inner_dis_contrib,
                                          state,
                                          debugger,
                                          color) +
           ComputeFloorCost(p0, p1, floor);
  };

  // Gripper energy at particular time
  auto ProcessFinger = [&MyCost](const Fingers& fingers) -> double {
    // EASY_BLOCK("ProcessFinger");
    double cost = 0;
    _SegState state;
    for (size_t i = 0; i < fingers.size(); i++) {
      state.is_first = true;
      for (size_t j = 0; j < fingers[i].rows() - 1; j++) {
        cost += MyCost(fingers[i].row(j),
                       fingers[i].row(j + 1),
                       state,
                       Eigen::RowVector3d(1, 0.5, 0));
      }
    }
    return cost;
  };

  // Linearize trajectory
  EASY_BLOCK("Linearize Trajectory");
  Trajectory new_trajectory;
  std::vector<Fingers> new_fingers;
  std::vector<std::pair<int, double>> traj_contrib;
  AdaptiveSubdivideTrajectory(params.trajectory,
                              params.fingers,
                              settings.cost.d_linearity,
                              new_trajectory,
                              new_fingers,
                              traj_contrib);
  size_t n_trajectory = new_trajectory.size();
  std::vector<Eigen::Affine3d> new_trans(new_trajectory.size());
  for (size_t i = 0; i < new_trajectory.size(); i++) {
    new_trans[i] = robots::Forward(new_trajectory[i]);
  }
  EASY_END_BLOCK;

  // ========================
  // Gripper Energy
  // ========================
  double gripper_energy = 0;

  if (settings.cost.gripper_energy != 0.) {
    EASY_BLOCK("Gripper Energy");

    // std::vector<bool> traj_skip(n_trajectory - 1, false);
    std::vector<double> traj_devs(n_trajectory - 1);
    // std::vector<size_t> traj_subs(n_trajectory - 1, 0);
    double total_dev = 0;

    EASY_BLOCK("Prepare Grip");

    // #pragma omp parallel for
    for (long long i = 0; i < n_trajectory - 1; i++) {
      double max_deviation = 0;
      bool intersects = false;
      for (size_t j = 0; j < new_fingers[i].size(); j++) {
        double dev = (new_fingers[i + 1][j] - new_fingers[i][j])
                         .rowwise()
                         .norm()
                         .maxCoeff();
        max_deviation = std::max(max_deviation, dev);
        Eigen::RowVector3d p_min =
            new_fingers[i][j].colwise().minCoeff().cwiseMin(
                new_fingers[i + 1][j].colwise().minCoeff());
        Eigen::RowVector3d p_max =
            new_fingers[i][j].colwise().maxCoeff().cwiseMax(
                new_fingers[i + 1][j].colwise().maxCoeff());
        p_min.array() -= settings.cost.d_subdivision;
        p_max.array() += settings.cost.d_subdivision;
        intersects = intersects ||
                     remeshed_mdr.Intersects(Eigen::AlignedBox3d(p_min, p_max));
      }

      // if (intersects)
      total_dev += max_deviation;
      traj_devs[i] = max_deviation;
      // traj_skip[i] = !intersects;
    }
    double d_sub = total_dev / settings.cost.n_finger_steps;
    std::vector<Pose> poses;
    poses.reserve(settings.cost.n_finger_steps + 32);
    for (long long i = 0; i < n_trajectory - 1; i++) {
      size_t cur_sub = std::ceil(traj_devs[i] / d_sub);
      size_t iters = cur_sub;
      if (i == n_trajectory - 2) iters++;
      for (long long j = 0; j < iters; j++) {
        double t = (double)j / cur_sub;
        Pose pose = new_trajectory[i] * (1. - t) + new_trajectory[i + 1] * t;
        poses.push_back(pose);
      }
    }
    EASY_END_BLOCK;

    EASY_BLOCK("Actual Grip");
    long long n_poses = poses.size();

#pragma omp parallel
    {
      double t_max = 0;

#pragma omp for nowait
      for (long long j = 0; j < n_poses; j++) {
        auto f = TransformFingers(fingers, robots::Forward(poses[j]));
        t_max = std::max(t_max, ProcessFinger(f));
      }
#pragma omp critical
      gripper_energy = std::max(gripper_energy, t_max);
    }

    /*
    size_t gripper_subs = 0;
    for (size_t i = 0; i < n_trajectory - 1; i++) {
      // if (traj_skip[i]) continue;
      size_t cur_sub = traj_subs[i];
      size_t iters = cur_sub;
      gripper_subs += cur_sub;
      if (i == n_trajectory - 2) iters++;

#pragma omp parallel
      {
        double t_max = 0;

#pragma omp for
        for (long long j = 0; j < iters; j++) {
          double t = (double)j / cur_sub;
          Pose pose = new_trajectory[i] * (1. - t) + new_trajectory[i + 1] * t;
          auto f = TransformFingers(fingers, robots::Forward(pose));
          t_max = std::max(t_max, ProcessFinger(f));
        }

#pragma omp critical
        gripper_energy = std::max(gripper_energy, t_max);
      }
    }
    */
    EASY_END_BLOCK;
    // std::cout << "gripper_sub: " << gripper_subs << std::endl;
  }

  // ========================
  // Trajectory Energy
  // ========================
  double trajectory_energy = 0;

  if (settings.cost.traj_energy != 0.) {
    EASY_BLOCK("Trajectory Energy");

    EASY_BLOCK("Prepare Traj");
    std::vector<Eigen::Vector3d> d_fingers;
    size_t traj_subs = 0;
    for (size_t i = 0; i < fingers.size(); i++) {
      double total_norm = 0;
      for (size_t j = 1; j < fingers[i].rows(); j++) {
        double norm = (fingers[i].row(j) - fingers[i].row(j - 1)).norm();
        total_norm += norm;
      }
      double d_sub =
          total_norm * fingers.size() / settings.cost.n_trajectory_steps;
      for (size_t j = 1; j < fingers[i].rows(); j++) {
        double norm = (fingers[i].row(j) - fingers[i].row(j - 1)).norm();
        size_t subs = std::ceil(norm / d_sub);
        traj_subs += subs;
        size_t iters = subs;
        for (size_t k = (j == 1) ? 0 : 1; k <= subs; k++) {
          double t = (double)k / subs;
          d_fingers.push_back(fingers[i].row(j - 1) * (1. - t) +
                              fingers[i].row(j) * t);
        }
      }
    }
    EASY_END_BLOCK;
    // std::cout << "traj_subs: " << traj_subs << std::endl;
    EASY_BLOCK("Actual Traj");
#pragma omp parallel
    {
      double t_max = 0;
      _SegState state;

#pragma omp for
      for (long long j = 0; j < d_fingers.size(); j++) {
        Eigen::Vector3d p0 = new_trans[0] * d_fingers[j];
        state.is_first = true;
        double cur_cost = 0;
        for (size_t k = 1; k < new_trajectory.size(); k++) {
          Eigen::Vector3d p1 = new_trans[k] * d_fingers[j];
          cur_cost += MyCost(p0, p1, state, Eigen::RowVector3d(1, 0, 0.5));
          p0 = p1;
        }
        t_max = std::max(t_max, cur_cost);
      }

#pragma omp critical
      trajectory_energy = std::max(trajectory_energy, t_max);
    }
    EASY_END_BLOCK;
  }

  // ========================
  // Robot floor collision
  // ========================
  EASY_BLOCK("Robot floor collision");
  double robot_floor = 0;
  double max_penetration = 0;
  constexpr double robot_clearance = 0.05;
  for (const auto& trans : new_trans) {
    max_penetration =
        std::max(max_penetration,
                 std::max(0., robot_clearance - trans.translation()(1)));
  }
  robot_floor = max_penetration;
  EASY_END_BLOCK;

  // ========================
  // L2 regularization term
  // ========================
  EASY_BLOCK("L2 regularization term");
  double traj_reg = 0;
  for (size_t i = 1; i < params.trajectory.size() - 1; i++) {
    traj_reg += (params.trajectory[i] - init_params.trajectory[i])
                    .matrix()
                    .squaredNorm();
  }
  EASY_END_BLOCK;

  // ========================
  // Total Cost
  // ========================
  return settings.cost.gripper_energy * gripper_energy +
         settings.cost.traj_energy * trajectory_energy +
         settings.cost.robot_collision * robot_floor +
         settings.cost.regularization * traj_reg;
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

  if (params.fingers.size() == 0 || params.trajectory.size() <= 1llu)
    return false;

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
