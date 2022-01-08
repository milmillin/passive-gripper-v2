#include "Initialization.h"

#include <igl/random_points_on_mesh.h>
#include <random>
#include "GeometryUtils.h"
#include "QualityMetric.h"
#include "DiscreteDistanceField.h"
#include "robots/Robots.h"

#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

namespace psg {
namespace core {

// Distance from p to line ab
static bool PointToLineDist(const Eigen::Vector3d& a,
                            const Eigen::Vector3d& b,
                            const Eigen::Vector3d& p) {
  Eigen::Vector3d ab = b - a;
  Eigen::Vector3d ap = p - a;
  Eigen::Vector3d proj = (ap.dot(ab) / ab.squaredNorm()) * ab;
  return (ap - proj).norm();
}

static bool ShouldPopB(Eigen::Vector3d a,
                       Eigen::Vector3d c,
                       const MeshDependentResource& mdr) {
  Eigen::RowVector3d cc;
  double sign;
  Eigen::Vector3d ac = (c - a).normalized();
  a += ac * 1e-6;
  c -= ac * 1e-6;
  if (mdr.ComputeSignedDistance(a, cc, sign) < 0 ||
      mdr.ComputeSignedDistance(c, cc, sign) < 0)
    return false;

  igl::Hit hit;
  return !mdr.intersector.intersectSegment(
      a.transpose().cast<float>(), (c - a).transpose().cast<float>(), hit);
}

void InitializeMeshPosition(const Eigen::MatrixXd& V,
                            Eigen::MatrixXd& out_V,
                            Eigen::Affine3d& out_trans) {
  Eigen::Vector3d minimum = V.colwise().minCoeff();
  Eigen::Vector3d maximum = V.colwise().maxCoeff();

  Eigen::Vector3d translate(
      -minimum.x() / 2. - maximum.x() / 2., -minimum.y(), 0.07 - minimum.z());

  Eigen::MatrixXd SV = V.rowwise() + translate.transpose();
  Eigen::Translation3d mesh_trans(
      (SV.colwise().minCoeff() + SV.colwise().maxCoeff()) / 2.);

  Eigen::Affine3d trans = robots::Forward(kInitPose);
  SV = (trans * (SV.transpose().colwise().homogeneous())).transpose();

  double min_y = SV.colwise().minCoeff().y();
  SV.col(1).array() -= min_y;
  out_V = SV;
  out_trans = Eigen::Translation3d(0, -min_y, 0) * trans * mesh_trans;
}

Eigen::MatrixXd InitializeFinger(const ContactPoint contact_point,
                                 const MeshDependentResource& mdr,
                                 const Eigen::Vector3d& effector_pos,
                                 const std::vector<double>& dist,
                                 const std::vector<int>& par,
                                 size_t n_finger_joints) {
  Eigen::MatrixXd res(n_finger_joints, 3);

  size_t fid = mdr.ComputeClosestFacet(contact_point.position);
  size_t vid = -1;
  double bestDist = std::numeric_limits<double>::max();
  double curDist;
  for (int j = 0; j < 3; j++) {
    int v = mdr.F(fid, j);
    if ((curDist = (contact_point.position - mdr.V.row(v).transpose()).norm() +
                   dist[v]) < bestDist) {
      bestDist = curDist;
      vid = v;
    }
  }
  std::vector<Eigen::Vector3d> finger;
  std::vector<int> fingerVid;
  finger.push_back(contact_point.position);
  fingerVid.push_back(-1);
  while (vid != -1) {
    Eigen::Vector3d toPush = mdr.V.row(vid);
    while (finger.size() > 1 &&
           ShouldPopB(finger[finger.size() - 2], toPush, mdr)) {
      finger.pop_back();
      fingerVid.pop_back();
    }
    finger.push_back(toPush);
    fingerVid.push_back(vid);
    vid = par[vid];
  }
  finger.push_back(effector_pos);
  fingerVid.push_back(-1);

  // Expand segment by 0.01 or half the clearance
  for (size_t j = 1; j < finger.size() - 1; j++) {
    Eigen::RowVector3d normal = mdr.VN.row(fingerVid[j]);
    igl::Hit hit;
    double avail_dis = 0.01;
    if (mdr.intersector.intersectRay(
            (mdr.V.row(fingerVid[j]) + normal * 1e-6).cast<float>(),
            normal.cast<float>(),
            hit)) {
      avail_dis = std::min(avail_dis, hit.t / 2.);
    }
    finger[j] += mdr.VN.row(fingerVid[j]) * avail_dis;
  }

  // Fix number of segment
  while (finger.size() > n_finger_joints) {
    size_t bestId = -1llu;
    size_t bestFallbackId = -1llu;
    double best = std::numeric_limits<double>::max();
    double bestFallback = std::numeric_limits<double>::max();
    for (size_t j = 1; j < finger.size() - 1; j++) {
      double cost = PointToLineDist(finger[j - 1], finger[j + 1], finger[j]);
      if (ShouldPopB(finger[j - 1], finger[j + 1], mdr)) {
        if (cost < best) {
          best = cost;
          bestId = j;
        }
      }
      if (cost < bestFallback) {
        bestFallback = cost;
        bestFallbackId = j;
      }
    }
    if (bestId == -1llu) bestId = bestFallbackId;
    finger.erase(finger.begin() + bestId);
  }
  while (finger.size() < n_finger_joints) {
    size_t bestId = -1llu;
    double best = -1;
    for (size_t j = 1; j < finger.size(); j++) {
      double cur = (finger[j] - finger[j - 1]).squaredNorm();
      if (cur > best) {
        best = cur;
        bestId = j;
      }
    }
    finger.insert(finger.begin() + bestId,
                  (finger[bestId] + finger[bestId - 1]) / 2.);
  }

  for (size_t j = 0; j < n_finger_joints; j++) {
    res.row(j) = finger[j];
  }
  return res;
}

std::vector<Eigen::MatrixXd> InitializeFingers(
    const std::vector<ContactPoint>& contact_points,
    const MeshDependentResource& mdr,
    const Eigen::Vector3d& effector_pos,
    size_t n_finger_joints) {
  std::vector<double> dist;
  std::vector<int> par;
  ComputeConnectivityFrom(mdr, effector_pos, dist, par);

  std::vector<Eigen::MatrixXd> res(contact_points.size());
  for (size_t i = 0; i < contact_points.size(); i++) {
    res[i] = InitializeFinger(
        contact_points[i], mdr, effector_pos, dist, par, n_finger_joints);
  }
  return res;
}

static void LengthParameterize(const Eigen::MatrixXd& V,
                               size_t nSteps,
                               Eigen::MatrixXd& out_V) {
  std::vector<double> cumDis(V.rows(), 0);
  for (size_t i = 1; i < V.rows(); i++) {
    cumDis[i] = cumDis[i - 1] + (V.row(i) - V.row(i - 1)).norm();
  }
  double disStep = cumDis.back() / nSteps;
  out_V.resize(nSteps + 1, 3);
  out_V.row(0) = V.row(0);
  double curDis = 0;
  size_t curV = 0;
  for (size_t i = 1; i < nSteps; i++) {
    double targetStep = i * disStep;
    while (curV + 1 < V.rows() && cumDis[curV + 1] < targetStep) {
      curV++;
    }
    double t = (targetStep - cumDis[curV]) / (cumDis[curV + 1] - cumDis[curV]);
    out_V.row(i) = V.row(curV + 1) * t + V.row(curV) * (1 - t);
  }
  out_V.row(nSteps) = V.row(V.rows() - 1);
}

Trajectory InitializeTrajectory(const std::vector<Eigen::MatrixXd>& fingers,
                                const Pose& initPose,
                                size_t n_keyframes) {
  static constexpr size_t subdivide = 4;
  const size_t nSize = (n_keyframes - 1) * subdivide;
  Eigen::MatrixXd avgNorms(nSize, 3);
  avgNorms.setZero();
  Eigen::Affine3d fingerTransInv = robots::Forward(initPose).inverse();
  double minY = -0.05;
  for (size_t i = 0; i < fingers.size(); i++) {
    Eigen::MatrixXd finger;
    LengthParameterize(fingers[i], nSize, finger);
    avgNorms += (finger.block(1, 0, nSize, 3) - finger.block(0, 0, nSize, 3));
    Eigen::MatrixXd transformedFinger =
        (fingerTransInv * fingers[i].transpose().colwise().homogeneous())
            .transpose();
    minY = std::min(minY, transformedFinger.colwise().minCoeff()(1));
  }
  avgNorms /= fingers.size();
  Eigen::MatrixXd trans(n_keyframes, 3);
  trans.row(0).setZero();
  for (size_t i = 0; i < n_keyframes - 1; i++) {
    trans.row(i + 1) =
        trans.row(i) +
        avgNorms.block(i * subdivide, 0, subdivide, 3).colwise().sum();
  }

  Trajectory result;
  result.reserve(n_keyframes);
  result.push_back(initPose);
  Eigen::Affine3d cumTrans = robots::Forward(initPose);
  for (size_t i = 1; i < n_keyframes; i++) {
    cumTrans = Eigen::Translation3d(trans.row(i)) * cumTrans;
    Eigen::Affine3d curTrans = cumTrans;
    curTrans.translation().y() =
        std::max(curTrans.translation().y(), -minY + 0.003);
    std::vector<Pose> candidates;
    if (robots::Inverse(curTrans, candidates)) {
      double best = std::numeric_limits<double>::max();
      double cur;
      size_t bestI = -1;
      for (size_t j = 0; j < candidates.size(); j++) {
        if ((cur = SumSquaredAngularDistance(result.back(), candidates[j])) <
            best) {
          best = cur;
          bestI = j;
        }
      }
      result.push_back(FixAngles(result.back(), candidates[bestI]));
    }
  }
  return result;
}

std::vector<ContactPointMetric> InitializeContactPoints(
    const MeshDependentResource& mdr,
    const MeshDependentResource& mdr_floor,
    const ContactSettings& settings,
    const Eigen::Vector3d& effector_pos,
    size_t num_candidates,
    size_t num_seeds) {

  std::vector<double> v_dist;
  std::vector<int> v_par;
  ComputeConnectivityFrom(mdr_floor, effector_pos, v_dist, v_par);

  std::vector<int> FI;
  std::vector<Eigen::Vector3d> X;

  while (FI.size() < num_seeds) {
    Eigen::MatrixXd B_;
    Eigen::VectorXi FI_;
    Eigen::MatrixXd X_;
    igl::random_points_on_mesh(num_seeds, mdr.V, mdr.F, B_, FI_, X_);
    for (long long i = 0; i < X_.rows(); i++) {
      Eigen::RowVector3d x = X_.row(i);
      // filter floor
      if (x.y() <= settings.floor) continue;
      // filter unreachable point
      if (v_par[mdr_floor.ComputeClosestVertex(x)] == -2) continue;
      FI.push_back(FI_(i));
      X.push_back(X_.row(i));
    }
  }
  std::cout << "Num seeds: " << X.size() << std::endl;

  std::mt19937 gen;
  std::uniform_int_distribution<int> dist(0, num_seeds - 1);

  std::vector<ContactPointMetric> prelim;
  prelim.reserve(num_candidates);

  // To be used for tolerance check
  NeighborInfo neighborInfo;
  std::cout << "Building neighbor info" << std::endl;
  buildNeighborInfo(mdr.F, neighborInfo);
  std::cout << "Done building neighbor info" << std::endl;

  std::cout << "Building distance field" << std::endl;
  DiscreteDistanceField distanceField(mdr.V, mdr.F, 50, effector_pos);
  std::cout << "Done building distance field" << std::endl;
#pragma omp parallel
  {
    while (true) {
      bool toContinue;
#pragma omp critical
      { toContinue = prelim.size() < num_candidates; }
      if (!toContinue) break;

      // Random 3 contact points
      int pids[3] = {dist(gen), dist(gen), dist(gen)};
      if (pids[0] == pids[1] || pids[1] == pids[2] || pids[0] == pids[2])
        continue;
      std::vector<ContactPoint> contactPoints(3);
      std::vector<std::vector<int>> neighbors;
      // std::cout << "0" << std::endl;
      for (int i = 0; i < 3; i++) {
        contactPoints[i].position = X[pids[i]];
        contactPoints[i].normal = mdr.FN.row(FI[pids[i]]);
        contactPoints[i].fid = FI[pids[i]];

        // Check tolerance
        neighbors.push_back(getNeighbors(neighborInfo, contactPoints[i], mdr.V, mdr.F, 0.01));
      }
      // std::cout << "1" << std::endl;

      // Check Feasibility: Minimum Wrench
      bool passMinimumWrench = true;
      std::vector<ContactPoint> contact_cones;
      double partialMinWrench = 0;
      int sample;
      for (sample = 0; sample < 20; sample++) {
        std::vector<ContactPoint> sample_contact_cones;
        sample_contact_cones.reserve(3 * settings.cone_res);
        for (size_t i = 0; i < 3; i++) {
          const auto&& cone = GenerateContactCone(contactPoints[i],
                                                  settings.cone_res,
                                                  settings.friction);
          sample_contact_cones.insert(sample_contact_cones.end(), cone.begin(), cone.end());
        }

      // std::cout << "2" << std::endl;
        double samplePartialMinWrench =
            ComputePartialMinWrenchQP(sample_contact_cones,
                                      mdr.center_of_mass,
                                      -Eigen::Vector3d::UnitY(),
                                      Eigen::Vector3d::Zero());

      // std::cout << "3" << std::endl;
        if (sample == 0) {
          contact_cones = sample_contact_cones;
          partialMinWrench = samplePartialMinWrench;
        }

        if (samplePartialMinWrench == 0) {
          passMinimumWrench = false;
          break;
        }

      // std::cout << "4" << std::endl;
        // Prepare next sample
        for (size_t i = 0; i < 3; i++) {
          std::uniform_int_distribution<> dist(0, neighbors[i].size() - 1);
          int fid = neighbors[i][dist(gen)];
          contactPoints[i].normal = mdr.FN.row(fid);
        }
      }
      // Get at least a partial closure
      if (!passMinimumWrench) {
        // std::cout << "Failed after " << sample << " samples" << std::endl;
        continue;
      }

      for (int i = 0; i < 3; i++) {
        contactPoints[i].normal = mdr.FN.row(contactPoints[i].fid);
      }

      // Check Feasiblity: Approach Direction
      Eigen::Affine3d trans;
      if (!CheckApproachDirection(
              contactPoints,
              kPi / 2 * 8 / 9,
              1,
              0.1,
              1e-12,
              100,
              trans)) {
        continue;
      }
      // if (!CheckApproachDirection(
      //         contactPoints, 0.01, kDegToRad * 80, mdr.center_of_mass, trans)) {
      //   continue;
      // }

      double minWrench = ComputeMinWrenchQP(contact_cones, mdr.center_of_mass);

      ContactPointMetric candidate;
      candidate.contact_points = contactPoints;
      candidate.partial_min_wrench = partialMinWrench;
      candidate.min_wrench = minWrench;
      candidate.trans = trans;
      candidate.finger_distance = getFingerDistance(distanceField, contactPoints);
#pragma omp critical
      {
        prelim.push_back(candidate);
        if (prelim.size() % 10 == 0)
          std::cout << "prelim prog: " << prelim.size() << "/" << num_candidates
                    << std::endl;
      }
    }
  }

  std::sort(prelim.begin(),
            prelim.end(),
            [](const ContactPointMetric& a, const ContactPointMetric& b) {
              if (a.finger_distance == b.finger_distance)
                return a.partial_min_wrench > b.partial_min_wrench;
              return a.finger_distance < b.finger_distance;
            });

  // Remove solutions that are not in the frontier
  double partial_min_wrench = 0;
  for (int i = 0; i < prelim.size(); i++) {
    if (prelim[i].partial_min_wrench >= partial_min_wrench) {
      partial_min_wrench = prelim[i].partial_min_wrench;
    } else {
      prelim.erase(prelim.begin() + i);
      i--;
    }
  }

  return prelim;
}

void InitializeGripperBound(const PassiveGripper& psg,
                            Eigen::Vector3d& out_lb,
                            Eigen::Vector3d& out_ub) {
  out_lb.setZero();
  out_ub.setZero();

  double attachment_r = psg.GetTopoOptSettings().attachment_size / 2.;
  out_lb.x() = out_lb.y() = -attachment_r;
  out_ub.x() = out_ub.y() = attachment_r;

  Eigen::Affine3d finger_trans_inv = psg.GetFingerTransInv();
  for (const Eigen::MatrixXd& finger : psg.GetFingers()) {
    Eigen::MatrixXd transformedFinger =
        (finger_trans_inv * finger.transpose().colwise().homogeneous())
            .transpose();
    out_lb =
        out_lb.cwiseMin(transformedFinger.colwise().minCoeff().transpose());
    out_ub =
        out_ub.cwiseMax(transformedFinger.colwise().maxCoeff().transpose());
  }
  constexpr double padding = 0.03;
  out_lb.array() -= padding;
  out_ub.array() += padding;
  out_lb.z() = 0;
}

void InitializeConservativeBound(const PassiveGripper& psg,
                                 Eigen::Vector3d& out_lb,
                                 Eigen::Vector3d& out_ub) {
  out_lb.setZero();
  out_ub.setZero();

  double attachment_r = psg.GetTopoOptSettings().attachment_size / 2.;
  out_lb.x() = out_lb.y() = -attachment_r;
  out_ub.x() = out_ub.y() = attachment_r;

  Eigen::Affine3d finger_trans_inv = psg.GetFingerTransInv();

  auto V = (psg.GetFingerTransInv() *
            psg.GetMDR().V.transpose().colwise().homogeneous())
               .transpose();

  out_lb = out_lb.cwiseMin(V.colwise().minCoeff().transpose());
  out_ub = out_ub.cwiseMax(V.colwise().maxCoeff().transpose());

  constexpr double padding = 0.03;
  out_lb.array() -= padding;
  out_ub.array() += padding;
  out_lb.z() = 0;
}

}  // namespace core
}  // namespace psg