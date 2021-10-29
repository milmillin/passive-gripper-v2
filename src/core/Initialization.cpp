#include "Initialization.h"

#include <igl/random_points_on_mesh.h>
#include <random>
#include "GeometryUtils.h"
#include "QualityMetric.h"
#include "robots/Robots.h"

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

Eigen::MatrixXd InitializeFinger(const ContactPoint& contactPoint,
                                 const MeshDependentResource& mdr,
                                 const Eigen::Vector3d& effectorPos,
                                 size_t n_finger_joints) {
  // Preprocess Mesh
  struct VertexInfo {
    int id;
    double dist;
    bool operator<(const VertexInfo& r) const { return dist > r.dist; }
  };
  struct EdgeInfo {
    int id;
    double dist;
  };
  std::vector<double> dist(mdr.V.rows(), std::numeric_limits<double>::max());
  std::vector<int> par(mdr.V.rows(), -2);
  std::vector<std::vector<EdgeInfo>> edges(mdr.V.rows());
  std::priority_queue<VertexInfo> q;

  for (size_t i = 0; i < mdr.V.rows(); i++) {
    Eigen::RowVector3d direction = mdr.V.row(i) - effectorPos.transpose();
    igl::Hit hit;
    direction -= direction.normalized() * 1e-7;
    if (!mdr.intersector.intersectSegment(effectorPos.transpose().cast<float>(),
                                          direction.cast<float>(),
                                          hit)) {
      dist[i] = (mdr.V.row(i) - effectorPos.transpose()).norm();
      par[i] = -1;
      q.push(VertexInfo{(int)i, dist[i]});
    }
  }
  for (size_t i = 0; i < mdr.F.rows(); i++) {
    for (int iu = 0; iu < 3; iu++) {
      int u = mdr.F(i, iu);
      int v = mdr.F(i, (iu + 1) % 3);
      edges[u].push_back(EdgeInfo{v, (mdr.V.row(v) - mdr.V.row(u)).norm()});
    }
  }
  while (!q.empty()) {
    VertexInfo now = q.top();
    q.pop();
    double nextDist;
    for (const auto& next : edges[now.id]) {
      if ((nextDist = dist[now.id] + next.dist) < dist[next.id]) {
        dist[next.id] = nextDist;
        par[next.id] = now.id;
        q.push(VertexInfo{next.id, nextDist});
      }
    }
  }
  Eigen::MatrixXd res(n_finger_joints, 3);

  size_t fid = mdr.ComputeClosestFacet(contactPoint.position);
  size_t vid = -1;
  double bestDist = std::numeric_limits<double>::max();
  double curDist;
  for (int j = 0; j < 3; j++) {
    int v = mdr.F(fid, j);
    if ((curDist = (contactPoint.position - mdr.V.row(v).transpose()).norm() +
                   dist[v]) < bestDist) {
      bestDist = curDist;
      vid = v;
    }
  }
  std::vector<Eigen::Vector3d> finger;
  std::vector<int> fingerVid;
  finger.push_back(contactPoint.position);
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
  finger.push_back(effectorPos);
  fingerVid.push_back(-1);

  // Expand segment by 0.01
  for (size_t j = 1; j < finger.size() - 1; j++) {
    finger[j] += mdr.VN.row(fingerVid[j]) * 0.01;
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
  for (size_t i = 0; i < fingers.size(); i++) {
    Eigen::MatrixXd finger;
    LengthParameterize(fingers[i], nSize, finger);
    avgNorms += (finger.block(1, 0, nSize, 3) - finger.block(0, 0, nSize, 3));
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
  Eigen::Affine3d curTrans = robots::Forward(initPose);
  for (size_t i = 1; i < n_keyframes; i++) {
    curTrans = Eigen::Translation3d(trans.row(i)) * curTrans;
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
    const GripperSettings& settings,
    size_t num_candidates,
    size_t num_seeds) {
  std::vector<ContactPointMetric> prelim;
  prelim.reserve(num_candidates);

  std::vector<int> FI;
  std::vector<Eigen::Vector3d> X;

  while (FI.size() < num_seeds) {
    Eigen::MatrixXd B_;
    Eigen::VectorXi FI_;
    Eigen::MatrixXd X_;
    igl::random_points_on_mesh(num_seeds, mdr.V, mdr.F, B_, FI_, X_);
    for (long long i = 0; i < X_.rows(); i++) {
      if (X_.row(i).y() > settings.cost.floor) {
        FI.push_back(FI_(i));
        X.push_back(X_.row(i));
      }
    }
  }
  std::cout << "Num seeds: " << X.size() << std::endl;

  std::mt19937 gen;
  std::uniform_int_distribution<int> dist(0, num_seeds - 1);

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
      for (int i = 0; i < 3; i++) {
        contactPoints[i].position = X[pids[i]];
        contactPoints[i].normal = mdr.FN.row(FI[pids[i]]);
        contactPoints[i].fid = FI[pids[i]];
      }

      // Check Feasibility
      std::vector<ContactPoint> contact_cones;
      contact_cones.reserve(3 * settings.contact.cone_res);
      for (size_t i = 0; i < 3; i++) {
        const auto&& cone = GenerateContactCone(contactPoints[i],
                                                settings.contact.cone_res,
                                                settings.contact.friction);
        contact_cones.insert(contact_cones.end(), cone.begin(), cone.end());
      }

      double partialMinWrench =
          ComputePartialMinWrenchQP(contact_cones,
                                    mdr.center_of_mass,
                                    -Eigen::Vector3d::UnitY(),
                                    Eigen::Vector3d::Zero());
      // Get at least a partial closure
      if (partialMinWrench == 0) continue;

      double minWrench = ComputeMinWrenchQP(contact_cones, mdr.center_of_mass);

      ContactPointMetric candidate;
      candidate.contact_points = contactPoints;
      candidate.partial_min_wrench = partialMinWrench;
      candidate.min_wrench = minWrench;
#pragma omp critical
      {
        prelim.push_back(candidate);
        if (prelim.size() % 100 == 0)
          std::cout << "prelim prog: " << prelim.size() << "/" << num_candidates
                    << std::endl;
      }
    }
  }

  std::sort(prelim.begin(),
            prelim.end(),
            [](const ContactPointMetric& a, const ContactPointMetric& b) {
              if (a.min_wrench == b.min_wrench)
                return a.partial_min_wrench > b.partial_min_wrench;
              return a.min_wrench > b.min_wrench;
            });
  return prelim;
}

}  // namespace core
}  // namespace psg