#include "Robots.h"

#define IKFAST_HAS_LIBRARY
#include "ikfast.h"

namespace psg {
namespace core {
namespace robots {
// Change axis
static const Eigen::Affine3d globalTrans =
    (Eigen::Affine3d)(Eigen::Matrix3d() << 1, 0, 0, 0, 0, 1, 0, -1, 0)
        .finished();
static const Eigen::Affine3d globalTransInv = globalTrans.inverse();

static void ForwardImpl(const Pose& jointConfig,
                        Eigen::Matrix3d& out_rot,
                        Eigen::Vector3d& out_trans) {
  double eerot[9];
  double eetrans[3];

  ComputeFk(&jointConfig[0], eetrans, eerot);

  for (size_t i = 0; i < 9; i++) {
    out_rot(i / 3, i % 3) = eerot[i];
  }
  for (size_t i = 0; i < 3; i++) {
    out_trans(i) = eetrans[i];
  }
}

Eigen::Affine3d robots::Forward(const Pose& jointConfig) {
  Eigen::Matrix3d rot;
  Eigen::Vector3d trans;
  ForwardImpl(jointConfig, rot, trans);
  Eigen::Affine3d a;
  a.setIdentity();
  a.translate(trans);
  a.rotate(rot);
  return globalTrans * a;
}

static bool InverseImpl(const Eigen::Matrix3d& rot,
                        const Eigen::Vector3d& trans,
                        std::vector<Pose>& out_jointConfigs) {
  using namespace ikfast;
  IkSolutionList<IkReal> solutions;
  std::vector<IkReal> vfree(GetNumFreeParameters());

  double eerot[9];
  double eetrans[3];
  for (int i = 0; i < 9; i++) {
    eerot[i] = rot(i / 3, i % 3);
  }
  for (int i = 0; i < 3; i++) {
    eetrans[i] = trans(i);
  }

  bool success =
      ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

  if (!success) {
    return false;
  }

  size_t numSolutions = solutions.GetNumSolutions();
  std::vector<IkReal> solvalues(GetNumJoints());
  out_jointConfigs.clear();

  for (size_t i = 0; i < numSolutions; i++) {
    const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
    size_t numSolFreeParams = sol.GetFree().size();
    std::vector<IkReal> vsolfree(numSolFreeParams);
    sol.GetSolution(solvalues, vsolfree);
    Pose p;
    std::copy_n(solvalues.begin(), kNumDOFs, p.begin());
    out_jointConfigs.push_back(p);
  }

  return true;
}

bool robots::Inverse(Eigen::Affine3d trans,
                     std::vector<Pose>& out_jointConfigs) {
  trans = globalTransInv * trans;
  return InverseImpl(trans.linear(), trans.translation(), out_jointConfigs);
}

static Eigen::Affine3d JointTransform(double theta,
                                      double a,
                                      double d,
                                      double alpha) {
  Eigen::Affine3d result;
  result = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) *
           Eigen::Translation3d(a, 0, d) *
           Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX());
  return result;
}

void robots::ForwardIntermediate(const Pose& jointConfig,
                                 std::vector<Eigen::Affine3d>& out_trans) {
  out_trans.resize(kNumDOFs);
  out_trans[0] =
      JointTransform(jointConfig[0], kRobotA[0], kRobotD[0], kRobotAlpha[0]);
  for (size_t i = 1; i < kNumDOFs; i++) {
    out_trans[i] =
        out_trans[i - 1] *
        JointTransform(jointConfig[i], kRobotA[i], kRobotD[i], kRobotAlpha[i]);
  }
  for (size_t i = 0; i < kNumDOFs; i++) {
    out_trans[i] = globalTrans * out_trans[i];
  }
}

}  // namespace robots
}  // namespace core
}  // namespace psg