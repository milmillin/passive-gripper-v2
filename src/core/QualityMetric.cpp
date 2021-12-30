#include "QualityMetric.h"

#include <CGAL/QP_functions.h>
#include <CGAL/QP_models.h>

#include "GeometryUtils.h"

#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

// #ifdef CGAL_USE_GMP
// #include <CGAL/Gmpzf.h>
// typedef CGAL::Gmpzf ET;
// #else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
// #endif

namespace psg {
namespace core {
static Eigen::MatrixXd CreateGraspMatrix(
    const std::vector<ContactPoint>& contactCones,
    const Eigen::Vector3d& centerOfMass) {
  size_t nContacts = contactCones.size();
  Eigen::MatrixXd G(6, nContacts);
  for (size_t i = 0; i < nContacts; i++) {
    G.block<3, 1>(0, i) = -contactCones[i].normal;
    G.block<3, 1>(3, i) = (contactCones[i].position - centerOfMass)
                              .cross(-contactCones[i].normal);
  }
  return G;
}

static CGAL::Quotient<ET> MinNormVectorInFacet(const Eigen::MatrixXd& facet) {
  typedef CGAL::Quadratic_program<double> Program;
  typedef CGAL::Quadratic_program_solution<ET> Solution;

  size_t dim = facet.cols();

  Eigen::MatrixXd G;
  G = facet.transpose() * facet;
  G.diagonal().array() += kWrenchReg;
  G *= 2;

  // Solve QP to minimize x'Dx + c'x subject to Ax = B, x >= 0
  Program qp;

  // 1'x = 1
  for (size_t i = 0; i < dim; i++) {
    qp.set_a(i, 0, 1);
  }
  qp.set_b(0, 1);

  for (size_t i = 0; i < dim; i++) {
    for (size_t j = 0; j <= i; j++) {
      qp.set_d(i, j, G(i, j));
    }
  }

  Solution s = CGAL::solve_quadratic_program(qp, ET());
  return s.objective_value();
}

static CGAL::Quotient<ET> WrenchInPositiveSpan(
    const Eigen::MatrixXd& wrenchBasis,
    const Eigen::VectorXd& targetWrench) {
  typedef CGAL::Quadratic_program<double> Program;
  typedef CGAL::Quadratic_program_solution<ET> Solution;

  // min (targetWrench - wrenchBasis * x)^2

  Eigen::MatrixXd D = wrenchBasis.transpose() * wrenchBasis;
  D.diagonal().array() += kWrenchReg;

  Eigen::VectorXd c = -wrenchBasis.transpose() * targetWrench;

  // Solve QP to minimize x'Dx + c'x subject to Ax <= B, x >= 0
  Program qp(CGAL::SMALLER);

  // L1 finger contstraints
  /*
  size_t nWrenchesPerFinger = wrenchBasis.cols() / nFingers;
  for (size_t i = 0; i < nFingers; i++) {
    for (size_t j = 0; j < nWrenchesPerFinger; j++) {
      qp.set_a(i * nWrenchesPerFinger + j, i, 1);
    }
    qp.set_b(i, forceLimit);
  }
  */

  for (size_t i = 0; i < D.cols(); i++) {
    for (size_t j = 0; j <= i; j++) {
      qp.set_d(i, j, D(i, j));
    }
    qp.set_c(i, c(i));
  }

  Solution s = CGAL::solve_quadratic_program(qp, ET());
  /*
  for (auto it = s.variable_numerators_begin();
       it != s.variable_numerators_end();
       it++) {
    auto aa = *it;
    CGAL::to_double(aa);
  }
  */
  return s.objective_value() * 2 + targetWrench.squaredNorm();
}

bool CheckForceClosureQP(const std::vector<ContactPoint>& contactCones,
                         const Eigen::Vector3d& centerOfMass) {
  Eigen::MatrixXd G = CreateGraspMatrix(contactCones, centerOfMass);
  return MinNormVectorInFacet(G) < kWrenchNormThresh;
}

bool CheckPartialClosureQP(const std::vector<ContactPoint>& contactCones,
                           const Eigen::Vector3d& centerOfMass,
                           const Eigen::Vector3d& extForce,
                           const Eigen::Vector3d& extTorque) {
  Eigen::MatrixXd G = CreateGraspMatrix(contactCones, centerOfMass);
  Eigen::VectorXd targetWrench(6);
  targetWrench.block<3, 1>(0, 0) = -extForce;
  targetWrench.block<3, 1>(3, 0) = -extTorque;
  return WrenchInPositiveSpan(G, targetWrench) < kWrenchNormThresh;
}

double ComputeMinWrenchQP(const std::vector<ContactPoint>& contactCones,
                          const Eigen::Vector3d& centerOfMass) {
  Eigen::MatrixXd G = CreateGraspMatrix(contactCones, centerOfMass);
  if (MinNormVectorInFacet(G) >= kWrenchNormThresh) {
    // Zero not in convex hull
    return 0;
  }

  // Compute Convex Hull
  std::vector<size_t> hullIndices;
  std::vector<std::vector<size_t>> facets;
  if (ComputeConvexHull(G.transpose(), hullIndices, facets)) {
    CGAL::Quotient<ET> minDist;
    bool valid = false;
    // Compare against every facet
    for (const auto& facet : facets) {
      Eigen::MatrixXd F(6, facet.size());
      for (size_t i = 0; i < facet.size(); i++) {
        F.col(i) = G.col(facet[i]);
      }
      auto dist = MinNormVectorInFacet(F);
      if (!valid || dist < minDist) {
        minDist = dist;
        valid = true;
      }
    }
    if (!valid) std::cout << "Error: empty facet" << std::endl;
    return CGAL::to_double(minDist);
  }
  return 0;
}

double ComputePartialMinWrenchQP(const std::vector<ContactPoint>& contactCones,
                                 const Eigen::Vector3d& centerOfMass,
                                 const Eigen::Vector3d& extForce,
                                 const Eigen::Vector3d& extTorque) {
  Eigen::MatrixXd G = CreateGraspMatrix(contactCones, centerOfMass);
  Eigen::VectorXd targetWrench(6);
  targetWrench.block<3, 1>(0, 0) = -extForce;
  targetWrench.block<3, 1>(3, 0) = -extTorque;
  if (WrenchInPositiveSpan(G, targetWrench) >= kWrenchNormThresh) {
    // Not Partial Closure
    return 0.;
  }

  // Compute Convex Hull with Zero
  Eigen::MatrixXd V(G.cols() + 1, 6);
  V.block(1, 0, G.cols(), 6) = G.transpose();
  V.row(0).setZero();
  std::vector<size_t> hullIndices;
  std::vector<std::vector<size_t>> facets;
  if (ComputeConvexHull(V, hullIndices, facets)) {
    CGAL::Quotient<ET> minDist;
    bool valid = false;
    // Check against every face with Zero
    for (const auto& facet : facets) {
      bool zeroInFacet = false;
      for (size_t i : facet) {
        if (i == 0) {
          zeroInFacet = true;
          break;
        }
      }
      if (!zeroInFacet) continue;

      Eigen::MatrixXd F(6, facet.size() - 1);
      size_t id = 0;
      for (size_t i : facet) {
        if (i == 0) continue;
        F.col(id++) = G.col(i - 1);
      }
      auto dist = WrenchInPositiveSpan(F, targetWrench);
      if (!valid || dist < minDist) {
        minDist = dist;
        valid = true;
      }
    }
    if (valid)
      return CGAL::to_double(minDist);
    else
      return std::numeric_limits<double>::max();
  }
  return 0.0;
}

using autodiff::real;
using autodiff::Vector3real;

static real lossFn(const std::vector<Vector3real>& positions,
                   const std::vector<Vector3real>& normals,
                   Vector3real& trans,
                   Vector3real& rot,
                   Vector3real& center,
                   double maxCos,
                   double maxV) {
  assert(positions.size() == normals.size());
  auto n = positions.size();

  real loss = 0;
  for (size_t i = 0; i < n; i++) {
    Vector3real v = trans + rot.cross(positions[i] - center);
    real x = std::max<real>(maxCos - v.normalized().dot(normals[i]), 0);
    real y = std::max<real>(0.001 - v.norm(), 0);
    loss += x * x + y * y;
    // loss += std::max<real>(maxCos - v.dot(normals[i]), 0) +
    // std::max<real>(v.norm() - maxV, 0);
  }
  return loss;
}

bool CheckApproachDirection(const std::vector<ContactPoint>& contactPoints,
                            double maxAngle,
                            double maxV,
                            double learningRate,
                            double threshold,
                            int max_iterations) {
  using autodiff::at;
  using autodiff::gradient;
  using autodiff::Matrix3real;
  using autodiff::wrt;

  Vector3real trans = Vector3real::Zero();
  Vector3real rot = Vector3real::Zero();
  Vector3real center = Vector3real::Zero();

  std::vector<Vector3real> positions;
  std::vector<Vector3real> normals;
  for (const auto& cp : contactPoints) {
    positions.push_back(cp.position);
    normals.push_back(cp.normal.normalized());
  }

  double maxCos = std::cos(maxAngle);

  real loss;
  for (int i = 0; i < max_iterations; ++i) {
    Eigen::VectorXd grad =
        gradient(lossFn,
                 wrt(trans, rot, center),
                 at(positions, normals, trans, rot, center, maxCos, maxV),
                 loss);
    grad *= learningRate;

    if (loss < threshold) {
      std::cout << i << " loss: " << loss << std::endl;
      return true;
    }

    trans -= grad.block(0, 0, 3, 1);
    rot -= grad.block(3, 0, 3, 1);
    center -= grad.block(6, 0, 3, 1);
  }
  std::cout << "failed: " << loss << std::endl;
  return false;
}

static real LossFn2(const std::vector<Vector3real>& positions,
                    const std::vector<Vector3real>& normals,
                    Vector3real& trans,
                    Vector3real& rot,
                    Vector3real& center,
                    real away_dist) {
  assert(positions.size() == normals.size());
  size_t n = positions.size();

  real loss = 0;
  for (size_t i = 0; i < n; i++) {
    Vector3real v = trans + rot.cross(positions[i] - center);
    // real x = std::max<real>(away_dist - v.norm(), 0);
    // real y = std::max<real>(max_cos - v.normalized().dot(normals[i]), 0);
    // real y = std::max<real>(v.norm() - limit, 0);
    real x = std::max<real>(away_dist - v.dot(normals[i]), 0);
    loss += x * x;
  }
  return loss;
}

bool CheckApproachDirection2(const std::vector<ContactPoint>& contactPoints,
                             double away_dist,
                             double max_angle,
                             const Eigen::Vector3d& center_of_mass,
                             Eigen::Affine3d& out_trans) {
  using autodiff::at;
  using autodiff::gradient;
  using autodiff::Matrix3real;
  using autodiff::wrt;

  constexpr double learningRate = 0.1;

  Vector3real trans = Vector3real::Zero();
  Vector3real rot = Vector3real::Zero();
  Vector3real center = center_of_mass.cast<real>();

  std::vector<Vector3real> positions;
  std::vector<Vector3real> normals;
  for (const auto& cp : contactPoints) {
    positions.push_back(cp.position);
    normals.push_back(cp.normal.normalized());
  }

  double max_cos = cos(max_angle);

  real loss;
  for (int i = 0; i < 10000; ++i) {
    Eigen::VectorXd grad =
        gradient(LossFn2,
                 wrt(trans, rot, center),
                 at(positions, normals, trans, rot, center, away_dist),
                 loss);
    grad *= learningRate;

    if (loss < 1e-12) {
      std::cout << i << " loss: " << loss << std::endl;

      Eigen::Vector3d rotd = rot.cast<double>();
      Eigen::Vector3d transd = trans.cast<double>();
      Eigen::Vector3d centerd = center.cast<double>();

      double maxv = 0;
      for (size_t j = 0; j < positions.size(); j++) {
        Eigen::Vector3d v =
            transd + rotd.cross(contactPoints[j].position - centerd);
        maxv = std::max(v.norm(), maxv);
      }

      std::cout << maxv << std::endl;

      double factor = away_dist / maxv;

      transd *= factor;
      rotd *= factor;

      double theta = rotd.norm();

      out_trans = Eigen::Translation3d(transd + centerd) *
                  Eigen::AngleAxisd(theta, rotd / theta) *
                  Eigen::Translation3d(-centerd);
      return true;
    }

    trans -= grad.block(0, 0, 3, 1);
    rot -= grad.block(3, 0, 3, 1);
    center -= grad.block(6, 0, 3, 1);
  }
  std::cout << "failed: " << loss << std::endl;
  return false;
}

}  // namespace core
}  // namespace psg