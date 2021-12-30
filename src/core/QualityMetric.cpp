#include "QualityMetric.h"

#include <CGAL/QP_functions.h>
#include <CGAL/QP_models.h>

#include "GeometryUtils.h"

// #ifdef CGAL_USE_GMP
// #include <CGAL/Gmpzf.h>
// typedef CGAL::Gmpzf ET;
// #else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
// #endif

namespace psg {
namespace core {
Eigen::MatrixXd CreateGraspMatrix(const std::vector<ContactPoint>& contactCones,
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

bool CheckForceClosureQP(const Eigen::MatrixXd& G) {
  return MinNormVectorInFacet(G) < kWrenchNormThresh;
}

bool CheckPartialClosureQP(const Eigen::MatrixXd& G,
                           const Eigen::Vector3d& extForce,
                           const Eigen::Vector3d& extTorque) {
  Eigen::VectorXd targetWrench(6);
  targetWrench.block<3, 1>(0, 0) = -extForce;
  targetWrench.block<3, 1>(3, 0) = -extTorque;
  return WrenchInPositiveSpan(G, targetWrench) < kWrenchNormThresh;
}

double ComputeMinWrenchQP(const Eigen::MatrixXd& G) {
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

double ComputePartialMinWrenchQP(const Eigen::MatrixXd& G,
                                 const Eigen::Vector3d& extForce,
                                 const Eigen::Vector3d& extTorque) {
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

void ComputeQualityMetricSingle(const std::vector<ContactPoint>& contact_points,
                                const Eigen::Vector3d& center_of_mass,
                                size_t cone_res,
                                double friction,
                                const Eigen::Vector3d& ext_force,
                                const Eigen::Vector3d& ext_torque,
                                double& out_min_wrench,
                                double& out_partial_min_wrench) {
  std::vector<ContactPoint> contact_cones =
      GenerateContactCones(contact_points, cone_res, friction);
  Eigen::MatrixXd G = CreateGraspMatrix(contact_cones, center_of_mass);
  out_min_wrench = ComputeMinWrenchQP(G);
  out_partial_min_wrench = ComputePartialMinWrenchQP(G, ext_force, ext_torque);
}

bool ComputeRobustQualityMetric(const std::vector<ContactPoint>& contact_points,
                                const MeshDependentResource& mdr,
                                const GripperSettings& settings,
                                const Eigen::Vector3d& ext_force,
                                const Eigen::Vector3d& ext_torque,
                                ContactPointMetric& out_metric) {
  double min_wrench;
  double partial_min_wrench;
  out_metric.contact_points = contact_points;
  ComputeQualityMetricSingle(contact_points,
                             mdr.center_of_mass,
                             settings.contact.cone_res,
                             settings.contact.friction,
                             ext_force,
                             ext_torque,
                             min_wrench,
                             partial_min_wrench);
  if (partial_min_wrench == 0) {
    out_metric.min_wrench = min_wrench;
    out_metric.partial_min_wrench = partial_min_wrench;
    return false;
  }

  constexpr size_t kSamplePerContact = kSubR * kSubTheta;
  constexpr size_t kC50 =
      kSamplePerContact * kSamplePerContact * kSamplePerContact * 50 / 100;
  constexpr double kRadiusStep = kRadius / kSubR;
  constexpr double kThetaStep = kTwoPi / kSubTheta;

  std::vector<std::vector<ContactPoint>> sub_contact_points(
      contact_points.size(), std::vector<ContactPoint>(kSamplePerContact));
  for (size_t i = 0; i < contact_points.size(); i++) {
    const ContactPoint& base_cp = contact_points[i];
    Eigen::Vector3d B;
    Eigen::Vector3d T;
    GetPerp(base_cp.normal, B, T);

    std::cout << "base\n" << base_cp.normal.transpose() << std::endl;
    for (size_t j = 0; j < kSubR; j++) {
      double r = (j + 1) * kRadiusStep;
      for (size_t k = 0; k < kSubTheta; k++) {
        double theta = k * kThetaStep;
        ContactPoint& cp = sub_contact_points[i][j * kSubTheta + k];
        Eigen::Vector3d position =
            base_cp.position + r * cos(theta) * B + r * sin(theta) * T;
        Eigen::Vector3d c;
        size_t fid = mdr.ComputeClosestFacet(position, c);
        cp.position = c;
        cp.fid = fid;
        cp.normal = mdr.FN.row(fid);
        std::cout << cp.normal.transpose() << std::endl;
      }
    }
  }

  std::vector<ContactPointMetric> metrics;
  metrics.reserve(contact_points.size() * kSamplePerContact);
  std::vector<ContactPoint> try_contact_points;
  size_t success = 0;
  for (size_t i = 0; i < contact_points.size(); i++) {
    try_contact_points = contact_points;
    for (size_t j = 0; j < kSamplePerContact; j++) {
      try_contact_points[i] = sub_contact_points[i][j];
      ComputeQualityMetricSingle(try_contact_points,
                                 mdr.center_of_mass,
                                 settings.contact.cone_res,
                                 settings.contact.friction,
                                 ext_force,
                                 ext_torque,
                                 min_wrench,
                                 partial_min_wrench);
      if (partial_min_wrench > 0) success++;
      metrics.emplace_back();
      metrics.back().min_wrench = min_wrench;
      metrics.back().partial_min_wrench = partial_min_wrench;
      std::cout << i << "," << j << " " << std::setprecision(12)
                << "mw: " << min_wrench << ", pmw: " << partial_min_wrench
                << std::endl;
    }
  }
  std::cout << "success: " << success << "/" << metrics.size() << std::endl;
  std::sort(metrics.begin(), metrics.end());
  out_metric = metrics[metrics.size() / 2];
  out_metric.contact_points = contact_points;
  return out_metric.partial_min_wrench > 0;
}

}  // namespace core
}  // namespace psg