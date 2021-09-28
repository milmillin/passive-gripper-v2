#include "GeometryUtils.h"

#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullPoint.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <libqhullcpp/RboxPoints.h>

namespace psg {

bool ComputeConvexHull(const Eigen::MatrixXd& points,
                       std::vector<size_t>& out_hullIndices,
                       std::vector<std::vector<size_t>>& out_facets) {
  using namespace orgQhull;
  RboxPoints rbox;
  size_t dim = points.cols();
  rbox.setDimension(dim);
  for (size_t i = 0; i < points.rows(); i++) {
    for (size_t j = 0; j < dim; j++) {
      rbox.append(points(i, j));
    }
  }
  Qhull qHull;
  qHull.qh()->ferr = stdout;
  try {
    qHull.runQhull(rbox, "o");
  } catch (...) {
    std::cout << "Error while computing convex hull" << std::endl;
    return false;
  }

  // Adapted from qhull/src/user_eg3_r.cpp
  out_hullIndices.clear();
  for (const QhullVertex& vertex : qHull.vertexList()) {
    out_hullIndices.push_back(vertex.point().id());
  }

  out_facets.clear();
  for (const QhullFacet& facet : qHull.facetList()) {
    if (!facet.isGood()) continue;
    std::vector<size_t> vertices;
    if (!facet.isTopOrient() && facet.isSimplicial()) {
      QhullVertexSet vs = facet.vertices();
      vertices.push_back(vs[1].point().id());
      vertices.push_back(vs[0].point().id());
      for (size_t i = 2; i < vs.size(); i++) {
        vertices.push_back(vs[i].point().id());
      }
    } else {
      for (const QhullVertex& vertex : facet.vertices()) {
        vertices.push_back(vertex.point().id());
      }
    }
    out_facets.push_back(vertices);
  }
  return true;
}

double AngularDistance(double a, double b) {
  double diff = fmod(b - a, kTwoPi);
  double other = (diff > 0) ? diff - kTwoPi : diff + kTwoPi;
  return abs(diff) < abs(other) ? diff : other;
}

double SumSquaredAngularDistance(const Pose& a, const Pose& b) {
  size_t n = a.size();
  double sum = 0;
  for (size_t i = 0; i < n; i++) {
    double tmp = AngularDistance(a[i], b[i]);
    sum += tmp * tmp;
  }
  return sum;
}

Pose FixAngles(const Pose& a, const Pose& b) {
  size_t n = a.size();
  Pose c;
  for (size_t i = 0; i < n; i++) {
    c[i] = a[i] + AngularDistance(a[i], b[i]);
  }
  return c;
}

void FixTrajectory(Trajectory& t) {
  for (size_t i = 1; i < t.size(); i++) {
    t[i] = FixAngles(t[i - 1], t[i]);
  }
}

void GetPerp(const Eigen::Vector3d& N, Eigen::Vector3d& B, Eigen::Vector3d& T) {
  B = N.cross(Eigen::Vector3d::UnitX());
  if (B.squaredNorm() < 1e-12) B = N.cross(Eigen::Vector3d::UnitY());
  B.normalize();
  T = B.cross(N);
}

double DoubleTriangleArea(const Eigen::Vector3d& A,
                          const Eigen::Vector3d& B,
                          const Eigen::Vector3d& C) {
  return (B - A).cross(C - A).norm();
}

std::vector<ContactPoint> GenerateContactCone(const ContactPoint& contactPoint,
                                              size_t coneRes,
                                              double friction) {
  std::vector<ContactPoint> contactCones;
  contactCones.resize(coneRes);
  Eigen::Vector3d B;
  Eigen::Vector3d T;
  double stepSize = EIGEN_PI * 2 / coneRes;
  double curStep;
  const auto& cp = contactPoint;
  GetPerp(cp.normal, B, T);
  double coeff = std::max(-cp.normal.dot(Eigen::Vector3d::UnitY()), 1e-3);
  B *= friction * coeff;
  T *= friction * coeff;
  for (size_t j = 0; j < coneRes; j++) {
    curStep = j * stepSize;
    contactCones[j] = ContactPoint{
        cp.position, cp.normal + B * cos(curStep) + T * sin(curStep), cp.fid};
  }

  return contactCones;
}
}  // namespace psg