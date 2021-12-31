#include "GeometryUtils.h"

#include <unordered_map>
#include <unordered_set>

#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polyhedron_3.h>
#include <igl/volume.h>
#include <igl/copyleft/cgal/mesh_to_polyhedron.h>
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <igl/copyleft/cgal/polyhedron_to_mesh.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullPoint.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <libqhullcpp/RboxPoints.h>

#include "UnionFind.h"

namespace psg {
namespace core {

bool Remesh(const Eigen::MatrixXd& V,
            const Eigen::MatrixXi& F,
            Eigen::MatrixXd& out_V,
            Eigen::MatrixXi& out_F) {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Polyhedron_3<K> Mesh;
  // typedef Mesh::Vertex_index vertex_descriptor;
  namespace PMP = CGAL::Polygon_mesh_processing;

  Mesh mesh;
  igl::copyleft::cgal::mesh_to_polyhedron(V, F, mesh);
  PMP::isotropic_remeshing(
      faces(mesh), 0.003, mesh, PMP::parameters::number_of_iterations(1));
  igl::copyleft::cgal::polyhedron_to_mesh(mesh, out_V, out_F);
}

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
    contactCones[j].position = cp.position;
    contactCones[j].normal = cp.normal + B * cos(curStep) + T * sin(curStep);
    contactCones[j].fid = cp.fid;
  }

  return contactCones;
}

std::vector<ContactPoint> GenerateContactCones(
    const std::vector<ContactPoint>& cps,
    size_t cone_res,
    double friction) {
  std::vector<ContactPoint> cones;
  for (const auto& cp : cps) {
    std::vector<ContactPoint>&& cone =
        GenerateContactCone(cp, cone_res, friction);
    cones.insert(cones.end(), cone.begin(), cone.end());
  }
  return cones;
}

// From https://github.com/libigl/libigl/issues/694
double Volume(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
  Eigen::MatrixXd V2(V.rows() + 1, V.cols());
  V2.topRows(V.rows()) = V;
  V2.bottomRows(1).setZero();
  Eigen::MatrixXi T(F.rows(), 4);
  T.leftCols(3) = F;
  T.rightCols(1).setConstant(V.rows());
  Eigen::VectorXd vol;
  igl::volume(V2, T, vol);
  return std::abs(vol.sum());
}

Eigen::MatrixXd CreateCubeV(const Eigen::Vector3d& lb,
                            const Eigen::Vector3d& ub) {
  auto R = cube_V.array().rowwise() * (ub - lb).transpose().array();
  return R.array().rowwise() + lb.transpose().array();
}

void CreateSpheres(const Eigen::MatrixXd& P,
                   double r,
                   int res,
                   Eigen::MatrixXd& out_V,
                   Eigen::MatrixXi& out_F) {
  out_V.resize(res * res * P.rows(), 3);
  out_F.resize(2 * (res - 1) * res * P.rows(), 3);

  for (long long i = 0; i < P.rows(); i++) {
    Eigen::RowVector3d center = P.row(i);

    // creating vertices
    for (int j = 0; j < res; j++) {
      double z = center(2) + r * cos(kPi * (double)j / (double(res - 1)));
      for (int k = 0; k < res; k++) {
        double x = center(0) + r * sin(kPi * (double)j / (double(res - 1))) *
                                   cos(2 * kPi * (double)k / (double(res - 1)));
        double y = center(1) + r * sin(kPi * (double)j / (double(res - 1))) *
                                   sin(2 * kPi * (double)k / (double(res - 1)));
        out_V.row((res * res) * i + j * res + k) << x, y, z;
      }
    }

    // creating faces
    for (int j = 0; j < res - 1; j++) {
      for (int k = 0; k < res; k++) {
        int v1 = (res * res) * i + j * res + k;
        int v2 = (res * res) * i + (j + 1) * res + k;
        int v3 = (res * res) * i + (j + 1) * res + (k + 1) % res;
        int v4 = (res * res) * i + j * res + (k + 1) % res;
        out_F.row(2 * (((res - 1) * res) * i + res * j + k)) << v1, v2, v3;
        out_F.row(2 * (((res - 1) * res) * i + res * j + k) + 1) << v4, v1, v3;
      }
    }
  }
}

void CreateCylinderXY(const Eigen::Vector3d& o,
                      double r,
                      double h,
                      int res,
                      Eigen::MatrixXd& out_V,
                      Eigen::MatrixXi& out_F) {
  out_V.resize(res * 2, 3);
  out_F.resize(res * 2 + (res - 2) * 2, 3);

  for (int i = 0; i < res; i++) {
    double ang = (2. * kPi * i) / res;
    out_V.row(i) << cos(ang) * r, sin(ang) * r, 0;
  }
  out_V.block(res, 0, res, 3) = out_V.block(0, 0, res, 3).array().rowwise() +
                                Eigen::Array3d(0, 0, h).transpose();
  out_V = out_V.array().rowwise() + o.transpose().array();

  for (int i = 0; i < res; i++) {
    out_F.row(i * 2) << i, ((i + 1) % res) + res, i + res;
    out_F.row(i * 2 + 1) << i, (i + 1) % res, ((i + 1) % res) + res;
  }
  for (int i = 2; i < res; i++) {
    out_F.row(2 * res + i - 2) << 0, i, i - 1;
    out_F.row(2 * res + res - 4 + i) << res, res + i - 1, res + i;
  }
}

void MergeMesh(const Eigen::MatrixXd &V,
               const Eigen::MatrixXi &F,
               Eigen::MatrixXd &out_V,
               Eigen::MatrixXi &out_F) {
  struct Submesh {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    size_t face_count = 0;
    size_t next_face = 0;
    std::unordered_map<size_t, size_t> vertex_map;
  };
  std::unordered_map<size_t, Submesh> submeshes;
  UnionFind unionFind(V.rows());

  // Find sub-mesh that are linked by faces
  for (Eigen::Index f = 0; f < F.rows(); f++) {
    unionFind.merge(F(f, 0), F(f, 1));
    unionFind.merge(F(f, 1), F(f, 2));
  }

  // Assign vertices and faces for sub-mesh
  for (Eigen::Index f = 0; f < F.rows(); f++) {
    submeshes[unionFind.find(F(f, 0))].face_count++;
  }
  for (Eigen::Index v = 0; v < V.rows(); v++) {
    auto &vertex_map = submeshes[unionFind.find(v)].vertex_map;
    size_t next_id = vertex_map.size();
    vertex_map[v] = next_id;
  }

  for (auto &item : submeshes) {
    auto &submesh = item.second;
    submesh.V.resize(submesh.vertex_map.size(), 3);
    submesh.F.resize(submesh.face_count, 3);
  }

  // Map vertices and faces information to sub-mesh
  for (Eigen::Index f = 0; f < F.rows(); f++) {
    auto &submesh = submeshes[unionFind.find(F(f, 0))];
    size_t sub_f = submesh.next_face++;
    for (int i = 0; i < 3; i++) {
      submesh.F(sub_f, i) = submesh.vertex_map[F(f, i)];
    }
  }
  for (Eigen::Index v = 0; v < V.rows(); v++) {
    auto &submesh = submeshes[unionFind.find(v)];
    submesh.V.row(submesh.vertex_map[v]) = V.row(v);
  }

  // Merge all sub-mesh
  auto it = submeshes.begin();
  Eigen::MatrixXd VCurrent = it->second.V, VResult;
  Eigen::MatrixXi FCurrent = it->second.F, FResult;
  for (it++; it != submeshes.end(); it++) {
    auto &submesh = it->second;
    igl::copyleft::cgal::mesh_boolean(submesh.V, submesh.F, VCurrent, FCurrent,
                                      igl::MESH_BOOLEAN_TYPE_UNION,
                                      VResult, FResult);
    VCurrent.swap(VResult);
    FCurrent.swap(FResult);
  }
  out_V.swap(VCurrent);
  out_F.swap(FCurrent);

  int count = 0;
  for (auto &item : submeshes) {
    auto &submesh = item.second;
    std::cout << "submesh " << count++ << std::endl;
    std::cout << "  Faces: " << submesh.next_face << std::endl;
    std::cout << "  Vertices: " << submesh.V.rows() << std::endl;
  }
}

}  // namespace core
}  // namespace psg
