#include "GeometryUtils.h"

#include <unordered_map>
#include <unordered_set>

#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polyhedron_3.h>
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <igl/copyleft/cgal/mesh_to_polyhedron.h>
#include <igl/copyleft/cgal/polyhedron_to_mesh.h>
#include <igl/volume.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullPoint.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <libqhullcpp/RboxPoints.h>
#include <list>

#include "robots/Robots.h"
#include "../easy_profiler_headers.h"

#include "UnionFind.h"

namespace psg {
namespace core {

Fingers TransformFingers(const Fingers& fingers, const Eigen::Affine3d& trans) {
  Fingers out_fingers(fingers.size());
  for (size_t i = 0; i < fingers.size(); i++) {
    out_fingers[i] =
        (trans * fingers[i].transpose().colwise().homogeneous()).transpose();
  }
  return out_fingers;
}

struct _AdtTrajData {
  Fingers fingers;
  Pose pose;
  size_t traj_idx;
  double t;
};

static void AdaptiveSubdivideTrajectoryImpl(
    const Fingers& fingers,
    double flatness2,
    std::list<_AdtTrajData>& l,
    std::list<_AdtTrajData>::iterator l_p0,
    std::list<_AdtTrajData>::iterator l_p2) {
  Pose p1 = (l_p0->pose + l_p2->pose) / 2.;
  Fingers fingers_p1 = TransformFingers(fingers, robots::Forward(p1));

  double t2 = l_p2->t;
  if (l_p2->traj_idx != l_p0->traj_idx) t2 = 1;

  double max_deviation = 0;
  for (size_t j = 0; j < fingers_p1.size(); j++) {
    Eigen::MatrixXd fingers_p02 =
        (l_p2->fingers[j] - l_p0->fingers[j]).rowwise().normalized();
    Eigen::MatrixXd fingers_p01 = (fingers_p1[j] - l_p0->fingers[j]);
    for (size_t i = 0; i < fingers_p02.rows(); i++) {
      Eigen::RowVector3d p02 = fingers_p02.row(i);
      Eigen::RowVector3d p01 = fingers_p01.row(i);
      max_deviation = std::max(max_deviation, p01.cross(p02).squaredNorm());
    }
  }
  if (max_deviation > flatness2) {
    auto l_p1 = l.insert(
        l_p2,
        _AdtTrajData{fingers_p1, p1, l_p0->traj_idx, (l_p0->t + t2) / 2.});
    AdaptiveSubdivideTrajectoryImpl(fingers, flatness2, l, l_p0, l_p1);
    AdaptiveSubdivideTrajectoryImpl(fingers, flatness2, l, l_p1, l_p2);
  }
}

void AdaptiveSubdivideTrajectory(
    const Trajectory& trajectory,
    const Fingers& fingers,
    double flatness,
    Trajectory& out_trajectory,
    std::vector<Fingers>& out_t_fingers,
    std::vector<std::pair<int, double>>& out_traj_contrib) {
  EASY_FUNCTION();

  Eigen::Affine3d finger_trans_inv =
      robots::Forward(trajectory.front()).inverse();

  Fingers fingers0 = TransformFingers(fingers, finger_trans_inv);

  std::list<_AdtTrajData> l;

  size_t n_keyframes = trajectory.size();
  for (size_t i = 0; i < n_keyframes; i++) {
    l.push_back(
        _AdtTrajData{TransformFingers(fingers0, robots::Forward(trajectory[i])),
                     trajectory[i],
                     i,
                     0});
  }

  double flatness2 = flatness * flatness;

  std::list<_AdtTrajData>::iterator l_p0 = l.begin();
  std::list<_AdtTrajData>::iterator l_p2 = ++l.begin();
  for (size_t i = 1; i < n_keyframes; i++) {
    AdaptiveSubdivideTrajectoryImpl(fingers0, flatness2, l, l_p0, l_p2);
    l_p0 = l_p2;
    l_p2++;
  }
  size_t n_new_keyframes = l.size();
  out_trajectory.resize(n_new_keyframes);
  out_t_fingers.resize(n_new_keyframes);
  out_traj_contrib.resize(n_new_keyframes);
  std::list<_AdtTrajData>::iterator l_it = l.begin();
  for (size_t i = 0; i < n_new_keyframes; i++) {
    out_trajectory[i] = l_it->pose;
    out_t_fingers[i] = l_it->fingers;
    out_traj_contrib[i] = {l_it->traj_idx, l_it->t};
    l_it++;
  }
}

bool Remesh(const Eigen::MatrixXd& V,
            const Eigen::MatrixXi& F,
            size_t n_iters,
            Eigen::MatrixXd& out_V,
            Eigen::MatrixXi& out_F) {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Polyhedron_3<K> Mesh;
  // typedef Mesh::Vertex_index vertex_descriptor;
  namespace PMP = CGAL::Polygon_mesh_processing;

  Mesh mesh;
  if (!igl::copyleft::cgal::mesh_to_polyhedron(V, F, mesh)) {
    out_V = V;
    out_F = F;
    return false;
  }
  Eigen::RowVector3d p_min = V.colwise().minCoeff();
  Eigen::RowVector3d p_max = V.colwise().maxCoeff();
  Eigen::RowVector3d p_range = p_max - p_min;

  double r_max = p_range.maxCoeff();
  double edge_len = std::clamp(r_max * 0.02, 0.0005, 0.003);

  std::cout << "edge_len: " << edge_len << std::endl;

  PMP::isotropic_remeshing(faces(mesh),
                           edge_len,
                           mesh,
                           PMP::parameters::number_of_iterations(n_iters));
  igl::copyleft::cgal::polyhedron_to_mesh(mesh, out_V, out_F);
  return true;
}

void Barycentric(const Eigen::Vector3d& p,
                 const Eigen::Vector3d& a,
                 const Eigen::Vector3d& b,
                 const Eigen::Vector3d& c,
                 double& out_u,
                 double& out_v,
                 double& out_w) {
  // https://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
  Eigen::Vector3d v0 = b - a, v1 = c - a, v2 = p - a;
  float d00 = v0.dot(v0);
  float d01 = v0.dot(v1);
  float d11 = v1.dot(v1);
  float d20 = v2.dot(v0);
  float d21 = v2.dot(v1);
  float denom = d00 * d11 - d01 * d01;
  out_v = (d11 * d20 - d01 * d21) / denom;
  out_w = (d00 * d21 - d01 * d20) / denom;
  out_u = 1. - out_v - out_w;
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

Eigen::Vector3d CenterOfMass(const Eigen::MatrixXd& V,
                             const Eigen::MatrixXi& F) {
  // volume-weighted average of COM of tets
  double volume = 0;
  Eigen::RowVector3d center(0, 0, 0);
  for (size_t i = 0; i < F.rows(); i++) {
    Eigen::RowVector3d a = V.row(F(i, 0));
    Eigen::RowVector3d b = V.row(F(i, 1));
    Eigen::RowVector3d c = V.row(F(i, 2));
    double tet_volume = -a.dot(b.cross(c)) / 6.;
    Eigen::RowVector3d tet_center = (a + b + c) / 4.;
    center += tet_volume * tet_center;
    volume += tet_volume;
  }
  return center.transpose() / volume;
}

Eigen::MatrixXd CreateCubeV(const Eigen::Vector3d& lb,
                            const Eigen::Vector3d& ub) {
  auto R = cube_V.array().rowwise() * (ub - lb).transpose().array();
  return R.array().rowwise() + lb.transpose().array();
}

void ComputeConnectivityFrom(const MeshDependentResource& mdr,
                             const Eigen::Vector3d& from,
                             std::vector<double>& out_dist,
                             std::vector<int>& out_par) {
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

  Eigen::RowVector3f effector_pos_f = from.transpose().cast<float>();
  for (size_t i = 0; i < mdr.V.rows(); i++) {
    Eigen::RowVector3d direction = mdr.V.row(i) - from.transpose();
    igl::Hit hit;
    direction -= direction.normalized() * 1e-7;
    if (!mdr.intersector.intersectSegment(
            effector_pos_f, direction.cast<float>(), hit)) {
      dist[i] = (mdr.V.row(i) - from.transpose()).norm();
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
  out_dist = dist;
  out_par = par;
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

void CreateCylinder(const Eigen::Vector3d& a,
                    const Eigen::Vector3d& b,
                    double r,
                    int res,
                    Eigen::MatrixXd& out_V,
                    Eigen::MatrixXi& out_F) {
  out_V.resize(res * 2, 3);
  out_F.resize(res * 2 + (res - 2) * 2, 3);

  Eigen::Vector3d B;
  Eigen::Vector3d T;
  GetPerp((b - a).normalized(), B, T);

  B *= r;
  T *= r;

  for (int i = 0; i < res; i++) {
    double ang = (2. * kPi * i) / res;
    out_V.row(i) = a + cos(ang) * B + sin(ang) * T;
    out_V.row(res + i) = b + cos(ang) * B + sin(ang) * T;
  }

  for (int i = 0; i < res; i++) {
    out_F.row(i * 2) << i, ((i + 1) % res) + res, i + res;
    out_F.row(i * 2 + 1) << i, (i + 1) % res, ((i + 1) % res) + res;
  }
  for (int i = 2; i < res; i++) {
    out_F.row(2 * res + i - 2) << 0, i, i - 1;
    out_F.row(2 * res + res - 4 + i) << res, res + i - 1, res + i;
  }
}

void CreateCone(const Eigen::Vector3d& O,
                const Eigen::Vector3d& N,
                double r,
                double h,
                int res,
                Eigen::MatrixXd& out_V,
                Eigen::MatrixXi& out_F) {
  out_V.resize(res + 1, 3);
  out_F.resize(res + res - 2, 3);
  out_V.row(0) = O;

  Eigen::Vector3d B;
  Eigen::Vector3d T;
  GetPerp(N, B, T);

  for (int i = 0; i < res; i++) {
    double ang = (2. * kPi * i) / res;
    out_V.row(i + 1) = O + h * N + r * (cos(ang) * B + sin(ang) * T);
  }

  for (int i = 0; i < res; i++) {
    out_F.row(i) << 0, i + 1, (i + 1) % res + 1;
  }
  for (int i = 2; i < res; i++) {
    out_F.row(res + i - 2) << 1, i, i + 1;
  }
}

void MergeMesh(const Eigen::MatrixXd& V,
               const Eigen::MatrixXi& F,
               Eigen::MatrixXd& out_V,
               Eigen::MatrixXi& out_F) {
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
    auto& vertex_map = submeshes[unionFind.find(v)].vertex_map;
    size_t next_id = vertex_map.size();
    vertex_map[v] = next_id;
  }

  for (auto& item : submeshes) {
    auto& submesh = item.second;
    submesh.V.resize(submesh.vertex_map.size(), 3);
    submesh.F.resize(submesh.face_count, 3);
  }

  // Map vertices and faces information to sub-mesh
  for (Eigen::Index f = 0; f < F.rows(); f++) {
    auto& submesh = submeshes[unionFind.find(F(f, 0))];
    size_t sub_f = submesh.next_face++;
    for (int i = 0; i < 3; i++) {
      submesh.F(sub_f, i) = submesh.vertex_map[F(f, i)];
    }
  }
  for (Eigen::Index v = 0; v < V.rows(); v++) {
    auto& submesh = submeshes[unionFind.find(v)];
    submesh.V.row(submesh.vertex_map[v]) = V.row(v);
  }

  // Merge all sub-mesh
  auto it = submeshes.begin();
  Eigen::MatrixXd VCurrent = it->second.V, VResult;
  Eigen::MatrixXi FCurrent = it->second.F, FResult;
  for (it++; it != submeshes.end(); it++) {
    auto& submesh = it->second;
    igl::copyleft::cgal::mesh_boolean(submesh.V,
                                      submesh.F,
                                      VCurrent,
                                      FCurrent,
                                      igl::MESH_BOOLEAN_TYPE_UNION,
                                      VResult,
                                      FResult);
    VCurrent.swap(VResult);
    FCurrent.swap(FResult);
  }
  out_V.swap(VCurrent);
  out_F.swap(FCurrent);

  int count = 0;
  for (auto& item : submeshes) {
    auto& submesh = item.second;
    std::cout << "submesh " << count++ << std::endl;
    std::cout << "  Faces: " << submesh.next_face << std::endl;
    std::cout << "  Vertices: " << submesh.V.rows() << std::endl;
  }
}

}  // namespace core
}  // namespace psg
