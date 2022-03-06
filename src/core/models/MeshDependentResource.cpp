#include "MeshDependentResource.h"

#include <igl/gaussian_curvature.h>
#include <igl/invert_diag.h>
#include <igl/massmatrix.h>
#include <igl/per_edge_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/principal_curvature.h>
#include <igl/signed_distance.h>
#include "../GeometryUtils.h"
#include "../../easy_profiler_headers.h"

namespace psg {
namespace core {
namespace models {
void MeshDependentResource::init(const Eigen::MatrixXd& V_,
                                 const Eigen::MatrixXi& F_) {
  EASY_FUNCTION();

  if (initialized) {
    tree.deinit();
    intersector.deinit();
  }
  V = V_;
  F = F_;
  EASY_STATEMENT("AABB", tree.init(V_, F_));
  EASY_STATEMENT("intersector", intersector.init(V.cast<float>(), F, true));
  EASY_STATEMENT("per_face_normals", igl::per_face_normals(V_, F_, FN));
  EASY_STATEMENT("per_vertex_normals", igl::per_vertex_normals(
      V_, F_, igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_ANGLE, FN, VN));
  EASY_STATEMENT("per_edge_normals", igl::per_edge_normals(
      V_, F_, igl::PER_EDGE_NORMALS_WEIGHTING_TYPE_UNIFORM, FN, EN, E, EMAP));

  minimum = V.colwise().minCoeff();
  maximum = V.colwise().maxCoeff();
  size = maximum - minimum;

  EASY_STATEMENT("CenterOfMass", center_of_mass = CenterOfMass(V, F));

  EASY_STATEMENT("principal_curvature", igl::principal_curvature(V, F, PD1, PD2, PV1, PV2));

  SP_valid_ = false;
  // curvature_valid_ = false;
}

void MeshDependentResource::init(const MeshDependentResource& other) {
  EASY_FUNCTION();

  init(other.V, other.F);
  if (other.SP_valid_) {
    EASY_STATEMENT("copy SP_valid_", SP_valid_ = other.SP_valid_);
    EASY_STATEMENT("copy SP_", SP_ = other.SP_);
    EASY_STATEMENT("copy SP_par_", SP_par_ = other.SP_par_);
  }
  /*
  if (other.curvature_valid_) {
    curvature_valid_ = other.curvature_valid_;
    curvature_ = other.curvature_;
  }
  */
}

void MeshDependentResource::init_sp() const {
  EASY_FUNCTION();

  if (SP_valid_) return;
  std::lock_guard<std::mutex> lock(SP_mutex_);
  if (SP_valid_) return;

  size_t nV = V.rows();
  size_t nF = F.rows();
  std::vector<std::vector<std::pair<int, double>>> edges(nV);
  SP_.resize(nV, nV);
  SP_par_.resize(nV, nV);
  SP_.setConstant(std::numeric_limits<double>::max() / 2.);
  SP_par_.setConstant(-1);
  for (size_t i = 0; i < nF; i++) {
    for (size_t k = 0; k < 3; k++) {
      size_t jj = (k + 1) % 3;
      int u = F(i, k);
      int v = F(i, jj);
      edges[u].push_back({v, (V.row(u) - V.row(v)).norm()});
    }
  }

  struct VertexInfo {
    int id;
    double dist;
    bool operator<(const VertexInfo& r) const { return dist > r.dist; }
  };
#pragma omp parallel for
  for (int src = 0; src < nV; src++) {
    std::priority_queue<VertexInfo> q;
    q.push(VertexInfo{src, 0});
    SP_(src, src) = 0;
    while (!q.empty()) {
      VertexInfo u = q.top();
      q.pop();
      double curDis;
      for (const auto& v : edges[u.id]) {
        if ((curDis = u.dist + v.second) < SP_(v.first, src)) {
          SP_(v.first, src) = curDis;
          SP_par_(v.first, src) = u.id;
          q.push(VertexInfo{v.first, curDis});
        }
      }
    }
  }
  SP_valid_ = true;
}

/*
void MeshDependentResource::init_curvature() const {
  if (curvature_valid_) return;  // reduce lock overhead?
  std::lock_guard<std::mutex> lock(curvature_mutex_);
  if (curvature_valid_) return;

  //
https://github.com/libigl/libigl/blob/main/tutorial/202_GaussianCurvature/main.cpp
  /*
  Eigen::VectorXd K;
  // Compute integral of Gaussian curvature
  igl::gaussian_curvature(V, F, K);
  // Compute mass matrix
  Eigen::SparseMatrix<double> M, Minv;
  igl::massmatrix(V, F, igl::MASSMATRIX_TYPE_DEFAULT, M);
  igl::invert_diag(M, Minv);
  // Divide by area to get integral average
  K = (Minv * K).eval();
  curvature_ = K;


  Eigen::MatrixXd PD1, PD2;
  Eigen::VectorXd PV1, PV2;
  igl::principal_curvature(V, F, PD1, PD2, PV1, PV2);

  curvature_valid_ = true;
}
*/

double MeshDependentResource::ComputeSignedDistance(
    const Eigen::Vector3d& position,
    Eigen::RowVector3d& c,
    double& s) const {
  // EASY_FUNCTION();

  double sqrd;
  int i;
  Eigen::RowVector3d n;
  igl::signed_distance_pseudonormal(
      tree, V, F, FN, VN, EN, EMAP, position.transpose(), s, sqrd, i, c, n);
  return s * sqrt(sqrd);
}
void MeshDependentResource::ComputeClosestPoint(const Eigen::Vector3d& position,
                                                Eigen::RowVector3d& out_c,
                                                int& out_fid) const {
  tree.squared_distance(V, F, position.transpose(), out_fid, out_c);
}
size_t MeshDependentResource::ComputeClosestFacet(
    const Eigen::Vector3d& position) const {
  Eigen::RowVector3d c;
  int fid;
  tree.squared_distance(V, F, position.transpose(), fid, c);
  return fid;
}
size_t MeshDependentResource::ComputeClosestVertex(
    const Eigen::Vector3d& position) const {
  Eigen::RowVector3d c;
  int fid;
  tree.squared_distance(V, F, position.transpose(), fid, c);
  double dist = std::numeric_limits<double>::max();
  size_t vid = -1;
  for (size_t i = 0; i < 3; i++) {
    int cur_vid = F(fid, i);
    double cur_dist = (V.row(cur_vid) - position.transpose()).squaredNorm();
    if (cur_dist < dist) {
      dist = cur_dist;
      vid = cur_vid;
    }
  }
  return vid;
}

double MeshDependentResource::ComputeRequiredDistance(
    const Eigen::Vector3d& A,
    const Eigen::Vector3d& B,
    Debugger* const debugger) const {
  init_sp();
  Eigen::RowVector3d dir = B - A;
  double norm = dir.norm();
  if (norm < 1e-12 || isnan(norm)) return 0;
  dir /= norm;
  // std::cout << dir << std::endl;
  std::vector<igl::Hit> hits;
  int numRays;
  intersector.intersectRay(A.cast<float>(), dir.cast<float>(), hits, numRays);
  bool isIn = hits.size() % 2 == 1;
  size_t lastVid = -1;
  double lastT = 0;
  if (isIn) {
    lastVid = ComputeClosestVertex(A);
  }
  double totalDis = 0;
  for (const igl::Hit& hit : hits) {
    if (hit.t >= norm) break;
    Eigen::RowVector3d P = A.transpose() + dir * hit.t;
    size_t vid = -1;
    double bestDis = std::numeric_limits<double>::max();
    for (size_t i = 0; i < 3; i++) {
      int u = F(hit.id, i);
      double d = (P - V.row(u)).squaredNorm();
      if (d < bestDis) {
        bestDis = d;
        vid = u;
      }
    }
    bestDis = sqrt(bestDis);
    if (isIn) {
      totalDis += SP_(lastVid, vid) + bestDis + hit.t - lastT;
      if (debugger) {
        debugger->AddEdge(A.transpose() + dir * lastT, P, colors::kRed);
        int cur = lastVid;
        while (SP_par_(cur, vid) != -1) {
          debugger->AddEdge(V.row(cur), V.row(SP_par_(cur, vid)), colors::kRed);
          cur = SP_par_(cur, vid);
        }
        debugger->AddEdge(P, V.row(vid), colors::kRed);
        debugger->AddEdge(
            A.transpose() + dir * lastT, V.row(lastVid), colors::kRed);
      }
    }
    lastVid = vid;
    isIn = !isIn;
    lastT = hit.t;
  }
  if (isIn) {
    // B is in
    size_t vid = ComputeClosestVertex(B);
    totalDis +=
        SP_(lastVid, vid) + (V.row(vid) - B.transpose()).norm() + norm - lastT;
    if (debugger) {
      debugger->AddEdge(A.transpose() + dir * lastT, B, colors::kRed);
      int cur = lastVid;
      while (SP_par_(cur, vid) != -1) {
        debugger->AddEdge(V.row(cur), V.row(SP_par_(cur, vid)), colors::kRed);
        cur = SP_par_(cur, vid);
      }
      debugger->AddEdge(B, V.row(vid), colors::kRed);
      debugger->AddEdge(
          A.transpose() + dir * lastT, V.row(lastVid), colors::kRed);
    }
  }
  return totalDis;
}

static bool TreeIntersectsImpl(const igl::AABB<Eigen::MatrixXd, 3>* tree,
                               const Eigen::AlignedBox3d& box) {
  bool intersects = tree->m_box.intersects(box);
  if (!intersects) return false;
  if (tree->is_leaf()) return intersects;
  return TreeIntersectsImpl(tree->m_left, box) ||
         TreeIntersectsImpl(tree->m_right, box);
}

bool MeshDependentResource::Intersects(const Eigen::AlignedBox3d box) const {
  return TreeIntersectsImpl(&tree, box);
}

// Getters
const Eigen::MatrixXd& MeshDependentResource::GetSP() const {
  EASY_FUNCTION();

  init_sp();
  return SP_;
}

const Eigen::MatrixXi& MeshDependentResource::GetSPPar() const {
  init_sp();
  return SP_par_;
}

/*
const Eigen::VectorXd& MeshDependentResource::GetCurvature() const {
  init_curvature();
  return curvature_;
}
*/

}  // namespace models
}  // namespace core
}  // namespace psg