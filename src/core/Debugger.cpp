#include "Debugger.h"

namespace psg {
namespace core {

Debugger::Debugger() {
  Clear();
}

void Debugger::AddEdge(const Eigen::Vector3d& p0,
                       const Eigen::Vector3d& p1,
                       const Eigen::Vector3d& color) {
  std::lock_guard<std::mutex> guard(mtx_);
  if (n_P_ + 2 > P_.rows()) {
    P_.conservativeResize(P_.rows() + 10000, 3);
  }
  if (n_E_ + 1 > E_.rows()) {
    E_.conservativeResize(E_.rows() + 5000, 2);
  }
  if (n_C_ + 1 > E_.rows()) {
    C_.conservativeResize(C_.rows() + 5000, 3);
  }
  P_.row(n_P_) = p0;
  P_.row(n_P_ + 1) = p1;
  E_.row(n_E_) << n_P_, n_P_ + 1;
  C_.row(n_C_) = color;

  n_P_ += 2;
  n_E_++;
  n_C_++;
}

void Debugger::Clear() {
  P_.resize(1000, 3);
  E_.resize(500, 2);
  C_.resize(500, 3);
  n_P_ = 0;
  n_E_ = 0;
  n_C_ = 0;
}

void Debugger::Get(Eigen::MatrixXd& out_P,
                   Eigen::MatrixXi& out_E,
                   Eigen::MatrixXd& out_C) const {
  out_P = P_.block(0, 0, n_P_, 3);
  out_E = E_.block(0, 0, n_E_, 2);
  out_C = C_.block(0, 0, n_C_, 3);
}

}  // namespace core
}  // namespace psg