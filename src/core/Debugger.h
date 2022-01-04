#pragma once

#include <Eigen/Core>
#include <mutex>
#include <vector>

namespace psg {
namespace core {

class Debugger {
 public:
  Debugger();
  void AddEdge(const Eigen::Vector3d& p0,
               const Eigen::Vector3d& p1,
               const Eigen::Vector3d& color);
  void Clear();

  void Get(Eigen::MatrixXd& out_P,
           Eigen::MatrixXi& out_E,
           Eigen::MatrixXd& out_C) const;

 private:
  Eigen::MatrixXd P_;
  size_t n_P_;
  Eigen::MatrixXi E_;
  size_t n_E_;
  Eigen::MatrixXd C_;
  size_t n_C_;
  mutable std::mutex mtx_;
};

}  // namespace core
}  // namespace psg
