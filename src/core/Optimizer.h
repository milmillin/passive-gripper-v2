#pragma once

#include <nlopt.h>
#include <Eigen/Core>
#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <vector>

#include "PassiveGripper.h"

namespace psg {
namespace core {

class Optimizer {
 public:
  ~Optimizer();
  void Optimize(const PassiveGripper& psg);
  void Wait();
  void Cancel();
  void Resume();
  void Reset();

  inline bool IsRunning() { return opt_ != nullptr && is_running_; };
  inline bool IsResultAvailable() {
    return opt_ != nullptr && is_result_available_.load();
  }
  inline double GetCurrentCost() { return g_min_cost_; }
  inline std::chrono::time_point<std::chrono::high_resolution_clock>
  GetStartTime() {
    return start_time_;
  }
  const GripperParams& GetCurrentParams();


  // Internal use
  double ComputeCostInternal(unsigned n, const double* x);

 private:
  nlopt_opt opt_ = nullptr;
  int dimension_;

  GripperParams params_;
  GripperParams params_proto_;
  MeshDependentResource mdr_;
  GripperSettings settings_;
  std::unique_ptr<double> x_;
  std::unique_ptr<double> lb_;
  std::unique_ptr<double> ub_;

  std::future<nlopt_result> optimize_future_;
  std::atomic_bool is_running_ = false;
  std::atomic_bool is_resumable_ = false;
  std::atomic_bool is_result_available_ = false;

  long long n_iters_;
  std::atomic<double> g_min_cost_;
  double t_min_cost_;
  std::unique_ptr<double> g_min_x_;
  mutable std::mutex g_min_x_mutex_;

  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};

}  // namespace core
}  // namespace psg