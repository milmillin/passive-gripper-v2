#include "Optimizer.h"

#include "CostFunctions.h"

namespace psg {
namespace core {

size_t MyFlattenSize(const GripperParams& meta) {
  return meta.fingers.size() * (meta.fingers.front().rows() - 2) * 3 +
         (meta.trajectory.size() - 2) * meta.trajectory.front().size();
}

// meta -> x
void MyFlatten(const GripperParams& meta,
               const OptSettings& settings,
               double* x,
               double* lb,
               double* ub) {
  for (size_t i = 0; i < meta.fingers.size(); i++) {
    const auto& finger = meta.fingers[i];
    for (size_t r = 1; r < finger.rows() - 1; r++) {
      for (size_t c = 0; c < 3; c++) {
        *lb = *ub = *(x++) = finger(r, c);
        *lb -= settings.finger_wiggle;
        *ub += settings.finger_wiggle;
        lb++;
        ub++;
      }
    }
  }
  for (size_t i = 1; i < meta.trajectory.size() - 1; i++) {
    const auto& keyframe = meta.trajectory[i];
    for (size_t j = 0; j < keyframe.size(); j++) {
      *lb = *ub = *(x++) = keyframe[j];
      *lb -= settings.trajectory_wiggle[j];
      *ub += settings.trajectory_wiggle[j];
      lb++;
      ub++;
    }
  }
}

// x -> meta
void MyUnflatten(GripperParams& meta, const double* x) {
  for (size_t i = 0; i < meta.fingers.size(); i++) {
    auto& finger = meta.fingers[i];
    for (size_t r = 1; r < finger.rows() - 1; r++) {
      for (size_t c = 0; c < 3; c++) {
        finger(r, c) = *(x++);
      }
    }
  }
  for (size_t i = 1; i < meta.trajectory.size() - 1; i++) {
    auto& keyframe = meta.trajectory[i];
    for (size_t j = 0; j < keyframe.size(); j++) {
      keyframe[j] = *(x++);
    }
  }
}

static double ComputeCostWrapper(unsigned n,
                                 const double* x,
                                 double* grad,
                                 void* data) {
  // ignore grad
  return reinterpret_cast<Optimizer*>(data)->ComputeCostInternal(n, x);
}

Optimizer::~Optimizer() {
  Cancel();
  if (opt_ != nullptr) nlopt_destroy(opt_);
}

void Optimizer::Optimize(const PassiveGripper& psg) {
  Cancel();
  params_ = psg.GetParams();
  params_proto_ = psg.GetParams();
  const auto& mdr = psg.GetMDR();
  mdr_.init(mdr.V, mdr.F);
  settings_ = psg.GetSettings();
  dimension_ = MyFlattenSize(params_);
  x_.reset(new double[dimension_]);
  lb_.reset(new double[dimension_]);
  ub_.reset(new double[dimension_]);
  g_min_x_.reset(new double[dimension_]);
  MyFlatten(params_, settings_.opt, x_.get(), lb_.get(), ub_.get());

  if (opt_ != nullptr) nlopt_destroy(opt_);
  opt_ = nlopt_create(settings_.opt.algorithm, dimension_);
  if (settings_.opt.population > 0) {
    nlopt_set_population(opt_, settings_.opt.population);
  }

  nlopt_set_min_objective(opt_, ComputeCostWrapper, this);
  if (settings_.opt.max_runtime > 0.) {
    nlopt_set_maxtime(opt_, settings_.opt.max_runtime);
  }
  nlopt_set_lower_bounds(opt_, lb_.get());
  nlopt_set_upper_bounds(opt_, ub_.get());
  nlopt_set_stopval(opt_, 1e-15);
  if (settings_.opt.max_runtime == 0.) {
    nlopt_set_ftol_rel(opt_, settings_.opt.tolerance);
    nlopt_set_ftol_abs(opt_, 1e-15);
  }
  if (settings_.opt.max_iters > 0) {
    nlopt_set_maxeval(opt_, settings_.opt.max_iters);
  }

  n_iters_ = 0;
  g_min_cost_ = t_min_cost_ = std::numeric_limits<double>::max();
  is_running_ = true;
  is_resumable_ = true;
  start_time_ = std::chrono::high_resolution_clock::now();
  optimize_future_ = std::async(std::launch::async, [&] {
    double minf; /* minimum objective value, upon return */
    nlopt_result result = nlopt_optimize(opt_, x_.get(), &minf);
    MyUnflatten(params_, x_.get());
    is_running_ = false;
    return result;
  });
}

void Optimizer::Resume() {
  if (opt_ == nullptr) return;
  if (is_running_) return;
  if (!is_resumable_) return;
  is_running_ = true;
  optimize_future_ = std::async(std::launch::async, [&] {
    double minf; /* minimum objective value, upon return */
    nlopt_result result = nlopt_optimize(opt_, x_.get(), &minf);
    MyUnflatten(params_, x_.get());
    is_running_ = false;
    return result;
  });
}

void Optimizer::Reset() {
  Cancel();
  is_running_ = false;
  is_resumable_ = false;
  is_result_available_ = false;
}

void Optimizer::Wait() {
  if (optimize_future_.valid()) {
    optimize_future_.get();
  }
}

void Optimizer::Cancel() {
  if (opt_ != nullptr) {
    nlopt_force_stop(opt_);
  }
  Wait();
}

const GripperParams& Optimizer::GetCurrentParams() {
  std::lock_guard<std::mutex> guard(g_min_x_mutex_);
  MyUnflatten(params_proto_, g_min_x_.get());
  return params_proto_;
}
double Optimizer::ComputeCostInternal(unsigned n, const double* x) {
  MyUnflatten(params_, x);
  GripperParams dCost_dParam; // unused
  double cost = ComputeCost(params_, settings_, mdr_, dCost_dParam);
  n_iters_++;
  if (cost < t_min_cost_) {
    is_result_available_ = true;
    t_min_cost_ = cost;
    g_min_cost_ = cost;
    {
      std::lock_guard<std::mutex> guard(g_min_x_mutex_);
      memcpy(g_min_x_.get(), x, n * sizeof(double));
    }
    std::cerr << "Iter: " << n_iters_ << ", Current Cost : " << cost
              << std::endl;
  }
  return cost;
}
}  // namespace core
}  // namespace psg