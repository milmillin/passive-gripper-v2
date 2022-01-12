#include "Debug.h"

#include <iomanip>
#include <iostream>

#include "CostFunctions.h"
#include "Optimizer.h"

namespace psg {
namespace core {

void DebugSubdivision(const PassiveGripper& psg) {
  constexpr double base = 1.2;
  GripperSettings settings = psg.GetSettings();
  GripperParams dCost_dParam;  // unused
  for (size_t i = 10; i <= 38; i++) {
    size_t step = round(pow(base, i));
    settings.cost.n_finger_steps = step;
    settings.cost.n_trajectory_steps = step;
    double cost = ComputeCost(psg.GetParams(),
                              psg.GetParams(),
                              settings,
                              psg.GetMDR(),
                              dCost_dParam,
                              nullptr);
    std::cout << std::setprecision(5) << std::scientific << step << " , "
              << cost << std::endl;
  }
}

void DebugCost(const PassiveGripper& psg, Debugger& debugger) {
  size_t n = MyFlattenSize(psg.GetParams());
  GripperParams grad_;
  Eigen::MatrixXd grad(n, 1);
  double cost = ComputeCost1(psg.GetParams(),
                             psg.GetParams(),
                             psg.GetSettings(),
                             psg.GetMDR(),
                             grad_,
                             &debugger);
  MyFlattenGrad(grad_, grad.data());
  std::cout << "Cost: " << cost << std::endl;
  std::cout << "Computed Grad:\n" << grad.transpose() << std::endl;
}

void DebugGradient(const PassiveGripper& psg) {
  size_t n = MyFlattenSize(psg.GetParams());

  Eigen::MatrixXd org(n, 1);
  Eigen::MatrixXd grad(n, 1);
  MyFlattenGrad(psg.GetParams(), org.data());

  GripperParams grad_;
  double org_cost = ComputeCost1(psg.GetParams(),
                                 psg.GetParams(),
                                 psg.GetSettings(),
                                 psg.GetMDR(),
                                 grad_,
                                 nullptr);
  MyFlattenGrad(grad_, grad.data());

  double eps = 1e-9;
  GripperParams unused_grad_;

  Eigen::MatrixXd correct_grad(n, 1);
  for (int i = 0; i < n; i++) {
    Eigen::MatrixXd cur = org;
    cur(i) += eps;
    GripperParams cur_ = psg.GetParams();
    MyUnflatten(cur_, cur.data());
    double cur_cost = ComputeCost1(
        cur_, psg.GetParams(), psg.GetSettings(), psg.GetMDR(), unused_grad_, nullptr);
    correct_grad(i) = (cur_cost - org_cost) / eps;
  }

  std::cout << "Computed Grad:\n" << grad.transpose() << std::endl;
  std::cout << "Correct Grad:\n" << correct_grad.transpose() << std::endl;
  std::cout << "Error:\n" << (grad - correct_grad).transpose() << std::endl;
}

}  // namespace core
}  // namespace psg