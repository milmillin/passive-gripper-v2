#include "Debug.h"

#include <iomanip>
#include <iostream>

#include "CostFunctions.h"

namespace psg {
namespace core {

void DebugSubdivision(const PassiveGripper& psg) {
  constexpr double base = 1.2;
  GripperSettings settings = psg.GetSettings();
  GripperParams dCost_dParam; // unused
  for (size_t i = 10; i <= 38; i++) {
    size_t step = round(pow(base, i));
    settings.cost.n_finger_steps = step;
    settings.cost.n_trajectory_steps = step;
    double cost = ComputeCost(psg.GetParams(), settings, psg.GetMDR(), dCost_dParam);
    std::cout << std::setprecision(5) << std::scientific << step << " , " << cost
              << std::endl;
  }
}

}  // namespace core
}  // namespace psg