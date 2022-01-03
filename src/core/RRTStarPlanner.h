#pragma once

#include "PassiveGripper.h"

namespace psg {
namespace core {

class RRTStarPlanner {
 public:
  bool Optimize(const PassiveGripper& psg, Trajectory& out_trajectory);
};

}  // namespace core
}  // namespace psg