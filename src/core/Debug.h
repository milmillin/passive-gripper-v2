#pragma once

#include "PassiveGripper.h"
#include "Debugger.h"

namespace psg {
namespace core {

void DebugSubdivision(const PassiveGripper& psg);

void DebugGradient(const PassiveGripper& psg);

void DebugCost(const PassiveGripper& psg, Debugger& debugger);

}
}  // namespace psg