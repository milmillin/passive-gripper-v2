#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include "../core/PassiveGripper.h"

namespace psg_batch {

// Generates a PSG file from an STL
void GeneratePSG(const std::string& stl_fn, const std::string& psg_fn);

// Generates a CPX file containing a ranked list of grasp configurations.
void GenerateCPX(const psg::core::PassiveGripper& psg,
                 const std::string& cpx_fn);

constexpr size_t CPX_N_SEEDS = 1000;
constexpr size_t CPX_N_CANDIDATES = 3000;

}  // namespace psg_batch
