#include <omp.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "../Constants.h"
#include "../core/Initialization.h"
#include "../core/PassiveGripper.h"
#include "../core/robots/Robots.h"
#include "../core/serialization/Serialization.h"
#include "../utils.h"

int main(int argc, char** argv) {
  Log() << "Num threads: " << omp_get_max_threads() << std::endl;
  if (argc < 3) {
    Error() << "input .psg file and output .cpx file required" << std::endl;
    return 1;
  }
  std::string psg_fn = argv[1];
  std::ifstream psg_f(psg_fn, std::ios::in | std::ios::binary);
  if (!psg_f.is_open()) {
    Error() << "Cannot open " << psg_fn << std::endl;
    return 1;
  }

  psg::core::PassiveGripper psg;
  psg.Deserialize(psg_f);

  size_t n_seeds = 1000;
  size_t n_candidates = 5000;
  psg::core::models::ContactPointFilter cp_filter_1;
  psg::core::models::ContactPointFilter cp_filter_2;
  cp_filter_2.angle = psg::kPi / 2 - atan(psg.GetContactSettings().friction);

  Log() << "Phase 1" << std::endl;
  auto cps1 = psg::core::InitializeContactPoints(
      psg, cp_filter_1, n_candidates, n_seeds);
  Log() << "Phase 2" << std::endl;
  auto cps2 = psg::core::InitializeContactPoints(
      psg, cp_filter_2, n_candidates, n_seeds);

  std::vector<psg::core::models::ContactPointMetric> cps;
  cps.reserve(cps1.size() + cps2.size());

  // interleave
  size_t ii = 0;
  size_t jj = 0;
  while (ii < cps1.size() || jj < cps2.size()) {
    if (ii < cps1.size()) {
      cps.push_back(cps1[ii]);
      ii++;
    }
    if (jj < cps2.size()) {
      cps.push_back(cps2[jj]);
      jj++;
    }
  }

  std::string cp_fn = argv[2];
  std::ofstream cp_f(cp_fn, std::ios::out | std::ios::binary);
  if (!cp_f.is_open()) {
    Error() << "Cannot open " << cp_fn << std::endl;
    return 1;
  }
  psg::core::serialization::Serialize(cps, cp_f);
  Log() << "Contact point candidate written to: " << cp_fn << std::endl;
  return 0;
}