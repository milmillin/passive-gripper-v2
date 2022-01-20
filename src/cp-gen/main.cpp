#include <omp.h>
#include <chrono>
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
  size_t n_candidates = 3000;
  psg::core::models::ContactPointFilter cp_filter_1;

  auto start_time = std::chrono::high_resolution_clock::now();
  auto cps = psg::core::InitializeContactPoints(
      psg, cp_filter_1, n_candidates, n_seeds);
  auto stop_time = std::chrono::high_resolution_clock::now();
  long long duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                           stop_time - start_time)
                           .count();

  Log() << cps.size() << " candidates generated" << std::endl;
  Log() << "Contact Point Generation took " << duration << " ms." << std::endl;

  std::string cp_fn = argv[2];
  std::ofstream cp_f(cp_fn, std::ios::out | std::ios::binary);
  if (!cp_f.is_open()) {
    Error() << "Cannot open " << cp_fn << std::endl;
    return 1;
  }
  psg::core::serialization::Serialize(cps, cp_f);
  Log() << "Contact point candidate written to: " << cp_fn << std::endl;

  Out() << psg_fn << "," << duration << "," << cps.size() << std::endl;
  return 0;
}