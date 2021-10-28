#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <omp.h>

#include "../core/PassiveGripper.h"
#include "../core/Initialization.h"
#include "../core/serialization/Serialization.h"
#include "../utils.h"

int main(int argc, char** argv) {
  Log() << "Num threads: " << omp_get_max_threads() << std::endl;
  if (argc < 3) {
    Error() << "input .psg file and output .cp file required" << std::endl;
    return 1;
  }
  std::string psg_fn = argv[1];
  std::ifstream psg_f(psg_fn, std::ios::in | std::ios::binary);
  if (!psg_f.is_open()) {
    Error() << "Cannot open " << psg_fn << std::endl;
    return 1;
  }

  std::string cp_fn = argv[2];
  std::ofstream cp_f(cp_fn, std::ios::out | std::ios::binary);
  if (!cp_f.is_open()) {
    Error() << "Cannot open " << cp_fn << std::endl;
    return 1;
  }

  psg::core::PassiveGripper psg;
  psg.Deserialize(psg_f);
  auto cps = psg::core::InitializeContactPoints(
      psg.GetMDR(), psg.GetSettings(), 10000, 1000);
  std::vector<std::vector<psg::core::models::ContactPoint>> cp_list;
  cp_list.reserve(cps.size());
  for (const auto& cp : cps) {
    cp_list.push_back(cp.contact_points);
  }
  psg::core::serialization::Serialize(cp_list, cp_f);
  Log() << "Contact point candidate written to: " << cp_fn << std::endl;
  return 0;
}