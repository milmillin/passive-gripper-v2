#include "Testcase.h"

#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "../core/Optimizer.h"
#include "../core/PassiveGripper.h"
#include "../core/serialization/Serialization.h"
#include "Result.h"
#include "utils.h"

using namespace psg::core::models;

void Testcase::ProcessFrom(size_t j_cp, const TestcaseCallback& cb) const {
  Log() << "Processing " << name << std::endl;

  std::ifstream psg_file(psg_filename, std::ios::in | std::ios::binary);
  if (!psg_file.is_open()) {
    throw std::invalid_argument("Cannot open psg file " + psg_filename);
  }
  Log() << "> Loaded " << psg_filename << std::endl;

  psg::core::PassiveGripper psg;
  psg.Deserialize(psg_file);
  psg::core::Optimizer optimizer;
  const size_t buf_size = cp_filename_fmt.size() + 16;
  char* buf = new char[buf_size];
  for (size_t i = j_cp; i < n_cp_files; i++) {
    snprintf(buf, buf_size, cp_filename_fmt.c_str(), i);
    std::string cp_filename = buf;
    Log() << "> Processing " << cp_filename << std::endl;
    std::ifstream cp_file(cp_filename, std::ios::in | std::ios::binary);
    if (!cp_file.is_open()) {
      Error() << "> Cannot open cp file " << cp_filename << std::endl;
      Error() << ">> Skipping" << std::endl;
      continue;
    }
    std::vector<ContactPoint> contact_points;
    psg::core::serialization::Deserialize(contact_points, cp_file);
    psg.reinit_trajectory = true;
    psg.SetContactPoints(contact_points);
    auto start_time = std::chrono::high_resolution_clock::now();
    optimizer.Optimize(psg);
    optimizer.Wait();
    auto stop_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        stop_time - start_time);
    snprintf(buf, buf_size, (out_filename_fmt + ".psg").c_str(), i);
    std::string out_filename = buf;
    std::ofstream out_file(out_filename, std::ios::out | std::ios::binary);
    if (!out_file.is_open()) {
      Error() << "> Cannot open out file " << out_filename << std::endl;
      Error() << ">> Skipping" << std::endl;
      continue;
    }
    psg.SetParams(optimizer.GetCurrentParams());
    psg.Serialize(out_file);
    Log() << "> Optimization took " << duration.count() << " ms." << std::endl;
    Log() << "> Optimized gripper written to " << out_filename << std::endl;
    Result res{name,
               out_filename,
               psg.GetIsForceClosure(),
               psg.GetIsPartialClosure(),
               psg.GetMinWrench(),
               psg.GetPartialMinWrench(),
               psg.GetCost(),
               psg.GetMinDist(),
               duration.count()};
    Out() << res << std::endl;
    if (cb) cb(i, *this);
  }
  Log() << "Done processing " << name << std::endl;
}