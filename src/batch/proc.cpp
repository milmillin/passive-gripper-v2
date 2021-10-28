#include "proc.h"

#include "utils.h"

#include "../core/Optimizer.h"
#include "../core/PassiveGripper.h"
#include "../core/serialization/Serialization.h"

using namespace psg::core::models;

static const char* bool_str[2] = {"False", "True"};

bool ProcessTestCase(const std::string& name,
                     const std::string& psg_filename,
                     const std::string& cp_filename_fmt,
                     const std::string& out_filename_fmt,
                     int n_cp_files) {
  Log() << "Processing " << name << std::endl;

  std::ifstream psg_file(psg_filename, std::ios::in | std::ios::binary);
  if (!psg_file.is_open()) {
    Error() << "Cannot open psg file " << psg_filename << std::endl;
    return false;
  }
  psg::core::PassiveGripper psg;
  psg.Deserialize(psg_file);
  psg::core::Optimizer optimizer;
  const size_t buf_size = cp_filename_fmt.size() + 16;
  char* buf = new char[buf_size];
  for (int i = 0; i < n_cp_files; i++) {
    snprintf(buf, buf_size, cp_filename_fmt.c_str(), i);
    std::string cp_filename = buf;
    Log() << "> Processing " << cp_filename << std::endl;
    std::ifstream cp_file(cp_filename, std::ios::in | std::ios::binary);
    if (!cp_file.is_open()) {
      Error() << "> Cannot open cp file " << cp_filename << std::endl;
      return false;
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
    snprintf(buf, buf_size, out_filename_fmt.c_str(), i);
    std::string out_filename = buf;
    std::ofstream out_file(out_filename, std::ios::out | std::ios::binary);
    if (!out_file.is_open()) {
      Error() << "> Cannot open out file " << out_filename << std::endl;
      return false;
    }
    psg.SetParams(optimizer.GetCurrentParams());
    psg.Serialize(out_file);
    Log() << "> Optimization took " << duration.count() << " ms." << std::endl;
    Log() << "> Optimized gripper written to " << out_filename << std::endl;
    printf("%s,%s,%s,%s,%.5e,%.5e,%.5e,%.5e,%lld\n",
           name.c_str(),
           out_filename.c_str(),
           bool_str[psg.GetIsForceClosure()],
           bool_str[psg.GetIsPartialClosure()],
           psg.GetMinWrench(),
           psg.GetPartialMinWrench(),
           psg.GetCost(),
           psg.GetMinDist(),
           duration);
  }
  return true;
}