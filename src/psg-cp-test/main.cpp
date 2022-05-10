#include <omp.h>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <boost/process.hpp>

#include "../Constants.h"
#include "../core/Optimizer.h"
#include "../core/models/ContactPointMetric.h"
#include "../core/models/SettingsOverrider.h"
#include "../utils.h"

namespace fs = std::filesystem;
namespace bp = boost::process;

char buf[256];

int main(int argc, char** argv) {
  // psg_fn
  std::string psg_fn = argv[1];
  size_t lastdot = psg_fn.rfind('.');
  std::string raw_fn = psg_fn.substr(0, lastdot);

  // ckpt_fn
  std::string ckpt_fn = argv[2];

  // cpx_fn
  std::string cp_fn = argv[3];

  // out_csv_fn
  std::string csv_fn = argv[4];

  // output_dir
  std::string out_dir = argv[5];

  if (!fs::is_directory(out_dir) || !fs::exists(out_dir)) {
    fs::create_directory(out_dir);
    Log() << out_dir << " directory created " << std::endl;
  }

  std::ofstream csv_file(csv_fn, std::ios::app);
  if (!csv_file.is_open()) {
    throw std::invalid_argument("> Cannot open csv file " + csv_fn);
  }

  int ckpt_i = 0;
  {
    std::ifstream ckpt_file(ckpt_fn);
    if (!ckpt_file.is_open()) {
      Error() << "Warning: cannot open checkpoint file: " << ckpt_fn
              << std::endl;
      // write header
    } else {
      ckpt_file >> ckpt_i;
      Log() << "Checkpoint loaded: " << ckpt_i << std::endl;
      ckpt_i++;
    }
  }

  size_t lastslash = raw_fn.rfind('/');
  if (lastslash == std::string::npos) lastslash = raw_fn.rfind('\\');
  std::string wopath_fn = raw_fn.substr(lastslash + 1, std::string::npos);

  Log() << "Processing " << raw_fn << std::endl;

  std::ifstream psg_file(psg_fn, std::ios::in | std::ios::binary);
  if (!psg_file.is_open()) {
    throw std::invalid_argument("> Cannot open psg file " + psg_fn);
  }

  std::ifstream cp_file(cp_fn, std::ios::in | std::ios::binary);
  if (!cp_file.is_open()) {
    throw std::invalid_argument("> Cannot open cp file " + cp_fn);
  }
  std::vector<psg::core::models::ContactPointMetric> cps;
  psg::core::serialization::Deserialize(cps, cp_file);
  Log() << "> Loaded " << cp_fn << std::endl;

  psg::core::PassiveGripper psg;
  psg.Deserialize(psg_file);
  Log() << "> Loaded " << psg_fn << std::endl;

  psg::core::Optimizer optimizer;

  Log() << psg.GetOptSettings() << std::endl;
  Log() << psg.GetTopoOptSettings() << std::endl;

  for (size_t i = ckpt_i; i < cps.size(); i++) {
    Log() << "> Optimizating for " << i << "-th candidate" << std::endl;
    psg.reinit_trajectory = true;
    psg.SetContactPoints(cps[i].contact_points);
    auto start_time = std::chrono::high_resolution_clock::now();
    optimizer.Optimize(psg);
    optimizer.Wait();
    auto stop_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        stop_time - start_time);
    Log() << "> Optimization took " << duration.count() << " ms." << std::endl;

    psg.SetParams(optimizer.GetCurrentParams());
    bool failed = psg.GetMinDist() < -1e-5;

    csv_file << wopath_fn << " " << cps[i].finger_distance << " "
             << cps[i].partial_min_wrench << " " << psg::kBoolStr[!failed]
             << std::endl;

    snprintf(
        buf, sizeof(buf), "%s/%s-%05d.params", out_dir.c_str(), wopath_fn.c_str(), i);
    std::string out_fn = buf;
    psg.GetParams().SerializeFn(out_fn);
    Log() << ">> Done: Params dumped to " << out_fn << std::endl;

    {
      std::ofstream ckpt_file(ckpt_fn);
      if (!ckpt_file.is_open()) {
        Error() << "Warning: cannot open checkpoint file: " << ckpt_fn
                << std::endl;
      } else {
        ckpt_file << i << std::endl;
        Log() << "Checkpoint updated: " << i << std::endl;
      }
    }
  }
}
