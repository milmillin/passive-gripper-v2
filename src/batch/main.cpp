#include <omp.h>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <boost/process.hpp>

#include "../utils.h"
#include "../Constants.h"
#include "Psgtests.h"
#include "Result.h"
#include "SettingsOverrider.h"

namespace fs = std::filesystem;
namespace bp = boost::process;

int main(int argc, char** argv) {
  Log() << "Num threads: " << omp_get_max_threads() << std::endl;
  if (argc < 2) {
    Error() << ".psgtests file required." << std::endl;
    return 1;
  }

  bool root_set = false;
  std::string root;

  std::string ckpt_fn = ".checkpoint";
  size_t ckpt_i = 0;
  size_t ckpt_j = 0;
  constexpr size_t kNeed = 1;
  size_t ckpt_need = kNeed;
  bool restart_set = false;

  bool hook_set = false;
  std::string hook_str;

  bool stgo_set = false;
  std::string stgo_fn;

  for (int i = 2; i < argc; i++) {
    if (strncmp(argv[i], "-r", 4) == 0) {
      root_set = true;
      root = argv[i + 1];
      i++;
    } else if (strncmp(argv[i], "-c", 4) == 0) {
      ckpt_fn = argv[i + 1];
      i++;
    } else if (strncmp(argv[i], "-x", 4) == 0) {
      restart_set = true;
    } else if (strncmp(argv[i], "-h", 4) == 0) {
      hook_set = true;
      hook_str = argv[i + 1];
      i++;
    } else if (strncmp(argv[i], "-s", 4) == 0) {
      stgo_set = true;
      stgo_fn = argv[i + 1];
      i++;
    }
  }

  if (root_set) {
    fs::current_path(root);
    Log() << "Working directory set to " << root << std::endl;
  }

  if (!fs::is_directory("../output") || !fs::exists("../output")) {
    fs::create_directory("../output");
    Log() << "Output directory created" << std::endl;
  }

  if (!restart_set) {
    std::ifstream ckpt_file(ckpt_fn);
    if (!ckpt_file.is_open()) {
      Error() << "Warning: cannot open checkpoint file: " << ckpt_fn
              << std::endl;
    } else {
      ckpt_file >> ckpt_i >> ckpt_j >> ckpt_need;
      Log() << "Checkpoint loaded: " << ckpt_i << ' ' << ckpt_j << ' '
            << ckpt_need << std::endl;
      ckpt_j++;
    }
  }

  ProcessCallback cb = [hook_set, &ckpt_fn, &hook_str](
                           size_t i, size_t j, size_t need, const Result& r) {
    std::ofstream ckpt_file(ckpt_fn);
    if (!ckpt_file.is_open()) {
      Error() << "Warning: cannot open checkpoint file: " << ckpt_fn
              << std::endl;
    } else {
      ckpt_file << i << ' ' << j << ' ' << need << std::endl;
      Log() << "Checkpoint updated: " << i << ' ' << j << std::endl;
    }
    if (hook_set) {
      bp::ipstream out;
      bp::child c(hook_str,
                  bp::std_out > out,
                  r.name + "-" + std::to_string(r.cp_idx),
                  psg::kBoolStr[!r.failed],
                  r.name,
                  std::to_string(r.cp_idx),
                  psg::kBoolStr[r.force_closure],
                  psg::kBoolStr[r.partial_force_closure],
                  ToString(r.min_wrench),
                  ToString(r.partial_min_wrench),
                  ToString(r.cost),
                  ToString(r.min_dist),
                  std::to_string(r.duration));
      char a[1024];
      auto& out_s = Log() << "[child proc]" << std::endl;
      while (out.read(a, 1024)) {
        int read = out.gcount();
        for (int i = 0; i < read; i++) {
          out_s << a[i];
        }
      }
      out_s << std::endl;
      c.wait();
    }
  };

  try {
    SettingsOverrider stgo;
    if (stgo_set) stgo.Load(stgo_fn);
    Psgtests psgtests(argv[1], stgo);
    Out() << ResultHeader() << std::endl;
    psgtests.ProcessFrom(ckpt_i, ckpt_j, ckpt_need, kNeed, cb);
  } catch (const std::exception& e) {
    Error() << e.what() << std::endl;
  }
  return 0;
}
