#include <omp.h>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <boost/process.hpp>

#include "../Constants.h"
#include "../utils.h"
#include "Result.h"
#include "SettingsOverrider.h"
#include "Testcase.h"

namespace fs = std::filesystem;
namespace bp = boost::process;

void Usage(char* argv0) {
  Error() << "Usage: " << argv0
          << " psg output_dir [-s stgo] [-h hook] [-x] [-m maxiters]"
          << std::endl;
}

int main(int argc, char** argv) {
  Log() << "Num threads: " << omp_get_max_threads() << std::endl;
  if (argc < 3) {
    Usage(argv[0]);
    return 1;
  }

  // psg_fn
  std::string psg_fn = argv[1];
  size_t lastdot = psg_fn.rfind('.');
  std::string raw_fn = psg_fn.substr(0, lastdot);
  std::string ckpt_fn = raw_fn + ".ckpt";

  // output_dir
  std::string out_dir = argv[2];

  // -x
  bool restart_set = false;

  // -h
  bool hook_set = false;
  std::string hook_str;

  // -s
  bool stgo_set = false;
  std::string stgo_fn;

  // -m
  int maxiters = 15;

  for (int i = 3; i < argc; i++) {
    if (strncmp(argv[i], "-x", 4) == 0) {
      restart_set = true;
    } else if (strncmp(argv[i], "-h", 4) == 0) {
      hook_set = true;
      hook_str = argv[i + 1];
      i++;
    } else if (strncmp(argv[i], "-s", 4) == 0) {
      stgo_set = true;
      stgo_fn = argv[i + 1];
      i++;
    } else if (strncmp(argv[i], "-m", 4) == 0) {
      maxiters = std::stoi(argv[i + 1]);
      i++;
    }
  }

  if (!fs::is_directory(out_dir) || !fs::exists(out_dir)) {
    fs::create_directory(out_dir);
    Log() << out_dir << " directory created " << std::endl;
  }

  int ckpt_i = 0;
  int ckpt_need = 1;
  if (!restart_set) {
    std::ifstream ckpt_file(ckpt_fn);
    if (!ckpt_file.is_open()) {
      Error() << "Warning: cannot open checkpoint file: " << ckpt_fn
              << std::endl;
    } else {
      ckpt_file >> ckpt_i >> ckpt_need;
      Log() << "Checkpoint loaded: " << ckpt_i << ' ' << ckpt_need << std::endl;
      ckpt_i++;
    }
  } else {
    Out() << ResultHeader() << std::endl;
  }

  TestcaseCallback cb = [hook_set, &ckpt_fn, &hook_str, &out_dir](
                            size_t i, size_t need, const Result& r) {
    std::ofstream ckpt_file(ckpt_fn);
    if (!ckpt_file.is_open()) {
      Error() << "Warning: cannot open checkpoint file: " << ckpt_fn
              << std::endl;
    } else {
      ckpt_file << i << ' ' << need << std::endl;
      Log() << "Checkpoint updated: " << i << ' ' << need << std::endl;
    }
    if (hook_set) {
      bp::ipstream out;
      bp::child c(hook_str,
                  bp::std_out > out,
                  r.out_fn,
                  psg::kBoolStr[!r.failed],
                  r.name,
                  std::to_string(r.cp_idx),
                  psg::kBoolStr[r.force_closure],
                  psg::kBoolStr[r.partial_force_closure],
                  ToString(r.min_wrench),
                  ToString(r.partial_min_wrench),
                  ToString(r.cost),
                  ToString(r.min_dist),
                  ToString(r.volume),
                  ToString(r.pi_volume),
                  std::to_string(r.duration),
                  out_dir);
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
    ProcessFrom(raw_fn, out_dir, ckpt_i, ckpt_need, maxiters, stgo, cb);
  } catch (const std::exception& e) {
    Error() << e.what() << std::endl;
  }
  return 0;
}
