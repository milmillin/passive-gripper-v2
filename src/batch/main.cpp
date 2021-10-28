#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <boost/process.hpp>

#include "../utils.h"
#include "Psgtests.h"
#include "Result.h"

namespace fs = std::filesystem;
namespace bp = boost::process;

int main(int argc, char** argv) {
  if (argc < 2) {
    Error() << ".psgtests file required." << std::endl;
    return 1;
  }

  bool root_set = false;
  std::string root;

  std::string ckpt_fn = ".checkpoint";
  size_t ckpt_i = 0;
  size_t ckpt_j = 0;

  bool restart_set = false;

  bool hook_set = false;
  std::string hook_str;

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
    }
  }

  if (root_set) {
    fs::current_path(root);
    Log() << "Working directory set to " << root << std::endl;
  }

  if (!restart_set) {
    std::ifstream ckpt_file(ckpt_fn);
    if (!ckpt_file.is_open()) {
      Error() << "Warning: cannot open checkpoint file: " << ckpt_fn
              << std::endl;
    } else {
      ckpt_file >> ckpt_i >> ckpt_j;
      Log() << "Checkpoint loaded: " << ckpt_i << ' ' << ckpt_j << std::endl;
      ckpt_j++;
    }
  }

  ProcessCallback cb = [hook_set, &ckpt_fn, &hook_str](
                           size_t i, size_t j, const Testcase& tc) {
    std::ofstream ckpt_file(ckpt_fn);
    if (!ckpt_file.is_open()) {
      Error() << "Warning: cannot open checkpoint file: " << ckpt_fn
              << std::endl;
    } else {
      ckpt_file << i << ' ' << j << std::endl;
      Log() << "Checkpoint updated: " << i << ' ' << j << std::endl;
    }
    if (hook_set) {
      bp::child c(hook_str, std::to_string(i), std::to_string(j), tc.name);
      c.wait();
    }
  };

  try {
    Psgtests psgtests(argv[1]);
    Out() << ResultHeader() << std::endl;
    psgtests.ProcessFrom(ckpt_i, ckpt_j, cb);
  } catch (const std::exception& e) {
    Error() << e.what() << std::endl;
  }
  return 0;
}
