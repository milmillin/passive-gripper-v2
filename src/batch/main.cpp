#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Psgtests.h"
#include "proc.h"
#include "utils.h"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  if (argc < 2) {
    Error() << ".psgtests file required." << std::endl;
    return 1;
  }

  bool root_set = false;
  std::string root;
  for (int i = 2; i < argc; i++) {
    if (strncmp(argv[i], "-r", 4) == 0) {
      root_set = true;
      root = argv[i + 1];
      i++;
    }
  }

  if (root_set) {
    fs::current_path(root);
    Log() << "Working directory set to " << root << std::endl;
  }

  try {
    Psgtests psgtests(argv[1]);
    psgtests.ProcessFrom(0, 0);
  } catch (const std::exception& e) {
    Error() << e.what();
  }
  return 0;
}
