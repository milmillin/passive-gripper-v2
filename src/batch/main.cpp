#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

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

  std::ifstream tf(argv[1]);
  if (!tf.is_open()) {
    Error() << "Error openning file " << argv[1] << std::endl;
    return 1;
  }

  std::string signature;
  tf >> signature;
  if (signature != "%PSGTESTS") {
    Error() << "Invalid header" << std::endl;
    return 1;
  }
  int version;
  tf >> version;
  if (version == 1) {
    int n_tests;
    tf >> n_tests;
    // table header
    printf("name,id,fc,pfc,mw,pmw,cost,mind,time\n");
    for (int i = 0; i < n_tests; i++) {
      std::string name;
      std::string psg_filename;
      std::string cp_filename_fmt;
      int n_cp_files;
      std::string out_filename_fmt;
      tf >> name >> psg_filename >> cp_filename_fmt >> n_cp_files >>
          out_filename_fmt;

      if (!ProcessTestCase(name,
                           psg_filename,
                           cp_filename_fmt,
                           out_filename_fmt,
                           n_cp_files)) {
        Error() << "An error has occurred" << std::endl;
        return 1;
      }
    }
  } else {
    Error() << "Unknown Version " << version << std::endl;
    return 1;
  }
  return 0;
}
