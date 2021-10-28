#include "Psgtests.h"

#include <stdexcept>

#include "utils.h"

Psgtests::Psgtests(const std::string& filename) {
  std::ifstream tf(filename);
  if (!tf.is_open()) {
    throw std::invalid_argument("Error openning file " + filename);
  }

  std::string signature;
  tf >> signature;
  if (signature != "%PSGTESTS") {
    throw std::invalid_argument("Invalid header");
  }
  int version = -1;
  tf >> version;
  if (version == 1) {
    int n_tests = -1;
    tf >> n_tests;
    testcases.reserve(n_tests);
    for (int i = 0; i < n_tests; i++) {
      std::string name;
      std::string psg_filename;
      std::string cp_filename_fmt;
      int n_cp_files;
      std::string out_filename_fmt;
      tf >> name >> psg_filename >> cp_filename_fmt >> n_cp_files >>
          out_filename_fmt;
      testcases.emplace_back();
      testcases.back().name = name;
      testcases.back().psg_filename = psg_filename;
      testcases.back().cp_filename_fmt = cp_filename_fmt;
      testcases.back().n_cp_files = n_cp_files;
      testcases.back().out_filename_fmt = out_filename_fmt;
    }
  } else {
    throw std::invalid_argument("Unkown Version " + std::to_string(version));
  }
}

Psgtests::ProcessFrom(size_t i_obj, size_t j_cp) {
}
