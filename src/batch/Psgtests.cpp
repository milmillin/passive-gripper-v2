#include "Psgtests.h"

#include <fstream>
#include <iostream>
#include <stdexcept>

#include "../utils.h"

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

void Psgtests::ProcessFrom(size_t i_obj,
                           size_t j_cp,
                           const ProcessCallback& cb) const {
  TestcaseCallback tcb;
  if (i_obj < testcases.size()) {
    try {
      if (cb) tcb = [i_obj, &cb](size_t j, const Testcase& tc) { cb(i_obj, j, tc); };
      testcases[i_obj].ProcessFrom(j_cp, tcb);
    } catch (const std::exception& e) {
      Error() << e.what() << std::endl;
    }
  }
  for (size_t i = i_obj + 1; i < testcases.size(); i++) {
    try {
      if (cb) tcb = [i, &cb](size_t j, const Testcase& tc) { cb(i, j, tc); };
      testcases[i].ProcessFrom(0, tcb);
    } catch (const std::exception& e) {
      Error() << e.what() << std::endl;
    }
  }
}
