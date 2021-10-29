#include "Psgtests.h"

#include <fstream>
#include <iostream>
#include <stdexcept>

#include "../utils.h"

Psgtests::Psgtests(const std::string& filename, const SettingsOverrider& stgo_) {
  stgo = &stgo_;
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
      tf >> name;
      testcases.emplace_back();
      testcases.back().name = name;
    }
  } else {
    throw std::invalid_argument("Unkown Version " + std::to_string(version));
  }
}

void Psgtests::ProcessFrom(size_t i_obj,
                           size_t j_cp,
                           size_t ckpt_need,
                           size_t need,
                           const ProcessCallback& cb) const {
  TestcaseCallback tcb;
  if (i_obj < testcases.size()) {
    try {
      size_t need_ = ckpt_need;
      if (cb)
        tcb = [i_obj, &cb, &need_](const Result& r) {
          if (!r.failed) need_--;
          cb(i_obj, r.cp_idx, need_, r);
        };
      testcases[i_obj].ProcessFrom(j_cp, need_, *stgo, tcb);
    } catch (const std::exception& e) {
      Error() << e.what() << std::endl;
    }
  }
  for (size_t i = i_obj + 1; i < testcases.size(); i++) {
    try {
      size_t need_ = need;
      if (cb) 
        tcb = [i, &cb, &need_](const Result& r) {
          if (!r.failed) need_--;
          cb(i, r.cp_idx, need_, r);
        };
      testcases[i].ProcessFrom(0, need_, *stgo, tcb);
    } catch (const std::exception& e) {
      Error() << e.what() << std::endl;
    }
  }
}
