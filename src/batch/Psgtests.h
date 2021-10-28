#pragma once

#include <vector>

#include "Testcase.h"

struct Psgtests {
  std::vector<Testcase> testcases;
  Psgtests(const std::string& filename);
  void ProcessFrom(size_t i_obj, size_t j_cp);
};