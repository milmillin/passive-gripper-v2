#pragma once

#include <vector>

#include "Testcase.h"
#include "SettingsOverrider.h"

struct Psgtests {
  std::vector<Testcase> testcases;
  const SettingsOverrider* stgo;
  Psgtests(const std::string& filename, const SettingsOverrider& stgo_);
  void ProcessFrom(size_t i_obj,
                   size_t j_cp,
                   size_t ckpt_need,
                   size_t need,
                   const ProcessCallback& cb) const;
};