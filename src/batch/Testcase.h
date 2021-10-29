#pragma once

#include <string>
#include <functional>

#include "Result.h"
#include "SettingsOverrider.h"

typedef std::function<void(size_t, size_t, size_t, const Result&)> ProcessCallback;
typedef std::function<void(const Result&)> TestcaseCallback;

struct Testcase {
  std::string name;

  void ProcessFrom(size_t j_cp, size_t need, const SettingsOverrider& stgo, const TestcaseCallback& cb) const;
};