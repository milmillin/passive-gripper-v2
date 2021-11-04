#pragma once

#include <functional>
#include <string>

#include "Result.h"
#include "SettingsOverrider.h"

typedef std::function<void(size_t, size_t, const Result&)> TestcaseCallback;

void ProcessFrom(std::string raw_fn,
                 std::string output_dir,
                 size_t i_cp,
                 size_t need,
                 size_t maxiters,
                 const SettingsOverrider& stgo,
                 const TestcaseCallback& cb);
