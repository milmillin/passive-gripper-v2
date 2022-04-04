#pragma once

#include <functional>
#include <string>

#include "../core/models/SettingsOverrider.h"
#include "Result.h"

// A function to process the optimization result
typedef std::function<void(size_t, size_t, const Result&)> TestcaseCallback;

// Optimize for one object
// raw_fn: file path to configuration files without extension
// output_dir: file path to store optimization result
// i_cp: index of the first grasp candidate to try (inclusive)
// need: stop optimization after we find this number of success grasp candidates
// maxiters: index of the last grasp candidate to try (exclusive)
// stgo: settings to override
// cb: a function to process the optimization result
void ProcessFrom(std::string raw_fn,
                 std::string output_dir,
                 size_t i_cp,
                 size_t need,
                 size_t maxiters,
                 const psg::core::models::SettingsOverrider& stgo,
                 const TestcaseCallback& cb);
