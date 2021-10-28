#pragma once

#include <string>
#include <functional>

struct Testcase;

typedef std::function<void(size_t, size_t, const Testcase&)> ProcessCallback;
typedef std::function<void(size_t, const Testcase&)> TestcaseCallback;

struct Testcase {
  std::string name;
  std::string psg_filename;
  std::string cp_filename_fmt;
  size_t n_cp_files;
  std::string out_filename_fmt;

  void ProcessFrom(size_t j_cp, const TestcaseCallback& cb) const;
};