#pragma once

#include <string>

struct Testcase {
  std::string name;
  std::string psg_filename;
  std::string cp_filename_fmt;
  size_t n_cp_files;
  std::string out_filename_fmt;
};