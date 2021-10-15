#pragma once

#include <string>

bool ProcessTestCase(const std::string& name,
                     const std::string& psg_filename,
                     const std::string& cp_filename_fmt,
                     const std::string& out_filename_fmt,
                     int n_cp_files);