#pragma once

#include <iomanip>
#include <iostream>
#include <string>

struct Result {
  std::string name;
  std::string out_filename;
  bool force_closure;
  bool partial_force_closure;
  double min_wrench;
  double partial_min_wrench;
  double cost;
  double min_dist;
  double duration;
};

static const char* bool_str[2] = {"False", "True"};

inline std::ostream& operator<<(std::ostream& f, const Result& r) {
  f << std::setprecision(5) << std::scientific;
  f << r.name << '\t' << r.out_filename << '\t' << bool_str[r.force_closure]
    << '\t' << bool_str[r.partial_force_closure] << '\t' << r.min_wrench << '\t'
    << r.partial_min_wrench << '\t' << r.cost << '\t' << r.min_dist << '\t'
    << r.duration;
  return f;
}

struct ResultHeader {};

inline std::ostream& operator<<(std::ostream& f, const ResultHeader& r) {
  f << "name\tcname\tfc\tpfc\tmw\tpmw\tcost\tdist\ttime";
  return f;
}
