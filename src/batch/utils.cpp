#include "utils.h"

#include <chrono>
#include <ctime>

std::ostream& LogImpl(const char* type) {
  static thread_local char buf[64];
  std::time_t t = std::time(nullptr);
  std::strftime(buf, 64, "%F %T", std::localtime(&t));
  return std::cerr << "[" << buf << "][" << type << "] ";
}
