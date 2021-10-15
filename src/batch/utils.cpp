#include "utils.h"

#include <chrono>
#include <ctime>

std::ostream& LogImpl(const char* type) {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  return std::cerr << "[" << std::ctime(&now_time) << "][" << type << "] ";
}
