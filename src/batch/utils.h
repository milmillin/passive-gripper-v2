#pragma once

#include <iostream>
#include <string>

std::ostream& LogImpl(const char* type);

inline std::ostream& Log() {
  return LogImpl("info");
}
inline std::ostream& Error() {
  return LogImpl("error");
}
