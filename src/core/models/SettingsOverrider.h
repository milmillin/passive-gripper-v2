#pragma once

#include <map>
#include <string>
#include <vector>

#include "../PassiveGripper.h"

namespace psg {
namespace core {
namespace models {

class SettingsOverrider {
 private:
  std::map<std::string, std::string> mp;

 public:
  void Load(std::string fn);
  void Apply(psg::core::PassiveGripper& psg) const;
};

}  // namespace models
}  // namespace core
}  // namespace psg