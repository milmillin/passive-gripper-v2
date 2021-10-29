#pragma once

#include <map>
#include <string>
#include <vector>

#include "../core/PassiveGripper.h"

class SettingsOverrider {
 private:
  std::map<std::string, std::string> mp;

 public:
  void Load(std::string fn);
  void Apply(psg::core::PassiveGripper& psg) const;
};