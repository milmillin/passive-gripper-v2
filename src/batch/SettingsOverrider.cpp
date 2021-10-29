#include "SettingsOverrider.h"

#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "../utils.h"
#include "../core/PassiveGripper.h"

using namespace psg::core::models;

void SettingsOverrider::Load(std::string fn) {
  std::ifstream f(fn);
  if (!f.is_open()) {
    throw std::invalid_argument("Cannot open file " + fn);
  }

  mp.clear();
  std::string key;
  std::string value;
  while (f >> key) {
    f >> value;      
    mp.insert({key, value});
  }
}

void SettingsOverrider::Apply(psg::core::PassiveGripper& psg) const {
  bool tmp_reinit_fingers = psg.reinit_fingers;
  bool tmp_reinit_trajectory = psg.reinit_trajectory;
  psg.reinit_fingers = false;
  psg.reinit_trajectory = false;
  for (const auto& kv : mp) {
    if (kv.first == "algorithm") {
      OptSettings settings = psg.GetOptSettings();
      settings.algorithm = (nlopt_algorithm)std::stoi(kv.second);
      psg.SetOptSettings(settings);
    } else if (kv.first == "tolerance") {
      OptSettings settings = psg.GetOptSettings();
      settings.tolerance = std::stod(kv.second);
      psg.SetOptSettings(settings);
    }
  }
  psg.reinit_fingers = tmp_reinit_fingers;
  psg.reinit_trajectory = tmp_reinit_trajectory;
}
