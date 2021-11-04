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

  OptSettings opt_settings = psg.GetOptSettings();
  TopoOptSettings topo_opt_settings = psg.GetTopoOptSettings();
  bool opt_changed = false;
  bool topo_opt_changed = false;

  for (const auto& kv : mp) {
    if (kv.first == "algorithm") {
      opt_settings.algorithm = (nlopt_algorithm)std::stoi(kv.second);
      opt_changed = true;
    } else if (kv.first == "tolerance") {
      opt_settings.tolerance = std::stod(kv.second);
      opt_changed = true;
    } else if (kv.first == "population") {
      opt_settings.population = std::stoull(kv.second);
      opt_changed = true;
    } else if (kv.first == "max_runtime") {
      opt_settings.max_runtime = std::stod(kv.second);
      opt_changed = true;
    } else if (kv.first == "max_iters") {
      opt_settings.max_iters = std::stoull(kv.second);
      opt_changed = true;
    } else if (kv.first == "vol_frac") {
      topo_opt_settings.vol_frac = std::stod(kv.second);
      topo_opt_changed = true;
    } else if (kv.first == "topo_res") {
      topo_opt_settings.topo_res = std::stod(kv.second);
      topo_opt_changed = true;
    } else if (kv.first == "neg_vol_res") {
      topo_opt_settings.neg_vol_res = std::stod(kv.second);
      topo_opt_changed = true;
    }
  }

  if (opt_changed) psg.SetOptSettings(opt_settings);
  if (topo_opt_changed) psg.SetTopoOptSettings(topo_opt_settings);

  psg.reinit_fingers = tmp_reinit_fingers;
  psg.reinit_trajectory = tmp_reinit_trajectory;
}
