#include "SettingsOverrider.h"

#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "../PassiveGripper.h"
#include "../../utils.h"

namespace psg {
namespace core {
namespace models {

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
    Log() << key << ": " << value << std::endl;
    if (key == "algorithm") {
      Log() << "  > " << psg::labels::kAlgorithms[std::stoi(value)]
            << std::endl;
    }
  }
}

void SettingsOverrider::Apply(psg::core::PassiveGripper& psg) const {
  bool tmp_reinit_fingers = psg.reinit_fingers;
  bool tmp_reinit_trajectory = psg.reinit_trajectory;
  psg.reinit_fingers = false;
  psg.reinit_trajectory = false;

  OptSettings opt_settings = psg.GetOptSettings();
  TopoOptSettings topo_opt_settings = psg.GetTopoOptSettings();
  ContactSettings contact_settings = psg.GetContactSettings();
  CostSettings cost_settings = psg.GetCostSettings();
  bool opt_changed = false;
  bool topo_opt_changed = false;
  bool cost_settings_changed = false;
  bool contact_settings_changed = false;

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
    } else if (kv.first == "cost.floor") {
      cost_settings.floor = std::stod(kv.second);
      cost_settings_changed = true;
    } else if (kv.first == "contact.floor") {
      contact_settings.floor = std::stod(kv.second);
      contact_settings_changed = true;
    } else {
      Error() << "Unknown settings: " << kv.first << std::endl;    
    }
  }

  if (opt_changed) psg.SetOptSettings(opt_settings);
  if (topo_opt_changed) psg.SetTopoOptSettings(topo_opt_settings);
  if (contact_settings_changed) psg.SetContactSettings(contact_settings);
  if (cost_settings_changed) psg.SetCostSettings(cost_settings);

  psg.reinit_fingers = tmp_reinit_fingers;
  psg.reinit_trajectory = tmp_reinit_trajectory;
}

}  // namespace models
}  // namespace core
}  // namespace psg
