#include "Testcase.h"

#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "../core/Optimizer.h"
#include "../core/PassiveGripper.h"
#include "../core/SweptVolume.h"
#include "../core/TopoOpt.h"
#include "../core/serialization/Serialization.h"
#include "../utils.h"
#include "Result.h"

using namespace psg::core::models;

void Testcase::ProcessFrom(size_t j_cp,
                           size_t need,
                           const SettingsOverrider& stgo,
                           const TestcaseCallback& cb) const {
  Log() << "Processing " << name << std::endl;

  std::string psg_fn = name + ".psg";
  std::ifstream psg_file(psg_fn, std::ios::in | std::ios::binary);
  if (!psg_file.is_open()) {
    throw std::invalid_argument("> Cannot open psg file " + psg_fn);
  }
  Log() << "> Loaded " << psg_fn << std::endl;

  std::string cp_fn = name + ".cp";
  std::ifstream cp_file(cp_fn, std::ios::in | std::ios::binary);
  if (!cp_file.is_open()) {
    throw std::invalid_argument("> Cannot open cp file " + cp_fn);
  }
  Log() << "> Loaded " << cp_fn << std::endl;

  psg::core::PassiveGripper psg;
  psg.Deserialize(psg_file);
  stgo.Apply(psg);
  psg::core::Optimizer optimizer;

  std::vector<std::vector<ContactPoint>> cps;
  psg::core::serialization::Deserialize(cps, cp_file);

  constexpr size_t bufsize = 48;
  char* buf = new char[bufsize];

  size_t n_cps = cps.size();
  for (size_t i = j_cp; i < n_cps && need > 0; i++) {
    psg.reinit_trajectory = true;
    psg.SetContactPoints(cps[i]);
    auto start_time = std::chrono::high_resolution_clock::now();
    optimizer.Optimize(psg);
    optimizer.Wait();
    auto stop_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        stop_time - start_time);
    psg.SetParams(optimizer.GetCurrentParams());
    bool failed = psg.GetMinDist() < -1e-5;

    std::string out_fn_fmt =
        "../output/" + ((failed ? "__failed-" : "") + name) + "-optd-%03d";
    snprintf(buf, bufsize, out_fn_fmt.c_str(), i);
    std::string out_fn = buf;

    if (!failed) {
      std::string tpd_out_fn = out_fn + ".tpd";
      psg.InitGripperBound();
      Eigen::MatrixXd neg_V;
      Eigen::MatrixXi neg_F;
      NegativeSweptVolume(psg, neg_V, neg_F);
      GenerateTopyConfig(psg, neg_V, neg_F, tpd_out_fn);
    }

    std::string psg_out_fn = out_fn + ".psg";
    std::ofstream psg_out_f(psg_out_fn, std::ios::out | std::ios::binary);
    if (!psg_out_f.is_open()) {
      Error() << "> Cannot open out file " << psg_out_fn << std::endl;
      Error() << ">> Skipping" << std::endl;
      continue;
    }
    psg.Serialize(psg_out_f);

    Log() << "> Optimization took " << duration.count() << " ms." << std::endl;
    Log() << "> Optimized gripper written to " << psg_out_fn << std::endl;
    Result res{name,
               i,
               out_fn,
               failed,
               psg.GetIsForceClosure(),
               psg.GetIsPartialClosure(),
               psg.GetMinWrench(),
               psg.GetPartialMinWrench(),
               psg.GetCost(),
               psg.GetMinDist(),
               duration.count()};
    Out() << res << std::endl;
    if (!failed) need--;
    if (cb) cb(res);
  }
  Log() << "Done processing " << name << std::endl;
  delete[] buf;
}