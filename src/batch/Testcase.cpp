#include "Testcase.h"

#include <igl/copyleft/cgal/mesh_boolean.h>
#include <igl/writeSTL.h>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "../core/GeometryUtils.h"
#include "../core/Initialization.h"
#include "../core/Optimizer.h"
#include "../core/PassiveGripper.h"
#include "../core/SweptVolume.h"
#include "../core/TopoOpt.h"
#include "../core/serialization/Serialization.h"
#include "../utils.h"
#include "Result.h"

using namespace psg::core::models;

void ProcessFrom(std::string raw_fn,
                 std::string output_dir,
                 size_t i_cp,
                 size_t need,
                 size_t maxiters,
                 const SettingsOverrider& stgo,
                 const TestcaseCallback& cb) {
  size_t lastslash = raw_fn.rfind('/');
  if (lastslash == std::string::npos) lastslash = raw_fn.rfind('\\');
  std::string wopath_fn = raw_fn.substr(lastslash + 1, std::string::npos);

  Log() << "Processing " << raw_fn << std::endl;

  std::string psg_fn = raw_fn + ".psg";
  std::ifstream psg_file(psg_fn, std::ios::in | std::ios::binary);
  if (!psg_file.is_open()) {
    throw std::invalid_argument("> Cannot open psg file " + psg_fn);
  }
  Log() << "> Loaded " << psg_fn << std::endl;

  std::string cp_fn = raw_fn + ".cp";
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
  for (size_t i = i_cp; i < n_cps && need > 0; i++) {
    psg.reinit_trajectory = true;
    psg.SetContactPoints(cps[i]);
    auto start_time = std::chrono::high_resolution_clock::now();
    optimizer.Optimize(psg);
    optimizer.Wait();
    auto stop_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        stop_time - start_time);
    Log() << "> Optimization took " << duration.count() << " ms." << std::endl;

    psg.SetParams(optimizer.GetCurrentParams());
    bool failed = psg.GetIntersecting();

    snprintf(buf, bufsize, "%s-optd-%03d", wopath_fn.c_str(), i);
    std::string out_raw_fn = buf;
    if (failed) out_raw_fn = "__failed-" + out_raw_fn;
    std::string out_fn = output_dir + '/' + out_raw_fn;

    double pi_volume = -1.;
    double volume = -1.;

    if (!failed) {
      std::string tpd_out_fn = out_fn + ".tpd";
      psg.InitGripperBound();
      Eigen::MatrixXd neg_V;
      Eigen::MatrixXi neg_F;
      NegativeSweptVolumePSG(psg, neg_V, neg_F);
      volume = psg::core::Volume(neg_V, neg_F);
      GenerateTopyConfig(psg, neg_V, neg_F, tpd_out_fn);

      std::string stl_out_fn = out_fn + "_nv.stl";
      igl::writeSTL(stl_out_fn, neg_V, neg_F, igl::FileEncoding::Binary);
      Log() << "> Swept volume written to " << stl_out_fn << std::endl;

      // Compute negative volume
      Eigen::MatrixXd pi_neg_V;
      Eigen::MatrixXi pi_neg_F;
      PiNegativeSweptVolumePSG(psg, pi_neg_V, pi_neg_F);
      pi_volume = psg::core::Volume(pi_neg_V, pi_neg_F);
    }

    std::string psg_out_fn = out_fn + ".psg";
    std::ofstream psg_out_f(psg_out_fn, std::ios::out | std::ios::binary);
    if (!psg_out_f.is_open()) {
      Error() << "> Cannot open out file " << psg_out_fn << std::endl;
      Error() << ">> Skipping" << std::endl;
      continue;
    }
    psg.Serialize(psg_out_f);
    Log() << "> Optimized gripper written to " << psg_out_fn << std::endl;

    Result res{wopath_fn,
               i,
               out_raw_fn,
               failed,
               psg.GetIsForceClosure(),
               psg.GetIsPartialClosure(),
               psg.GetMinWrench(),
               psg.GetPartialMinWrench(),
               psg.GetCost(),
               psg.GetMinDist(),
               volume,
               pi_volume,
               duration.count()};
    Out() << res << std::endl;
    if (!failed) need--;
    if (cb) cb(i, need, res);
  }
  Log() << "Done processing " << wopath_fn << std::endl;
  delete[] buf;
}
