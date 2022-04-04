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
#include "../core/QualityMetric.h"
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
  // Remove folder name from the file name
  size_t lastslash = raw_fn.rfind('/');
  if (lastslash == std::string::npos) lastslash = raw_fn.rfind('\\');
  std::string wopath_fn = raw_fn.substr(lastslash + 1, std::string::npos);

  Log() << "Processing " << raw_fn << std::endl;

  // Load optimization configuration
  std::string psg_fn = raw_fn + ".psg";
  std::ifstream psg_file(psg_fn, std::ios::in | std::ios::binary);
  if (!psg_file.is_open()) {
    throw std::invalid_argument("> Cannot open psg file " + psg_fn);
  }

  psg::core::PassiveGripper psg;
  psg.Deserialize(psg_file);
  Log() << "> Loaded " << psg_fn << std::endl;

  // Override optimization settings
  stgo.Apply(psg);

  // Try to also apply override settings from the same folder
  try {
    psg::core::models::SettingsOverrider stgo;
    std::string stgo_fn = raw_fn + ".stgo";
    stgo.Load(stgo_fn);
    stgo.Apply(psg);
    Log() << "> Loaded " << stgo_fn << std::endl;
  } catch (std::invalid_argument const&) {
    ;  // Doesn't contain override file
  }

  Log() << psg.GetOptSettings() << std::endl;
  Log() << psg.GetTopoOptSettings() << std::endl;


  // Load contact points
  std::string cp_fn = raw_fn + ".cpx";
  std::ifstream cp_file(cp_fn, std::ios::in | std::ios::binary);
  if (!cp_file.is_open()) {
    throw std::invalid_argument("> Cannot open cp file " + cp_fn);
  }

  std::vector<ContactPointMetric> cps;
  psg::core::serialization::Deserialize(cps, cp_file);
  Log() << "> Loaded " << cp_fn << std::endl;

  // Optimize for each grasp candidate
  psg::core::Optimizer optimizer;
  size_t n_cps = std::min(cps.size(), maxiters);
  constexpr size_t bufsize = 48;
  char* buf = new char[bufsize];
  for (size_t i = i_cp; i < n_cps && need > 0; i++) {
    // Optimize
    Log() << "> Optimizating for " << i << "-th candidate" << std::endl;
    psg.reinit_trajectory = true;
    psg.SetContactPoints(cps[i].contact_points);
    auto start_time = std::chrono::high_resolution_clock::now();
    optimizer.Optimize(psg);
    optimizer.Wait();
    auto stop_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        stop_time - start_time);
    Log() << "> Optimization took " << duration.count() << " ms." << std::endl;

    // Process optimization result
    psg.SetParams(optimizer.GetCurrentParams());
    bool failed = psg.GetMinDist() < -1e-5;

    Log() << "> Success: " << psg::kBoolStr[!failed] << std::endl;

    snprintf(buf, bufsize, "%s-optd-%03d", wopath_fn.c_str(), (int)i);
    std::string out_raw_fn = buf;
    if (failed) out_raw_fn = "__failed-" + out_raw_fn;
    std::string out_fn = output_dir + '/' + out_raw_fn;

    double traj_complexity = 0;
    double pi_volume = -1.;
    double volume = -1.;
    Eigen::MatrixXd neg_V;
    Eigen::MatrixXi neg_F;

    if (!failed) {
      // Generate topology optimization configuration
      Log() << "> Generating TPD file" << std::endl;
      std::string tpd_out_fn = out_fn + ".tpd";
      psg.InitGripperBound();
      NegativeSweptVolumePSG(psg, neg_V, neg_F);
      volume = psg::core::Volume(neg_V, neg_F);
      GenerateTopyConfig(psg, neg_V, neg_F, tpd_out_fn, nullptr);
      Log() << ">> Done: TPD file written to " << tpd_out_fn << std::endl;

      // Compute negative volume
      /*
      Log() << "> Computing PI Volume" << std::endl;
      Eigen::MatrixXd pi_neg_V;
      Eigen::MatrixXi pi_neg_F;
      PiNegativeSweptVolumePSG(psg, pi_neg_V, pi_neg_F);
      pi_volume = psg::core::Volume(pi_neg_V, pi_neg_F);
      Log() << ">> Done" << std::endl;
      */

      Log() << "> Computing Traj Complexity" << std::endl;
      traj_complexity = psg::core::GetTrajectoryComplexity(psg.GetTrajectory());
      Log() << "> Done" << std::endl;
    }

    // Save optimization configuration
    std::string psg_out_fn = out_fn + ".psg";
    {
      std::ofstream psg_out_f(psg_out_fn, std::ios::out | std::ios::binary);
      if (!psg_out_f.is_open()) {
        Error() << "> Cannot open out file " << psg_out_fn << std::endl;
        Error() << ">> Skipping" << std::endl;
        continue;
      }
      psg.Serialize(psg_out_f);
    }
    Log() << "> Done: Optimized gripper written to " << psg_out_fn << std::endl;

    // Save the trajectory as a CSV file, where each row is a step in time and
    // each column is the angle for a joint
    std::string csv_out_fn = out_fn + ".csv";
    {
      std::ofstream traj_csv_file(csv_out_fn);
      if (!traj_csv_file.is_open()) {
        Error() << "> Cannot open out file " << csv_out_fn << std::endl;
      } else {
        const psg::Trajectory& traj = psg.GetTrajectory();
        for (int i = traj.size() - 1; i >= 0; i--) {
          for (int j = 0; j < psg::kNumDOFs; j++) {
            traj_csv_file << std::setprecision(15) << traj[i](j) << ",";
          }
          traj_csv_file << std::endl;
        }
      }
    }
    Log() << ">> Done: Trajectory dumped to " << csv_out_fn << std::endl;

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
               psg.GetIntersecting(),
               volume,
               traj_complexity,
               duration.count()};
    Out() << res << std::endl;
    if (!failed) need--;
    if (cb) cb(i, need, res);
    if (!failed) {
      // Load topology optimization results
      std::string bin_fn = out_fn + ".bin";
      std::string gripper_fn = out_fn + ".stl";
      Eigen::MatrixXd r_V;
      Eigen::MatrixXi r_F;
      Eigen::MatrixXd gripper_V;
      Eigen::MatrixXi gripper_F;
      Log() << "> Loading Result Bin " << bin_fn << std::endl;
      if (!psg::core::LoadResultBin(psg, bin_fn, r_V, r_F)) {
        Error() << ">> Error loading result bin" << std::endl;
        Error() << ">> Skipping" << std::endl;
        continue;
      }
      psg::core::RefineGripper(
          psg, r_V, r_F, neg_V, neg_F, gripper_V, gripper_F);
      Log() << "> Writing Gripper STL " << gripper_fn << std::endl;
      if (!igl::writeSTL(
              gripper_fn, gripper_V, gripper_F, igl::FileEncoding::Binary)) {
        Error() << ">> Error saving gripper STL" << std::endl;
        Error() << ">> Skipping" << std::endl;
        continue;
      }
    }
  }
  Log() << "Done processing " << wopath_fn << std::endl;
  delete[] buf;
}
