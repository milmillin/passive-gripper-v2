#include <omp.h>
#include <boost/process.hpp>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "../Constants.h"
#include "../core/Optimizer.h"
#include "../core/PassiveGripper.h"
#include "../core/QualityMetric.h"
#include "../utils.h"

namespace fs = std::filesystem;
namespace bp = boost::process;
namespace psgc = psg::core;
namespace psgs = psg::core::serialization;
namespace psgm = psg::core::models;

char __buf[1024];
std::string FormatOutput(const psgm::CostSettings& s, int iter) {
  std::snprintf(__buf,
                1024,
                "-%02d-%04lld-%.0f-%.0f-%.0f-%.0f",
                iter,
                s.n_trajectory_steps,
                s.geodesic_contrib,
                s.inner_dis_contrib,
                s.gripper_energy,
                s.traj_energy);
  return __buf;
}

struct Testcase {
  size_t d_subdivision;
  double geodesic_contrib;
  double inner_dis_contrib;
  double gripper_energy;
  double traj_energy;
};

// std::vector<double> subdivisions =
// {0.0001, 0.0003, 0.0005, 0.001, 0.005, 0.01, 0.03, 0.05};

std::vector<size_t> subdivisions = {1024, 512, 256, 128, 64, 32};

std::vector<Testcase> testcases;

void GenerateTestcases() {
  for (size_t sub : subdivisions) {
    // testcases.push_back(Testcase{sub, 1, 1, 1, 1});
    testcases.push_back(Testcase{sub, 0, 1, 1, 1});
    // testcases.push_back(Testcase{sub, 1, 1, 1, 0});
    testcases.push_back(Testcase{sub, 0, 1, 1, 0});
    // testcases.push_back(Testcase{sub, 1, 1, 0, 1});
    // testcases.push_back(Testcase{sub, 0, 1, 0, 1});
  }
}

int main(int argc, char** argv) {
  GenerateTestcases();
  Log() << "Num threads: " << omp_get_max_threads() << std::endl;
  if (argc < 3) {
    return 1;
  }

  // psg_fn
  std::string psg_fn = argv[1];
  size_t lastdot = psg_fn.rfind('.');
  std::string raw_fn = psg_fn.substr(0, lastdot);

  size_t lastslash = raw_fn.rfind('/');
  if (lastslash == std::string::npos) lastslash = raw_fn.rfind('\\');
  std::string wopath_fn = raw_fn.substr(lastslash + 1, std::string::npos);

  // output_dir
  std::string out_dir = argv[2];

  // -h hook
  bool hook_set = false;
  std::string hook_str;

  // -m iters
  int iters = 2;

  for (int i = 3; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "-h") {
      hook_set = true;
      hook_str = argv[i + 1];
      i++;
    } else if (arg == "-m") {
      iters = std::stoi(argv[i + 1]);
      i++;
    }
  }

  std::string out_raw_fn = out_dir + "/" + wopath_fn;
  std::string res_fn = out_dir + "/" + wopath_fn + "_ablation.csv";
  std::ofstream res_f(res_fn, std::ios::out);
  if (!res_f.is_open()) {
    Error() << "error: cannot open " << res_fn << std::endl;
  }
  res_f << "name\tsub\tgeodesic_contrib\tinner_contrib\tgripper_energy\ttraj_"
           "energy\tsuccess\tdist\ttime\tcost\tfc\tmw\tpmw\ttraj_cpx\tn_iters"
        << std::endl;

  if (!fs::is_directory(out_dir) || !fs::exists(out_dir)) {
    fs::create_directory(out_dir);
    Log() << out_dir << " directory created " << std::endl;
  }

  // load checkpoint
  int ckpt_i = 0;
  int ckpt_j = -1;
  std::string ckpt_fn = raw_fn + ".ckpt";
  std::ifstream ckpt_file(ckpt_fn);
  if (!ckpt_file.is_open()) {
    Error() << "Warning: cannot open checkpoint file: " << ckpt_fn << std::endl;
  } else {
    ckpt_file >> ckpt_i >> ckpt_j;
    Log() << "Checkpoint loaded: " << ckpt_i << " " << ckpt_j << std::endl;
  }
  ckpt_j++;
  if (ckpt_j >= testcases.size()) {
    ckpt_i++;
    ckpt_j = 0;
  }

  psgc::PassiveGripper psg;
  psgs::Deserialize(psg, psg_fn);
  psgm::CostSettings org_cost_settings = psg.GetCostSettings();
  std::vector<psgm::ContactPoint> org_cps = psg.GetContactPoints();


  auto Process = [&](const psgm::CostSettings& cost_settings, int iter) {
    Log() << "> Optimizing for:\n" << cost_settings << std::endl;
    psg.SetCostSettings(cost_settings);
    psg.reinit_trajectory = true;
    psg.SetContactPoints(org_cps);

    psgc::Optimizer optimizer;
    auto start_time = std::chrono::high_resolution_clock::now();
    optimizer.Optimize(psg);
    optimizer.Wait();
    auto stop_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        stop_time - start_time);
    Log() << "> Optimization took " << duration.count() << " ms." << std::endl;

    psgm::GripperParams params = optimizer.GetCurrentParams();
    psg.SetParams(params);
    bool success = psg.GetMinDist() >= -1e-5;

    std::string out_fn =
        out_raw_fn + FormatOutput(cost_settings, iter) + ".params";
    psgs::Serialize(params, out_fn);
    Log() << ">> Params written to " << out_fn << std::endl;

    res_f << wopath_fn << '\t' << cost_settings.n_trajectory_steps << '\t'
          << cost_settings.geodesic_contrib << '\t'
          << cost_settings.inner_dis_contrib << '\t'
          << cost_settings.gripper_energy << '\t' << cost_settings.traj_energy
          << '\t' << success << '\t' << psg.GetMinDist() << '\t'
          << duration.count() << '\t' << psg.GetCost() << '\t'
          << psg.GetIsForceClosure() << '\t' << psg.GetMinWrench() << '\t'
          << psg.GetPartialMinWrench() << '\t'
          << psgc::GetTrajectoryComplexity(psg.GetTrajectory()) << '\t'
          << optimizer.GetIters() << std::endl;

    if (hook_set) {
      bp::ipstream out;
      bp::child c(hook_str,
                  bp::std_out > out,
                  wopath_fn,
                  std::to_string(cost_settings.n_trajectory_steps),
                  ToString(cost_settings.geodesic_contrib),
                  ToString(cost_settings.inner_dis_contrib),
                  ToString(cost_settings.gripper_energy),
                  ToString(cost_settings.traj_energy),
                  psg::kBoolStr[success],
                  ToString(psg.GetMinDist()),
                  std::to_string(duration.count()),
                  ToString(psg.GetCost()),
                  psg::kBoolStr[psg.GetIsForceClosure()],
                  ToString(psg.GetMinWrench()),
                  ToString(psg.GetPartialMinWrench()),
                  ToString(psgc::GetTrajectoryComplexity(psg.GetTrajectory())),
                  std::to_string(optimizer.GetIters()));
      char a[1024];
      auto& out_s = Log() << "[child proc]" << std::endl;
      while (out.read(a, 1024)) {
        int read = out.gcount();
        for (int i = 0; i < read; i++) {
          out_s << a[i];
        }
      }
      out_s << std::endl;
      c.wait();
    }
  };

  for (int i = ckpt_i; i < iters; i++) {
    for (int j = ckpt_j; j < (int)testcases.size(); j++) {
      const auto& testcase = testcases[j];
      psgm::CostSettings cost_settings = org_cost_settings;
      cost_settings.n_finger_steps = testcase.d_subdivision;
      cost_settings.n_trajectory_steps = testcase.d_subdivision;
      cost_settings.geodesic_contrib = testcase.geodesic_contrib;
      cost_settings.inner_dis_contrib = testcase.inner_dis_contrib;
      cost_settings.traj_energy = testcase.traj_energy;
      cost_settings.gripper_energy = testcase.gripper_energy;
      Process(cost_settings, i);

      // save checkpoint
      std::ofstream ckpt_file(ckpt_fn);
      if (!ckpt_file.is_open()) {
        Error() << "Warning: cannot open checkpoint file: " << ckpt_fn
                << std::endl;
      } else {
        ckpt_file << i << " " << j << std::endl;
        Log() << "Ckeckpoint updated: " << i << " " << j << std::endl;
      }
    }
    ckpt_j = 0;
  }

  return 0;
}
