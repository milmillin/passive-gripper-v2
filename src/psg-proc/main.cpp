#include <omp.h>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <boost/process.hpp>

#include <igl/writeSTL.h>
#include "../Constants.h"
#include "../batch/Result.h"
#include "../core/GeometryUtils.h"
#include "../core/Optimizer.h"
#include "../core/QualityMetric.h"
#include "../core/SweptVolume.h"
#include "../core/TopoOpt.h"
#include "../core/models/SettingsOverrider.h"
#include "../utils.h"

namespace fs = std::filesystem;
namespace bp = boost::process;

void Usage(char* argv0) {
  Error() << "Usage: " << argv0
          << " psg [-s stgo] [--refine bin out-stl] [--gen-tpd out-tpd] "
             "[--dump-traj out-traj-csv]"
          << std::endl;
}

int main(int argc, char** argv) {
  Log() << "Num threads: " << omp_get_max_threads() << std::endl;
  if (argc < 2) {
    Usage(argv[0]);
    return 1;
  }

  // psg_fn
  std::string psg_fn = argv[1];
  size_t lastdot = psg_fn.rfind('.');
  std::string raw_fn = psg_fn.substr(0, lastdot);
  std::string ckpt_fn = raw_fn + ".ckpt";

  size_t lastslash = std::min(raw_fn.rfind('\\'), raw_fn.rfind('/'));
  std::string wopath_fn = raw_fn;
  if (lastslash != std::string::npos) wopath_fn = raw_fn.substr(lastslash + 1);

  // -s
  bool stgo_set = false;
  std::string stgo_fn;

  // --opt outpsg
  bool opt_set = false;
  std::string out_psg_fn;

  // --opt-hook hook
  bool opt_hook_set = false;
  std::string hook;

  // --refine
  bool refine_set = false;
  std::string bin_fn;
  std::string out_stl_fn;

  // --gen-tpd
  bool gen_tpd_set = false;
  std::string out_tpd_fn;

  // --dump-traj
  bool dump_traj_set = false;
  std::string out_traj_csv_fn;

  // --dump-viz
  bool dump_viz = false;

  for (int i = 2; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "-s") {
      stgo_set = true;
      stgo_fn = argv[i + 1];
      i++;
    } else if (arg == "--opt") {
      opt_set = true;
      out_psg_fn = argv[i + 1];
      i++;
    } else if (arg == "--opt-hook") {
      opt_hook_set = true;
      hook = argv[i + 1];
      i++;
    } else if (arg == "--refine") {
      refine_set = true;
      bin_fn = argv[i + 1];
      out_stl_fn = argv[i + 2];
      i += 2;
    } else if (arg == "--gen-tpd") {
      gen_tpd_set = true;
      out_tpd_fn = argv[i + 1];
      i++;
    } else if (arg == "--dump-traj") {
      dump_traj_set = true;
      out_traj_csv_fn = argv[i + 1];
      i++;
    } else if (arg == "--dump-viz") {
      dump_viz = true;
    } else {
      Error() << "Unknown option " << arg << std::endl;
    }
  }

  psg::core::PassiveGripper psg;
  std::ifstream psg_file(psg_fn, std::ios::in | std::ios::binary);
  if (!psg_file.is_open()) {
    throw std::invalid_argument("> Cannot open psg file " + psg_fn);
  }
  psg.Deserialize(psg_file);
  Log() << "> Loaded " << psg_fn << std::endl;

  if (stgo_set) {
    psg::core::SettingsOverrider stgo;
    stgo.Load(stgo_fn);
    stgo.Apply(psg);
  }
  if (opt_set) {
    psg::core::Optimizer optimizer;
    Log() << "> Optimizing" << std::endl;
    Log() << psg.GetOptSettings() << std::endl;
    Log() << psg.GetTopoOptSettings() << std::endl;
    psg.reinit_trajectory = true;
    auto cp = psg.GetContactPoints();
    psg.SetContactPoints(cp);
    if (cp.size() == 0) {
      Error() << "No contact points in PSG" << std::endl;
      goto opt_done;
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    optimizer.Optimize(psg);
    optimizer.Wait();
    auto stop_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        stop_time - start_time);
    Log() << "> Optimization took " << duration.count() << " ms." << std::endl;

    psg.SetParams(optimizer.GetCurrentParams());
    bool failed = psg.GetMinDist() < -1e-5;

    Result r{wopath_fn,
             0,
             wopath_fn,
             failed,
             psg.GetIsForceClosure(),
             psg.GetIsPartialClosure(),
             psg.GetMinWrench(),
             psg.GetPartialMinWrench(),
             psg.GetCost(),
             psg.GetMinDist(),
             psg.GetIntersecting(),
             psg::core::GetTrajectoryComplexity(
                 psg.GetTrajectory()),  // volume field
             -1,
             duration.count()};
    Out() << r << std::endl;

    std::string psg_out_fn = out_psg_fn;
    {
      std::ofstream psg_out_f(psg_out_fn, std::ios::out | std::ios::binary);
      if (!psg_out_f.is_open()) {
        Error() << "> Cannot open out file " << psg_out_fn << std::endl;
        Error() << ">> Skipping" << std::endl;
        goto opt_done;
      }
      psg.Serialize(psg_out_f);
      Log() << "> Done: Optimized gripper written to " << psg_out_fn
            << std::endl;
    }
    if (opt_hook_set) {
      bp::ipstream out;
      bp::child c(hook,
                  bp::std_out > out,
                  r.out_fn,
                  psg::kBoolStr[!r.failed],
                  r.name,
                  std::to_string(r.cp_idx),
                  psg::kBoolStr[r.force_closure],
                  psg::kBoolStr[r.partial_force_closure],
                  ToString(r.min_wrench),
                  ToString(r.partial_min_wrench),
                  ToString(r.cost),
                  ToString(r.min_dist),
                  psg::kBoolStr[r.intersecting],
                  ToString(r.volume),
                  ToString(r.pi_volume),
                  std::to_string(r.duration),
                  ".");
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
  }
opt_done:
  if (refine_set) {
    Log() << "> Refining mesh.." << std::endl;
    Eigen::MatrixXd bin_V;
    Eigen::MatrixXi bin_F;
    psg::core::LoadResultBin(psg, bin_fn, bin_V, bin_F);
    Log() << "> " << bin_fn << " loaded" << std::endl;

    Eigen::MatrixXd neg_V;
    Eigen::MatrixXi neg_F;
    psg::core::NegativeSweptVolumePSG(psg, neg_V, neg_F);
    Log() << "> Negative volume computed" << std::endl;

    Eigen::MatrixXd gripper_V;
    Eigen::MatrixXi gripper_F;
    psg::core::RefineGripper(
        psg, bin_V, bin_F, neg_V, neg_F, gripper_V, gripper_F);
    Log() << "> Gripper refined" << std::endl;

    igl::writeSTL(out_stl_fn, gripper_V, gripper_F, igl::FileEncoding::Binary);
    Log() << "> Gripper STL written to " << out_stl_fn << std::endl;
  }
  if (gen_tpd_set) {
    Log() << "> Generating TPD file" << std::endl;
    std::string tpd_out_fn = out_tpd_fn + ".tpd";
    psg.InitGripperBound();
    Eigen::MatrixXd neg_V;
    Eigen::MatrixXi neg_F;
    NegativeSweptVolumePSG(psg, neg_V, neg_F);
    GenerateTopyConfig(psg, neg_V, neg_F, tpd_out_fn, nullptr);
    Log() << ">> Done: TPD file written to " << tpd_out_fn << std::endl;
  }
  if (dump_traj_set) {
    Log() << "> Dumping trajectory" << std::endl;
    {
      std::ofstream traj_csv_file(out_traj_csv_fn);
      const psg::Trajectory& traj = psg.GetTrajectory();
      for (int i = traj.size() - 1; i >= 0; i--) {
        for (int j = 0; j < psg::kNumDOFs; j++) {
          traj_csv_file << std::setprecision(15) << traj[i](j) << ",";
        }
        traj_csv_file << std::endl;
      }
    }
    Log() << ">> Done: Trajectory dumped to " << out_traj_csv_fn << std::endl;
  }
  if (dump_viz) {
    Log() << "> Dumping Viz" << std::endl;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    psg.GetMesh(V, F);
    igl::writeSTL(raw_fn + "_mesh.stl", V, F, igl::FileEncoding::Binary);
    Log() << ">> Mesh Dumped to " << raw_fn + "_mesh.stl" << std::endl;



    std::vector<Eigen::MatrixXd> cp_V;
    std::vector<Eigen::MatrixXi> cp_F;
    size_t n_cp_V = 0;
    size_t n_cp_F = 0;
    auto cps = psg.GetContactPoints();
    // psg.SetContactPoints(cps);
    for (size_t i = 0; i < cps.size(); i++) {
      Eigen::MatrixXd V_;
      Eigen::MatrixXi F_;
      psg::core::CreateCone(
          cps[i].position, cps[i].normal, 0.0075, 0.02, 8, V_, F_);
      cp_V.push_back(V_);
      cp_F.push_back(F_);
      n_cp_V += V_.rows();
      n_cp_F += F_.rows();
    }
    Eigen::MatrixXd VVV(n_cp_V, 3);
    Eigen::MatrixXi FFF(n_cp_F, 3);
    size_t cur_cp_V = 0;
    size_t cur_cp_F = 0;
    for (int i = 0; i < cp_V.size(); i++) {
      VVV.block(cur_cp_V, 0, cp_V[i].rows(), 3) = cp_V[i];
      FFF.block(cur_cp_F, 0, cp_F[i].rows(), 3) =
          cp_F[i].array() + (int)cur_cp_V;
      cur_cp_V += cp_V[i].rows();
      cur_cp_F += cp_F[i].rows();
    }
    igl::writeSTL(raw_fn + "_cp.stl", VVV, FFF, igl::FileEncoding::Binary);
    Log() << ">> CP Dumped to " << raw_fn + "_cp.stl" << std::endl;

    psg::Fingers finger = psg.GetFingers();
    std::vector<Eigen::MatrixXd> finger_V;
    std::vector<Eigen::MatrixXi> finger_F;
    for (size_t i = 0; i < finger.size(); i++) {
      for (size_t j = 0; j < finger[i].rows(); j++) {
        Eigen::MatrixXd V_;
        Eigen::MatrixXi F_;
        psg::core::CreateSpheres(finger[i].row(j), 0.005, 10, V_, F_);
        finger_V.push_back(V_);
        finger_F.push_back(F_);
        if (j > 0) {
          psg::core::CreateCylinder(
              finger[i].row(j - 1), finger[i].row(j), 0.002, 5, V_, F_);
          finger_V.push_back(V_);
          finger_F.push_back(F_);
        }
      }
    }
    size_t n_V = 0;
    size_t n_F = 0;
    for (size_t i = 0; i < finger_V.size(); i++) {
      n_V += finger_V[i].rows();
      n_F += finger_F[i].rows();
    }
    Eigen::MatrixXd V2(n_V, 3);
    Eigen::MatrixXi F2(n_F, 3);
    size_t cur_V = 0;
    size_t cur_F = 0;
    for (size_t i = 0; i < finger_V.size(); i++) {
      V2.block(cur_V, 0, finger_V[i].rows(), 3) = finger_V[i];
      F2.block(cur_F, 0, finger_F[i].rows(), 3) =
          finger_F[i].array() + (int)cur_V;
      cur_V += finger_V[i].rows();
      cur_F += finger_F[i].rows();
    }
    igl::writeSTL(raw_fn + "_fingers.stl", V2, F2, igl::FileEncoding::Binary);
    Log() << ">> Fingers Dumped to " << raw_fn + "_fingers.stl" << std::endl;
  }

  Log() << "All jobs done" << std::endl;
  return 0;
}
