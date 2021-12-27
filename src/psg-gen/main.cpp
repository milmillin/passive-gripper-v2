#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <igl/readSTL.h>
#include <Eigen/Core>

#include "../core/PassiveGripper.h"
#include "../core/Initialization.h"
#include "../utils.h"
#include "../core/models/SettingsOverrider.h"

int main(int argc, char** argv) {
  if (argc < 3) {
    Error() << "input .stl file and output .psg file required" << std::endl;
    return 1;
  }

  std::string stl_fn = argv[1];
  std::ifstream stl_f(stl_fn, std::ios::in | std::ios::binary);
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  Eigen::MatrixXd N;
  igl::readSTL(stl_f, V, F, N);

  std::string psg_fn = argv[2];
  std::ofstream psg_f(psg_fn, std::ios::out | std::ios::binary);
  if (!psg_f.is_open()) {
    Error() << "Cannot open " << psg_fn << std::endl;
    return 1;
  }

  psg::core::PassiveGripper psg;
  Eigen::MatrixXd SV;
  Eigen::Affine3d trans;
  psg::core::InitializeMeshPosition(V, SV, trans);
  psg.SetMesh(SV, F);

  if (argc >= 4) {
    std::string stgo_fn = argv[3];  
    psg::core::models::SettingsOverrider stgo;
    stgo.Load(stgo_fn);
    stgo.Apply(psg);
  }

  psg.Serialize(psg_f);
  Log() << "PSG written to: " << psg_fn << std::endl;
  return 0;
}
