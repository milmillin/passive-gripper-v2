#pragma once

#include <Eigen/Core>

namespace psg {
namespace core {
namespace models {

struct ContactPoint {
  Eigen::Vector3d position;
  Eigen::Vector3d normal;  // pointing out of mesh
  int fid;
};

}  // namespace models
}  // namespace core
}  // namespace psg