#include "DiscreteDistanceField.h"

#include "TopoOpt.h"

#include <Eigen/Core>
#include <climits>
#include <iostream>
#include <queue>
#include <vector>

namespace psg {
namespace core {

DiscreteDistanceField::DiscreteDistanceField(const Eigen::MatrixXd& V,
                                             const Eigen::MatrixXi& F,
                                             int units,
                                             Eigen::Vector3d base) {
  lower_bound = V.colwise().minCoeff().transpose();
  upper_bound = V.colwise().maxCoeff().transpose();
  lower_bound = lower_bound.cwiseMin(base);
  upper_bound = upper_bound.cwiseMax(base);

  double shortest_side = (upper_bound - lower_bound).minCoeff();
  resolution = shortest_side / units;

  auto margin = 2 * Eigen::Vector3d(resolution, resolution, resolution);
  lower_bound -= margin;
  upper_bound += margin;

  // voxels = free space
  auto voxels =
      GetForbiddenVoxels(V, F, lower_bound, upper_bound, resolution, size);
  std::cout << "Map size: " << size(0) << " " << size(1) << " " << size(2)
            << std::endl;

  distance.resize(size(0) * size(1) * size(2));
  for (double& item : distance)
    item = -1;

  for (const auto& voxel : voxels)
    getVoxel(voxel) = std::numeric_limits<double>::max() / 2.;

  std::queue<Eigen::Vector3i> queue;
  Eigen::Vector3i start = ((base - lower_bound) / resolution).cast<int>();
  queue.push(start);
  getVoxel(start) = 0;
  while (queue.size() > 0) {
    Eigen::Vector3i next = queue.front();
    queue.pop();

    double current_cost = getVoxel(next);
    // std::cout << current_cost << std::endl;

    for (int dx = -1; dx <= 1; dx++)
      for (int dy = -1; dy <= 1; dy++)
        for (int dz = -1; dz <= 1; dz++) {
          double cur_cost = current_cost + Eigen::Vector3i(dx, dy, dz).cast<double>().norm();
          Eigen::Vector3i next_coord = next + Eigen::Vector3i(dx, dy, dz);
          next_coord = next_coord.cwiseMax(Eigen::Vector3i(0, 0, 0));
          next_coord = next_coord.cwiseMin(size - Eigen::Vector3i(1, 1, 1));
          if (getVoxel(next_coord) != -1 && getVoxel(next_coord) > cur_cost) {
            getVoxel(next_coord) = cur_cost;
            queue.push(next_coord);
          }
        }
  }
}

}  // namespace core
}  // namespace psg
