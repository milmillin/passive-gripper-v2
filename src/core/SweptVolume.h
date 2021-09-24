#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace psg {

// Compute Swept Volume of the box minus swept volume of mesh and floor
// V, F        : Mesh
// Transformations  : List of transformations
// boxLB, boxUB: Lower and upper bound of the box
// floor       : y-coordinate of the floor
void NegativeSweptVolume(const Eigen::MatrixXd& V,
                         const Eigen::MatrixXi& F,
                         const std::vector<Eigen::Matrix4d>& Transformations,
                         const Eigen::Vector3d& boxLB,
                         const Eigen::Vector3d& boxUB,
                         const Eigen::Vector3d& floor,
                         const Eigen::Vector3d& floorN,
                         Eigen::MatrixXi& out_CI,
                         Eigen::MatrixXd& out_CV,
                         Eigen::VectorXd& out_CS,
                         const double eps = 0.005,
                         const int num_seeds = 100);

void SweptVolume(const Eigen::MatrixXd& V,
                 const Eigen::MatrixXi& F,
                 const std::vector<Eigen::Matrix4d>& Transformations,
                 Eigen::MatrixXi& out_CI,
                 Eigen::MatrixXd& out_CV,
                 Eigen::VectorXd& out_CS,
                 const double eps = 0.005,
                 const int num_seeds = 100);

}