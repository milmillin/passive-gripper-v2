#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "Constants.h"
#include "models/ContactPoint.h"

namespace psg {

using namespace models;

// points: N by dim matrix
bool ComputeConvexHull(const Eigen::MatrixXd& points,
                       std::vector<size_t>& out_hull_indices,
                       std::vector<std::vector<size_t>>& out_facets);

// Computes binormal B and tangential T given N.
// Assumes N is normalized
void GetPerp(const Eigen::Vector3d& N, Eigen::Vector3d& B, Eigen::Vector3d& T);

// Returns the double the area of triable ABC
double DoubleTriangleArea(const Eigen::Vector3d& A,
                          const Eigen::Vector3d& B,
                          const Eigen::Vector3d& C);

// a, b: angles in radians
// return difference between two angles wrapping considered
// (can be positive or negative)
double AngularDistance(double a, double b);

double SumSquaredAngularDistance(const Pose& a, const Pose& b);

// a, b: list of angles in radians
// Fix angle in b so that it takes the least distance
Pose FixAngles(const Pose& a, const Pose& b);

void FixTrajectory(Trajectory& t);

std::vector<ContactPoint> GenerateContactCone(const ContactPoint& contact_point,
                                              size_t cone_res,
                                              double friction);

}  // namespace psg