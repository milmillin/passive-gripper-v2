#pragma once

#include <Eigen/Core>
#include <functional>
#include <vector>

#include "../core/models/ContactPoint.h"
#include "../core/models/Gripper.h"
#include "../core/models/MeshDependentResource.h"

namespace psg {

using namespace models;

class ViewModel {
 public:
  enum class Layers : int {
    kMesh = 0,
    kContactPoints,
    kFingers,
    kTrajectory,
    kMax
  };
  typedef std::function<void(Layers)> LayerInvalidatedDelegate;

  ViewModel();

  inline void RegisterLayerInvalidatedDelegate(
      const LayerInvalidatedDelegate& d) {
    LayerInvalidated_ = d;
    ForceInvalidateAll();
  }

  // Mesh
  void SetMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

  // Contact Point
  void AddContactPoint(const ContactPoint& contact_point);
  void RemoveContactPoint(size_t index);
  void ClearContactPoint();
  inline const std::vector<ContactPoint>& GetContactPoints() const {
    return gripper_.contact_points;
  }

  // Trajectory
  void AddKeyframe(const Pose& pose);
  void EditKeyframe(size_t index, const Pose& pose);
  void RemoveKeyframe(size_t index);
  void ClearKeyframe();
  inline const Trajectory& GetTrajectory() const {
    return gripper_.params.trajectory;
  }

  // Settings
  void SetSettings(const Settings& settings);
  inline const Settings& GetSettings() const { return gripper_.settings; }

  // Quality Metric
  inline bool GetIsForceClosure() const { return is_force_closure_; }
  inline bool GetIsPartialClosure() const { return is_partial_closure_; }
  inline double GetMinWrench() const { return min_wrench_; }
  inline double GetPartialMinWrench() const { return partial_min_wrench_; }

  // Cost
  inline double GetCost() const { return cost_; }

  bool reinit_trajectory = true;

 private:
  Gripper gripper_;
  MeshDependentResource mdr_;
  LayerInvalidatedDelegate LayerInvalidated_;
  std::vector<ContactPoint> contact_cones_;

  bool is_force_closure_;
  bool is_partial_closure_;
  double min_wrench_;
  double partial_min_wrench_;
  double cost_;

  // state dependency
  bool mesh_changed_ = false;
  bool settings_changed_ = false;
  bool contact_changed_ = false;
  bool finger_changed_ = false;
  bool trajectory_changed_ = false;
  bool quality_changed_ = false;
  bool cost_changed_ = false;

  void Invalidate();
  void ForceInvalidateAll();
  void InvokeLayerInvalidated(Layers layer);

  // To be called by Invalidate()
  void InvalidateMesh();
  void InvalidateSettings();
  void InvalidateContact();
  void InvalidateFinger();
  void InvalidateTrajectory();
  void InvalidateQuality();
  void InvalidateCost();

};

}  // namespace psg