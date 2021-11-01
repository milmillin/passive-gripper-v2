#pragma once

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuizmoPlugin.h>

#include "Layer.h"
#include "ViewModel.h"
#include "../core/Optimizer.h"
#include "../core/models/ContactPointMetric.h"

namespace psg {
namespace ui {

class MainUI : public igl::opengl::glfw::imgui::ImGuiMenu {
 public:
  MainUI();
  void init(igl::opengl::glfw::Viewer* _viewer) override;
  inline bool pre_draw() override;
  inline bool post_draw() override;
  bool mouse_down(int button, int modifier);

  // UI Menu
  void draw_viewer_window() override;
  void draw_viewer_menu() override;

 private:
  inline igl::opengl::ViewerData& GetLayer(Layer layer) {
    return viewer->data_list[(int)layer];
  }

  ViewModel vm_;

  enum class Tools {
    kNone,
    kContactPoint,
    kMeshPosition,
    kRobot,
    kOptimization,
    kTopoOpt
  };
  Tools selected_tools_ = Tools::kNone;

  // Load Mesh Options
  bool is_millimeter_ = false;
  bool is_swap_yz_ = false;

  // Draw Panel
  void DrawMetricPanel();
  void DrawToolPanel();
  void DrawContactPointPanel();
  void DrawTransformPanel();
  void DrawRobotPanel();
  void DrawOptimizationPanel();
  void DrawTopoOptPanel();
  void DrawViewPanel();
  void DrawGuizmoOptionPanel();
  void DrawOptimizationStatusPanel();

  // Draw Component
  void DrawLayerOptions(Layer layer, const char* id);
  void DrawToolButton(const char* label, Tools thisTool, float width);

  // Button Callback
  void OnLoadMeshClicked();
  void OnLoadPSGClicked();
  void OnSavePSGClicked();
  void OnAlignCameraCenter();
  void OnExportContactPointCandidates();

  // Layer Invalidation
  void OnLayerInvalidated(Layer layer);
  void OnMeshInvalidated();
  void OnCenterOfMassInvalidated();
  void OnContactPointsInvalidated();
  void OnFingersInvalidated();
  void OnRobotInvalidated();
  void OnTrajectoryInvalidated();
  void OnSweptSurfaceInvalidated();
  void OnGripperBoundInvalidated();
  void OnNegVolInvalidated();

  // Contact Point Candidates
  size_t cp_num_seeds = 1000;
  size_t cp_num_candidates = 10000;
  std::vector<ContactPointMetric> contact_point_candidates_;
  size_t cp_export_size = 100;

  // Keyframe
  size_t selected_keyframe_index_ = -1;

  // Transform
  Eigen::Vector3d mesh_translate_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d mesh_rotation_ = Eigen::Vector3d::Zero();

  // Optimization
  Optimizer optimizer_;

  // ImGuizmo
  ImGuizmo::OPERATION imguizmo_operation = ImGuizmo::TRANSLATE;
  inline bool IsGuizmoVisible();
  Eigen::Affine3d GetGuizmoTransform() const;
  void SetGuizmoTransform(const Eigen::Affine3d& trans);
  bool imguizmo_pre_draw();
  bool imguizmo_post_draw();
};

}  // namespace ui
}  // namespace psg