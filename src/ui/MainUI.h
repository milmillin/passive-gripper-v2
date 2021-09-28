#pragma once

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuizmoPlugin.h>

#include "ViewModel.h"

namespace psg {

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
  inline igl::opengl::ViewerData& GetLayer(ViewModel::Layers layer) {
    return viewer->data_list[(int)layer];
  }

  ViewModel vm_;

  enum class Tools {
    kNone,
    kContactPoint,
    kMeshPosition,
    kRobot,
    kOptimization
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
  void DrawViewPanel();
  void DrawGuizmoOptionPanel();

  // Draw Component
  void DrawLayerOptions(ViewModel::Layers layer, const char* id);
  void DrawToolButton(const char* label, Tools thisTool, float width);

  // Button Callback
  void OnLoadMeshClicked();

  // Layer Invalidation
  void OnLayerInvalidated(ViewModel::Layers layer);
  void OnMeshInvalidated();
  void OnCenterOfMassInvalidated();
  void OnContactPointsInvalidated();
  void OnFingersInvalidated();
};

}  // namespace psg