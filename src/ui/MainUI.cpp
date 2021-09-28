#include "MainUI.h"

#include <igl/remove_duplicate_vertices.h>
#include <igl/unproject_onto_mesh.h>
#include "../core/GeometryUtils.h"
#include "../core/robots/Robots.h"

namespace psg {

MainUI::MainUI() {
}

void MainUI::init(igl::opengl::glfw::Viewer* viewer_) {
  using namespace std::placeholders;
  igl::opengl::glfw::imgui::ImGuiMenu::init(viewer_);

  for (int i = 1; i < (int)ViewModel::Layers::kMax; i++) {
    viewer->data_list.emplace_back();
    viewer->data_list.back().id = i;
  }
  viewer->next_data_id = (int)ViewModel::Layers::kMax;

  viewer->core().orthographic = true;
  viewer->core().is_animating = true;

  vm_.RegisterLayerInvalidatedDelegate(
      std::bind(&MainUI::OnLayerInvalidated, this, _1));

  auto& axisLayer = GetLayer(ViewModel::Layers::kAxis);
  axisLayer.set_edges(axis_V * 0.1, axis_E, Eigen::Matrix3d::Identity());
  axisLayer.line_width = 2;
}

inline bool MainUI::pre_draw() {
  return ImGuiMenu::pre_draw();
}

inline bool MainUI::post_draw() {
  return ImGuiMenu::post_draw();
}

bool MainUI::mouse_down(int button, int modifier) {
  if (ImGuiMenu::mouse_down(button, modifier)) return true;

  double x = viewer->current_mouse_x;
  double y = viewer->core().viewport(3) - viewer->current_mouse_y;

  if (modifier & IGL_MOD_CONTROL) {
    if (vm_.IsMeshLoaded() && selected_tools_ == Tools::kContactPoint) {
      const auto& meshLayer = GetLayer(ViewModel::Layers::kMesh);
      const auto& V = meshLayer.V;
      const auto& F = meshLayer.F;
      const auto& N = meshLayer.F_normals;

      int fid;
      Eigen::Vector3f bc;
      if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y),
                                   viewer->core().view,
                                   viewer->core().proj,
                                   viewer->core().viewport,
                                   V,
                                   F,
                                   fid,
                                   bc)) {
        Eigen::Vector3d a = V.row(F(fid, 0));
        Eigen::Vector3d b = V.row(F(fid, 1));
        Eigen::Vector3d c = V.row(F(fid, 2));
        vm_.AddContactPoint(
            ContactPoint{bc(0) * a + bc(1) * b + bc(2) * c, N.row(fid), fid});
        return true;
      }
    }
  }
  return false;
}

void MainUI::draw_viewer_window() {
  float menu_width = 320.f * menu_scaling();
  ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f),
                                      ImVec2(menu_width, -1.0f));
  bool _viewer_menu_visible = true;
  ImGui::Begin(
      "Toolbar",
      &_viewer_menu_visible,
      ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.4f);
  if (callback_draw_viewer_menu) {
    callback_draw_viewer_menu();
  } else {
    draw_viewer_menu();
  }
  ImGui::PopItemWidth();
  ImGui::End();
}

void MainUI::draw_viewer_menu() {
  float w = ImGui::GetContentRegionAvailWidth();
  float p = ImGui::GetStyle().FramePadding.x;
  if (ImGui::Button("Load Mesh", ImVec2(w, 0))) {
    OnLoadMeshClicked();
  }
  ImGui::Checkbox("Millimeter", &is_millimeter_);
  ImGui::Checkbox("Swap YZ", &is_swap_yz_);
  ImGui::Separator();
  if (vm_.IsMeshLoaded()) {
    DrawMetricPanel();
    DrawToolPanel();
    switch (selected_tools_) {
      case Tools::kContactPoint:
        DrawContactPointPanel();
        break;
      case Tools::kMeshPosition:
        DrawGuizmoOptionPanel();
        DrawTransformPanel();
        break;
      case Tools::kRobot:
        DrawGuizmoOptionPanel();
        DrawRobotPanel();
        break;
      case Tools::kOptimization:
        DrawOptimizationPanel();
        break;
      case Tools::kNone:
        break;
      default:
        break;
    }
  }
  DrawViewPanel();
}

void MainUI::DrawMetricPanel() {
  static const char* kFalseTrue[2] = {"False", "True"};
  if (ImGui::CollapsingHeader("Metric", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushID("Metric");
    ImGui::Text("Force Closure: %s", kFalseTrue[vm_.GetIsForceClosure()]);
    ImGui::Text("Partial Force Closure: %s",
                kFalseTrue[vm_.GetIsPartialClosure()]);
    ImGui::Text("Min Wrench: %.4e", vm_.GetMinWrench());
    ImGui::Text("Partial Min Wrench: %.4e", vm_.GetPartialMinWrench());
    ImGui::Text("Cost: %.4e", vm_.GetCost());
    ImGui::PopID();
  }
}

void MainUI::DrawToolPanel() {
  float w = ImGui::GetContentRegionAvailWidth();
  if (ImGui::CollapsingHeader("Tools", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushID("Tools");
    DrawToolButton("None", Tools::kNone, w);
    DrawToolButton("Contact Point", Tools::kContactPoint, w);
    DrawToolButton("Mesh Position", Tools::kMeshPosition, w);
    DrawToolButton("Trajectory", Tools::kRobot, w);
    DrawToolButton("Optimization", Tools::kOptimization, w);
    ImGui::PopID();
  }
}

void MainUI::DrawContactPointPanel() {
  static char buf_[32];
  float w = ImGui::GetContentRegionAvailWidth();
  if (ImGui::CollapsingHeader("Options", ImGuiTreeNodeFlags_DefaultOpen)) {
    FingerSettings settings = vm_.GetFingerSettings();
    bool update = false;
    update |=
        ImGui::InputDouble("Friction Coeff", &settings.friction, 0.1, 0.5);
    update |= ImGui::InputInt("Cone Resolution", (int*)&settings.cone_res);
    update |= ImGui::InputInt("Finger Joints", (int*)&settings.n_finger_joints);
    if (update) vm_.SetFingerSettings(settings);
  }
  if (ImGui::CollapsingHeader("Contact Points",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    size_t to_delete = -1;
    const auto& contact_points = vm_.GetContactPoints();
    for (size_t i = 0; i < contact_points.size(); i++) {
      snprintf(buf_, sizeof(buf_), "Delete C%llu", i);
      if (ImGui::Button(buf_, ImVec2(w, 0))) {
        to_delete = i;
      }
    }
    if (to_delete != -1) {
      vm_.RemoveContactPoint(to_delete);
    }
    ImGui::Separator();
    if (ImGui::Button("Delete All Contact Points", ImVec2(w, 0))) {
      vm_.ClearContactPoint();
    }
  }
}

void MainUI::DrawTransformPanel() {}

void MainUI::DrawRobotPanel() {}

void MainUI::DrawOptimizationPanel() {}

void MainUI::DrawViewPanel() {}

void MainUI::DrawGuizmoOptionPanel() {}

void MainUI::DrawLayerOptions(ViewModel::Layers layer, const char* id) {}

void MainUI::DrawToolButton(const char* label, Tools thisTool, float width) {
  bool disabled = thisTool == selected_tools_;
  if (disabled) {
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
  }
  if (ImGui::Button(label, ImVec2(width, 0))) {
    selected_tools_ = thisTool;
  }
  if (disabled) {
    ImGui::PopStyleVar();
    ImGui::PopItemFlag();
  }
}

void MainUI::OnLoadMeshClicked() {
  std::string filename = igl::file_dialog_open();
  if (filename.empty()) return;
  size_t last_dot = filename.rfind('.');
  if (last_dot == std::string::npos) {
    std::cerr << "Error: No file extension found in " << filename << std::endl;
    return;
  }

  std::string extension = filename.substr(last_dot + 1);

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  if (extension == "obj" || extension == "OBJ") {
    Eigen::MatrixXd corner_normals;
    Eigen::MatrixXi fNormIndices;

    Eigen::MatrixXd UV_V;
    Eigen::MatrixXi UV_F;

    if (!(igl::readOBJ(
            filename, V, UV_V, corner_normals, F, UV_F, fNormIndices))) {
      printf("Error: %s is not a recognized file type.\n", extension.c_str());
      return;
    }
  } else {
    if (!igl::read_triangle_mesh(filename, V, F)) {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n", extension.c_str());
      return;
    }
  }

  Eigen::MatrixXd SV;
  Eigen::VectorXd SVI;
  Eigen::VectorXd SVJ;
  igl::remove_duplicate_vertices(V, 0, SV, SVI, SVJ);
  Eigen::MatrixXi SF = F;
  for (size_t i = 0; i < SF.size(); i++) {
    SF(i) = SVJ(SF(i));
  }

  if (is_millimeter_) {
    SV /= 1000.;
  }
  if (is_swap_yz_) {
    SV.col(1).swap(SV.col(2));
    SF.col(1).swap(SF.col(2));
  }

  vm_.SetMesh(SV, SF);
}

void MainUI::OnLayerInvalidated(ViewModel::Layers layer) {
  switch (layer) {
    case ViewModel::Layers::kMesh:
      OnMeshInvalidated();
      break;
    case ViewModel::Layers::kCenterOfMass:
      OnCenterOfMassInvalidated();
      break;
    case ViewModel::Layers::kContactPoints:
      OnContactPointsInvalidated();
      break;
    case ViewModel::Layers::kFingers:
      OnFingersInvalidated();
      break;
  }
}

void MainUI::OnMeshInvalidated() {
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  vm_.GetMesh(V, F);
  auto& meshLayer = GetLayer(ViewModel::Layers::kMesh);
  meshLayer.clear();
  meshLayer.set_mesh(V, F);
  meshLayer.uniform_colors((colors::kGold * 0.3).transpose(),
                           (Eigen::Vector3d)colors::kGold.transpose(),
                           Eigen::Vector3d::Zero());
}

void MainUI::OnCenterOfMassInvalidated() {
  auto& comLayer = GetLayer(ViewModel::Layers::kCenterOfMass);
  comLayer.clear();
  comLayer.set_points(vm_.GetCenterOfMass().transpose(),
                     Eigen::RowVector3d(0.7, 0.2, 0));
}

void MainUI::OnContactPointsInvalidated() {
  auto& cpLayer = GetLayer(ViewModel::Layers::kContactPoints);
  cpLayer.clear();

  const auto& contact_points = vm_.GetContactPoints();
  const auto& contact_cones = vm_.GetContactCones();
  const auto& finger_settings = vm_.GetFingerSettings();

  size_t nContacts = contact_points.size();
  Eigen::MatrixXd V(nContacts * finger_settings.cone_res * 2, 3);
  Eigen::MatrixXi VE(nContacts * finger_settings.cone_res * 3, 2);
  Eigen::MatrixXd PL(nContacts, 3);
  std::vector<std::string> labels(nContacts);
  size_t tmp;
  size_t tmp2;
  for (size_t i = 0; i < contact_points.size(); i++) {
    PL.row(i) = contact_points[i].position + contact_points[i].normal * 0.01;
    labels[i] = "C" + std::to_string(i);
    for (size_t j = 0; j < finger_settings.cone_res; j++) {
      const auto& cp = contact_cones[i * finger_settings.cone_res + j];
      tmp = i * finger_settings.cone_res * 2 + j * 2;
      tmp2 = i * finger_settings.cone_res * 2 +
             ((j + 1) % finger_settings.cone_res) * 2;
      V.row(tmp) = cp.position + 0.02 * cp.normal;
      V.row(tmp + 1) = cp.position;  // + 0.02 * cp.normal;
      VE.row(i * finger_settings.cone_res * 3 + j * 3) =
          Eigen::RowVector2i(tmp, tmp + 1);
      VE.row(i * finger_settings.cone_res * 3 + j * 3 + 1) =
          Eigen::RowVector2i(tmp, tmp2);
      VE.row(i * finger_settings.cone_res * 3 + j * 3 + 2) =
          Eigen::RowVector2i(tmp + 1, tmp2 + 1);
    }
  }
  cpLayer.set_edges(V, VE, Eigen::RowVector3d(0.2, 0.5, 0.1));
  cpLayer.set_labels(PL, labels);
  cpLayer.line_width = 2;
  cpLayer.show_custom_labels = true;
  cpLayer.show_overlay = true;
}

void MainUI::OnFingersInvalidated() {
  auto& fingerLayer = GetLayer(ViewModel::Layers::kFingers);

  const auto& fingers = vm_.GetFingers();
  const auto& finger_settings = vm_.GetFingerSettings();
  const Pose& first = vm_.GetTrajectory().front();
  const Pose& current_pose = vm_.GetCurrentPose();

  Eigen::Affine3d finger_trans_inv = robots::Forward(first).inverse();

  size_t nFingers = fingers.size();
  Eigen::MatrixXd V(nFingers * finger_settings.n_finger_joints, 3);
  Eigen::MatrixXi E(nFingers * (finger_settings.n_finger_joints - 1), 2);

  Eigen::Affine3d curTrans = robots::Forward(current_pose) * finger_trans_inv;

  for (size_t i = 0; i < nFingers; i++) {
    V.block(i * finger_settings.n_finger_joints,
            0,
            finger_settings.n_finger_joints,
            3)
        .transpose() =
        curTrans * fingers[i].transpose().colwise().homogeneous();
    for (size_t j = 0; j < finger_settings.n_finger_joints - 1; j++) {
      E(i * (finger_settings.n_finger_joints - 1) + j, 0) =
          i * finger_settings.n_finger_joints + j;
      E(i * (finger_settings.n_finger_joints - 1) + j, 1) =
          i * finger_settings.n_finger_joints + j + 1;
    }
  }

  fingerLayer.set_points(V, Eigen::RowVector3d(0.8, 0.4, 0));
  fingerLayer.set_edges(V, E, colors::kRed);
  fingerLayer.line_width = 5;
  fingerLayer.point_size = 9;
}

}  // namespace psg