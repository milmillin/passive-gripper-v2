#include "MainUI.h"

#include <igl/remove_duplicate_vertices.h>
#include <igl/unproject_onto_mesh.h>
#include <iostream>

#include "../core/CostFunctions.h"
#include "../core/GeometryUtils.h"
#include "../core/Initialization.h"
#include "../core/TopoOpt.h"
#include "../core/models/GripperSettings.h"
#include "../core/robots/Robots.h"
#include "../core/serialization/Serialization.h"
#include "Components.h"

using namespace psg::core;

namespace psg {
namespace ui {

MainUI::MainUI() {}

void MainUI::init(igl::opengl::glfw::Viewer* viewer_) {
  using namespace std::placeholders;
  igl::opengl::glfw::imgui::ImGuiMenu::init(viewer_);

  for (int i = 1; i < (int)Layer::kMax; i++) {
    viewer->data_list.emplace_back();
    viewer->data_list.back().id = i;
  }
  viewer->next_data_id = (int)Layer::kMax;

  viewer->core().orthographic = true;
  viewer->core().is_animating = true;

  vm_.RegisterInvalidatedDelegate(
      std::bind(&MainUI::OnLayerInvalidated, this, _1));

  auto& axisLayer = GetLayer(Layer::kAxis);
  axisLayer.set_edges(axis_V * 0.1, axis_E, Eigen::Matrix3d::Identity());
  axisLayer.line_width = 2;

  GetLayer(Layer::kSweptSurface).show_lines = false;
}

inline bool MainUI::pre_draw() {
  if (vm_.GetIsAnimating()) vm_.NextFrame();
  bool res = ImGuiMenu::pre_draw();
  imguizmo_pre_draw();
  return res;
}

inline bool MainUI::post_draw() {
  imguizmo_post_draw();
  return ImGuiMenu::post_draw();
}

bool MainUI::mouse_down(int button, int modifier) {
  if (ImGuiMenu::mouse_down(button, modifier)) return true;

  double x = viewer->current_mouse_x;
  double y = viewer->core().viewport(3) - viewer->current_mouse_y;

  if (modifier & IGL_MOD_CONTROL) {
    if (vm_.PSG().IsMeshLoaded() && selected_tools_ == Tools::kContactPoint) {
      const auto& meshLayer = GetLayer(Layer::kMesh);
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
        ContactPoint cp;
        cp.position = bc(0) * a + bc(1) * b + bc(2) * c;
        cp.normal = N.row(fid);
        cp.fid = fid;
        vm_.PSG().AddContactPoint(cp);
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
  ImGui::Checkbox("Reinit Trajectory", &vm_.PSG().reinit_trajectory);
  ImGui::Separator();
  if (ImGui::Button("Load PSG", ImVec2((w - p) / 2, 0))) {
    OnLoadPSGClicked();
  }
  ImGui::SameLine();
  if (MyButton("Save PSG", ImVec2((w - p) / 2, 0), !vm_.PSG().IsMeshLoaded())) {
    OnSavePSGClicked();
  }
  if (vm_.PSG().IsMeshLoaded()) {
    if (optimizer_.IsRunning() || optimizer_.IsResultAvailable()) {
      DrawOptimizationStatusPanel();
    }
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
      case Tools::kTopoOpt:
        DrawTopoOptPanel();
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
    ImGui::Text("Force Closure: %s", kFalseTrue[vm_.PSG().GetIsForceClosure()]);
    ImGui::Text("Partial Force Closure: %s",
                kFalseTrue[vm_.PSG().GetIsPartialClosure()]);
    ImGui::Text("Min Wrench: %.4e", vm_.PSG().GetMinWrench());
    ImGui::Text("Partial Min Wrench: %.4e", vm_.PSG().GetPartialMinWrench());
    ImGui::Text("Cost: %.4e", vm_.PSG().GetCost());
    ImGui::Text("Min Dist: %.4e", vm_.PSG().GetMinDist());
    ImGui::Text("Intersecting: %s", kFalseTrue[vm_.PSG().GetIntersecting()]);
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
    DrawToolButton("Topo Opt", Tools::kTopoOpt, w);
    ImGui::PopID();
  }
}

void MainUI::DrawContactPointPanel() {
  static char buf_[32];
  float w = ImGui::GetContentRegionAvailWidth();
  if (ImGui::CollapsingHeader("Options", ImGuiTreeNodeFlags_DefaultOpen)) {
    FingerSettings finger_settings = vm_.PSG().GetFingerSettings();
    ContactSettings contact_settings = vm_.PSG().GetContactSettings();
    bool contact_update = false;
    bool finger_update = false;
    contact_update |= ImGui::InputDouble(
        "Friction Coeff", &contact_settings.friction, 0.1, 0.5);
    contact_update |=
        ImGui::InputInt("Cone Resolution", (int*)&contact_settings.cone_res);
    finger_update |= ImGui::InputInt("Finger Joints",
                                     (int*)&finger_settings.n_finger_joints);
    if (finger_update) vm_.PSG().SetFingerSettings(finger_settings);
    if (contact_update) vm_.PSG().SetContactSettings(contact_settings);
  }
  if (ImGui::CollapsingHeader("Contact Points",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    size_t to_delete = -1;
    const auto& contact_points = vm_.PSG().GetContactPoints();
    for (size_t i = 0; i < contact_points.size(); i++) {
      snprintf(buf_, sizeof(buf_), "Delete C%llu", i);
      if (ImGui::Button(buf_, ImVec2(w, 0))) {
        to_delete = i;
      }
    }
    if (to_delete != -1) {
      vm_.PSG().RemoveContactPoint(to_delete);
    }
    ImGui::Separator();
    if (ImGui::Button("Delete All Contact Points", ImVec2(w, 0))) {
      vm_.PSG().ClearContactPoint();
    }
  }
  if (ImGui::CollapsingHeader("Candidates", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::InputInt("# Candidates", (int*)&cp_num_candidates, 1000);
    ImGui::InputInt("# Seeds", (int*)&cp_num_seeds, 1000);
    if (ImGui::Button("Generate Candidates", ImVec2(w, 0))) {
      contact_point_candidates_ =
          InitializeContactPoints(vm_.PSG().GetMDR(),
                                  vm_.PSG().GetSettings(),
                                  cp_num_candidates,
                                  cp_num_seeds);
    }
    ImGui::Separator();
    size_t k = std::min((size_t)10, contact_point_candidates_.size());
    ImGui::PushID("CPC");
    for (size_t i = 0; i < k; i++) {
      ImGui::PushID(std::to_string(i).c_str());
      if (ImGui::Button("Select")) {
        vm_.PSG().SetContactPoints(contact_point_candidates_[i].contact_points);
      }
      ImGui::SameLine();
      ImGui::Text("mw: %.4e, pmw: %.4e",
                  contact_point_candidates_[i].min_wrench,
                  contact_point_candidates_[i].partial_min_wrench);
      ImGui::PopID();
    }
    ImGui::PopID();
    ImGui::Separator();
    ImGui::InputInt("# Export", (int*)&cp_export_size, 10);
    ImGui::Text("Select filename as a format, e.g. bunny_%%03d.cp");
    if (ImGui::Button("Save Candidates", ImVec2(w, 0))) {
      OnExportContactPointCandidates();
    }
  }
}

void MainUI::DrawTransformPanel() {
  float w = ImGui::GetContentRegionAvailWidth();
  if (ImGui::CollapsingHeader("Transform", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Translation");
    MyInputDouble3("Translate", mesh_translate_.data());
    ImGui::Text("Rotation");
    MyInputDouble3Convert(
        "Rotation", mesh_rotation_.data(), kRadToDeg, 1, "%.1f");
    if (ImGui::Button("Apply", ImVec2(w, 0))) {
      Eigen::Affine3d mesh_trans = vm_.PSG().GetMeshTrans();
      Eigen::Affine3d trans =
          mesh_trans * Eigen::Translation3d(mesh_translate_) *
          Eigen::AngleAxisd(mesh_rotation_(1), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(mesh_rotation_(0), Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(mesh_rotation_(2), Eigen::Vector3d::UnitZ()) *
          mesh_trans.inverse();
      vm_.PSG().TransformMesh(trans);
    }
  }
}

void MainUI::DrawRobotPanel() {
  float w = ImGui::GetContentRegionAvailWidth();
  ImGui::PushID("RobotPanel");
  if (ImGui::CollapsingHeader("Forward Kinematics")) {
    bool updateFK = false;
    Pose p = vm_.GetCurrentPose();
    updateFK |= MyInputDouble3Convert("FK13", &p[0], kRadToDeg, 1, "%.1f");
    updateFK |= MyInputDouble3Convert("FK46", &p[3], kRadToDeg, 1, "%.1f");
    if (updateFK) {
      vm_.SetCurrentPose(p);
    }
  }
  if (ImGui::CollapsingHeader("Inverse Kinematics")) {
    ImGui::PushID("IK");
    bool updateIK = false;
    Eigen::Vector3d eff_pos = vm_.GetEffPosition();
    Eigen::Vector3d eff_ang = vm_.GetEffAngles();
    ImGui::Text("Translation");
    updateIK |= MyInputDouble3("IKPos", eff_pos.data());
    ImGui::Text("Rotation");
    updateIK |=
        MyInputDouble3Convert("IKRot", eff_ang.data(), kRadToDeg, 1, "%.1f");
    if (MyButton("Toggle Pose", ImVec2(w, 0), !vm_.CanTogglePose())) {
      vm_.TogglePose();
    }
    if (updateIK) {
      vm_.SetCurrentPose(eff_pos, eff_ang);
    }
    ImGui::PopID();  // IK
  }
  if (ImGui::CollapsingHeader("Keyframes", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushID("Keyframes");
    size_t to_delete = -1;
    const auto& trajectory = vm_.PSG().GetTrajectory();
    for (size_t i = 0; i < trajectory.size(); i++) {
      ImGui::PushID(std::to_string(i).c_str());

      if (ImGui::Button("X")) {
        to_delete = i;
      }
      ImGui::SameLine();
      if (ImGui::Button("Goto")) {
        vm_.AnimateTo(trajectory[i]);
      }
      ImGui::SameLine();
      if (ImGui::Button(">>")) {
        selected_keyframe_index_ = i;
        vm_.SetCurrentPose(trajectory[i]);
      }
      if (selected_keyframe_index_ == i) {
        ImGui::SameLine();
        ImGui::Text(">> ");
      }
      ImGui::SameLine();
      ImGui::Text("P%llu", i);

      ImGui::PopID();  // i
    }
    if (to_delete != -1) {
      vm_.PSG().RemoveKeyframe(to_delete);
    }
    if (ImGui::Button(">>")) {
      selected_keyframe_index_ = -1;
    }
    if (selected_keyframe_index_ == -1) {
      ImGui::SameLine();
      if (ImGui::Button(">> New Keyframe")) {
        vm_.PSG().reinit_trajectory = false;
        vm_.PSG().AddKeyframe(vm_.GetCurrentPose());
      }
    } else {
      ImGui::SameLine();
      ImGui::Text("New Keyframe");
    }
    ImGui::PopID();  // Keyframes
  }
  ImGui::PopID();  // RobotPanel
}

void MainUI::DrawOptimizationPanel() {
  ImGui::PushID("Optimization");
  float w = ImGui::GetContentRegionAvailWidth();
  if (ImGui::CollapsingHeader("Optimization", ImGuiTreeNodeFlags_DefaultOpen)) {
    OptSettings opt_settings = vm_.PSG().GetOptSettings();
    CostSettings cost_settings = vm_.PSG().GetCostSettings();
    bool opt_update = false;
    bool cost_update = false;
    ImGui::Text("Trajectory Wiggle (deg)");
    opt_update |= MyInputDouble3Convert(
        "traj13", &opt_settings.trajectory_wiggle[0], kRadToDeg, 1, "%.1f");
    opt_update |= MyInputDouble3Convert(
        "traj46", &opt_settings.trajectory_wiggle[3], kRadToDeg, 1, "%.1f");
    opt_update |= ImGui::InputDouble(
        "Finger Wiggle (m)", &opt_settings.finger_wiggle, 0.01);
    opt_update |=
        ImGui::InputDouble("Max Runtime (s)", &opt_settings.max_runtime, 1);
    opt_update |=
        ImGui::InputDouble("Tolerance", &opt_settings.tolerance, 0.0001);

    if (ImGui::BeginCombo("Algorithm",
                          labels::kAlgorithms[opt_settings.algorithm])) {
      for (int i = 0; i < NLOPT_NUM_ALGORITHMS; i++) {
        bool is_selected = (opt_settings.algorithm == i);
        if (ImGui::Selectable(labels::kAlgorithms[i], is_selected)) {
          opt_settings.algorithm = (nlopt_algorithm)i;
          opt_update = true;
        }
        if (is_selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    opt_update |=
        ImGui::InputInt("Population", (int*)&opt_settings.population, 1000);

    cost_update = ImGui::InputDouble("Floor", &cost_settings.floor, 0.001);
    if (opt_update) vm_.PSG().SetOptSettings(opt_settings);
    if (cost_update) vm_.PSG().SetCostSettings(cost_settings);
    if (ImGui::Button("Optimize", ImVec2(w, 0))) {
      optimizer_.Optimize(vm_.PSG());
    }
  }
  ImGui::PopID();
}

void MainUI::DrawTopoOptPanel() {
  ImGui::PushID("TopoOpt");
  float w = ImGui::GetContentRegionAvailWidth();
  if (ImGui::CollapsingHeader("Topo Opt", ImGuiTreeNodeFlags_DefaultOpen)) {
    TopoOptSettings settings = vm_.PSG().GetTopoOptSettings();
    bool update = false;
    ImGui::Text("Lower Bound");
    update |= MyInputDouble3("lb", settings.lower_bound.data(), 0.001, "%.3f");
    ImGui::Text("Lower Bound");
    update |= MyInputDouble3("ub", settings.upper_bound.data(), 0.001, "%.3f");
    update |= ImGui::InputDouble(
        "Topo Res (mm)", &settings.topo_res, 0.001, 0.001, "%.3f");
    update |= ImGui::InputDouble(
        "Neg Vol Res (mm)", &settings.neg_vol_res, 0.001, 0.001, "%.3f");
    update |= ImGui::InputDouble("Attachment Size (mm)",
                                 &settings.attachment_size,
                                 0.001,
                                 0.001,
                                 "%.3f");
    update |= ImGui::InputDouble("Contact Pt Size (mm)",
                                 &settings.contact_point_size,
                                 0.001,
                                 0.001,
                                 "%.3f");
    update |= ImGui::InputDouble(
        "Base Thickness (mm)", &settings.base_thickness, 0.001, 0.001, "%.3f");

    if (update) vm_.PSG().SetTopoOptSettings(settings);
    if (ImGui::Button("Init Gripper Bound", ImVec2(w, 0))) {
      vm_.PSG().InitGripperBound();
    }
    if (ImGui::Button("Compute Neg Vol", ImVec2(w, 0))) {
      vm_.ComputeNegativeVolume();
    }
    if (ImGui::Button("Generate Topy Config", ImVec2(w, 0))) {
      std::string filename = igl::file_dialog_save();
      if (!filename.empty()) {
        vm_.ComputeNegativeVolume();
        GenerateTopyConfig(
            vm_.PSG(), vm_.GetNegVolV(), vm_.GetNegVolF(), filename);
      }
    }
    if (ImGui::Button("Load Result Bin", ImVec2(w, 0))) {
      std::string filename = igl::file_dialog_open();
      if (!filename.empty()) {
        vm_.LoadResultBin(filename);
      }
    }
  }
  ImGui::PopID();
}

void MainUI::DrawViewPanel() {
  if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_None)) {
    ImGui::PushID("View");
    DrawLayerOptions(Layer::kMesh, "Mesh");
    DrawLayerOptions(Layer::kContactPoints, "Contact Points");
    DrawLayerOptions(Layer::kRobot, "Robot");
    DrawLayerOptions(Layer::kCenterOfMass, "Center Of Mass");
    DrawLayerOptions(Layer::kAxis, "Axis");
    DrawLayerOptions(Layer::kFingers, "Fingers");
    DrawLayerOptions(Layer::kTrajectory, "Trajectory");
    DrawLayerOptions(Layer::kSweptSurface, "Swept Finger");
    DrawLayerOptions(Layer::kGripperBound, "Gripper Bound");
    DrawLayerOptions(Layer::kNegVol, "Negative Volume");
    DrawLayerOptions(Layer::kGripper, "Gripper");
    ImGui::PopID();
  }
}

void MainUI::DrawGuizmoOptionPanel() {
  float w = ImGui::GetContentRegionAvailWidth();
  float p = ImGui::GetStyle().FramePadding.x;
  if (ImGui::CollapsingHeader("Manipulation Mode",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushID("Manipulation");
    if (MyButton("Trans",
                 ImVec2((w - p) / 2, 0),
                 imguizmo_operation == ImGuizmo::TRANSLATE)) {
      imguizmo_operation = ImGuizmo::TRANSLATE;
    }
    ImGui::SameLine(0, p);
    if (MyButton("Rot",
                 ImVec2((w - p) / 2, 0),
                 imguizmo_operation == ImGuizmo::ROTATE)) {
      imguizmo_operation = ImGuizmo::ROTATE;
    }
    ImGui::PopID();
  }
}

void MainUI::DrawOptimizationStatusPanel() {
  float w = ImGui::GetContentRegionAvailWidth();
  float p = ImGui::GetStyle().FramePadding.x;

  ImGui::PushID("OptProg");
  if (ImGui::CollapsingHeader("Optimization Progress",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    bool isRunning = optimizer_.IsRunning();
    auto start = optimizer_.GetStartTime();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    if (isRunning) {
      ImGui::Text("Optimizer Running...");
      ImGui::Text("Elapsed: %llds", duration.count());
    } else {
      ImGui::Text("Optimizer Stopped...");
    }
    ImGui::Text("Current Cost: %.4e", optimizer_.GetCurrentCost());
    if (isRunning) {
      if (ImGui::Button("Pause", ImVec2(w, 0))) {
        optimizer_.Cancel();
      }
    } else {
      if (ImGui::Button("Resume", ImVec2(w, 0))) {
        optimizer_.Resume();
      }
    }
    if (optimizer_.IsResultAvailable()) {
      if (ImGui::Button("Load Result", ImVec2(w, 0))) {
        vm_.PSG().SetParams(optimizer_.GetCurrentParams());
      }
    }
  }
  ImGui::PopID();
}

void MainUI::DrawLayerOptions(Layer layer, const char* id) {
  auto& data = GetLayer(layer);
  ImGui::PushID(id);
  ImGui::Checkbox("##V", (bool*)&data.is_visible);
  ImGui::SameLine();
  ImGui::Checkbox("##L", (bool*)&data.show_lines);
  ImGui::SameLine();
  ImGui::Checkbox("##F", (bool*)&data.show_faces);
  ImGui::SameLine();
  bool face_based = data.face_based;
  if (ImGui::Checkbox("##S", &face_based)) {
    data.set_face_based(face_based);
  }
  ImGui::SameLine();
  ImGui::Text(id);
  ImGui::PopID();
}

void MainUI::DrawToolButton(const char* label, Tools thisTool, float width) {
  bool disabled = thisTool == selected_tools_;
  if (MyButton(label, ImVec2(width, 0), disabled)) {
    selected_tools_ = thisTool;
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
  OnAlignCameraCenter();
  optimizer_.Reset();
}

void MainUI::OnLoadPSGClicked() {
  std::string filename = igl::file_dialog_open();

  // Sanity check
  if (filename.empty()) return;
  size_t last_dot = filename.rfind('.');
  if (last_dot == std::string::npos) {
    std::cerr << "Error: No file extension found in " << filename << std::endl;
    return;
  }
  std::string extension = filename.substr(last_dot + 1);
  if (extension != "psg") {
    std::cerr << "Error: Not a .psg file" << filename << std::endl;
    return;
  }

  std::ifstream myfile(filename, std::ios::in | std::ios::binary);
  psg::core::serialization::Deserialize(vm_.PSG(), myfile);
  std::cout << filename << " psg loaded!" << std::endl;

  // try load stl
  std::string stl_fn = filename.substr(0, last_dot) + ".stl";
  if (vm_.LoadGripper(stl_fn)) {
    std::cout << stl_fn << " gripper stl loaded!" << std::endl;
  }
  OnAlignCameraCenter();
}

void MainUI::OnSavePSGClicked() {
  std::string filename = igl::file_dialog_save();

  // Sanity check
  if (filename.empty()) return;
  size_t last_dot = filename.rfind('.');
  if (last_dot == std::string::npos) {
    std::cerr << "Error: No file extension found in " << filename << std::endl;
    return;
  }
  std::string extension = filename.substr(last_dot + 1);
  if (extension != "psg") {
    std::cerr << "Error: Not a .psg file" << filename << std::endl;
    return;
  }

  std::ofstream myfile(filename, std::ios::out | std::ios::binary);
  psg::core::serialization::Serialize(vm_.PSG(), myfile);
  std::cout << "PSG saved to " << filename << std::endl;
}

void MainUI::OnAlignCameraCenter() {
  const auto& mesh = GetLayer(Layer::kMesh);
  viewer->core().align_camera_center(mesh.V);
}

void MainUI::OnExportContactPointCandidates() {
  static thread_local char buf[64];
  std::string filename = igl::file_dialog_save();
  if (filename.empty()) return;
  size_t n_export = std::min(cp_export_size, contact_point_candidates_.size());
  for (size_t i = 0; i < n_export; i++) {
    snprintf(buf, 64, filename.c_str(), i);
    std::ofstream f(buf, std::ios::out | std::ios::binary);
    psg::core::serialization::Serialize(
        contact_point_candidates_[i].contact_points, f);
  }
}

void MainUI::OnLayerInvalidated(Layer layer) {
  switch (layer) {
    case Layer::kMesh:
      OnMeshInvalidated();
      break;
    case Layer::kCenterOfMass:
      OnCenterOfMassInvalidated();
      break;
    case Layer::kContactPoints:
      OnContactPointsInvalidated();
      break;
    case Layer::kFingers:
      OnFingersInvalidated();
      break;
    case Layer::kRobot:
      OnRobotInvalidated();
      break;
    case Layer::kTrajectory:
      OnTrajectoryInvalidated();
      break;
    case Layer::kSweptSurface:
      OnSweptSurfaceInvalidated();
      break;
    case Layer::kGripperBound:
      OnGripperBoundInvalidated();
      break;
    case Layer::kNegVol:
      OnNegVolInvalidated();
      break;
    case Layer::kGripper:
      OnGripperInvalidated();
      break;
  }
}

void MainUI::OnMeshInvalidated() {
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  vm_.PSG().GetMesh(V, F);
  auto& meshLayer = GetLayer(Layer::kMesh);
  meshLayer.clear();
  meshLayer.set_mesh(V, F);
  meshLayer.uniform_colors((colors::kGold * 0.3).transpose(),
                           (Eigen::Vector3d)colors::kGold.transpose(),
                           Eigen::Vector3d::Zero());
}

void MainUI::OnCenterOfMassInvalidated() {
  auto& comLayer = GetLayer(Layer::kCenterOfMass);
  comLayer.clear();
  comLayer.set_points(vm_.PSG().GetCenterOfMass().transpose(),
                      Eigen::RowVector3d(0.7, 0.2, 0));
}

void MainUI::OnContactPointsInvalidated() {
  auto& cpLayer = GetLayer(Layer::kContactPoints);
  cpLayer.clear();

  const auto& contact_points = vm_.PSG().GetContactPoints();
  const auto& contact_cones = vm_.PSG().GetContactCones();
  const auto& contact_settings = vm_.PSG().GetContactSettings();

  size_t nContacts = contact_points.size();
  Eigen::MatrixXd V(nContacts * contact_settings.cone_res * 2, 3);
  Eigen::MatrixXi VE(nContacts * contact_settings.cone_res * 3, 2);
  Eigen::MatrixXd PL(nContacts, 3);
  std::vector<std::string> labels(nContacts);
  size_t tmp;
  size_t tmp2;
  for (size_t i = 0; i < contact_points.size(); i++) {
    PL.row(i) = contact_points[i].position + contact_points[i].normal * 0.01;
    labels[i] = "C" + std::to_string(i);
    for (size_t j = 0; j < contact_settings.cone_res; j++) {
      const auto& cp = contact_cones[i * contact_settings.cone_res + j];
      tmp = i * contact_settings.cone_res * 2 + j * 2;
      tmp2 = i * contact_settings.cone_res * 2 +
             ((j + 1) % contact_settings.cone_res) * 2;
      V.row(tmp) = cp.position + 0.02 * cp.normal;
      V.row(tmp + 1) = cp.position;  // + 0.02 * cp.normal;
      VE.row(i * contact_settings.cone_res * 3 + j * 3) =
          Eigen::RowVector2i(tmp, tmp + 1);
      VE.row(i * contact_settings.cone_res * 3 + j * 3 + 1) =
          Eigen::RowVector2i(tmp, tmp2);
      VE.row(i * contact_settings.cone_res * 3 + j * 3 + 2) =
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
  auto& fingerLayer = GetLayer(Layer::kFingers);

  const auto& fingers = vm_.PSG().GetFingers();
  const auto& finger_settings = vm_.PSG().GetFingerSettings();
  const Pose& first = vm_.PSG().GetTrajectory().front();
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

void MainUI::OnRobotInvalidated() {
  auto& robotLayer = GetLayer(Layer::kRobot);
  robotLayer.clear();

  std::vector<Eigen::Affine3d> trans;
  robots::ForwardIntermediate(vm_.GetCurrentPose(), trans);

  Eigen::MatrixXd AV(6 * 4, 3);
  Eigen::MatrixXi AE(6 * 3, 2);

  Eigen::MatrixXd V(6 * 8, 3);
  Eigen::MatrixXi F(6 * 12, 3);

  for (size_t i = 0; i < 6; i++) {
    Eigen::Affine3d curTrans = trans[i];
    F.block<12, 3>(i * 12, 0) = cube_F.array() + (8 * i);
    V.block<8, 3>(i * 8, 0).transpose() =
        (curTrans * kLocalTrans[i] * cube_V.transpose());

    AV.block<4, 3>(i * 4, 0).transpose() =
        curTrans * (0.1 * axis_V).transpose();
    AE.block<3, 2>(i * 3, 0) = axis_E.array() + (4 * i);
  }

  robotLayer.set_mesh(V, F);
  robotLayer.set_edges(AV, AE, Eigen::Matrix3d::Identity().replicate<6, 1>());
  robotLayer.set_face_based(true);
  robotLayer.line_width = 2;
}

void MainUI::OnTrajectoryInvalidated() {
  const Trajectory& trajectory = vm_.PSG().GetTrajectory();
  size_t nKeyframe = trajectory.size();
  if (nKeyframe == 0) return;

  auto& trajectoryLayer = GetLayer(Layer::kTrajectory);

  Eigen::MatrixXd V(4 * nKeyframe, 3);
  Eigen::MatrixXi E(4 * nKeyframe - 1, 2);
  Eigen::MatrixXd C(4 * nKeyframe - 1, 3);

  for (size_t i = 0; i < nKeyframe; i++) {
    Eigen::Affine3d curTrans = robots::Forward(trajectory[i]);
    V.block<4, 3>(i * 4, 0).transpose() = curTrans * (0.1 * axis_V).transpose();
    E.block<3, 2>(i * 3, 0) = axis_E.array() + (4 * i);
    C.block<3, 3>(i * 3, 0) = Eigen::Matrix3d::Identity();
    if (i + 1 < nKeyframe) {
      E(3 * nKeyframe + i, 0) = i * 4;
      E(3 * nKeyframe + i, 1) = (i + 1) * 4;
      C.row(3 * nKeyframe + i) = Eigen::RowVector3d(0.7, 0, 0.8);
    }
  }

  trajectoryLayer.set_edges(V, E, C);
  trajectoryLayer.line_width = 2;
}

void MainUI::OnSweptSurfaceInvalidated() {
  const auto& fingers = vm_.PSG().GetFingers();
  const auto& trajectory = vm_.PSG().GetTrajectory();
  auto& layer = GetLayer(Layer::kSweptSurface);
  layer.clear();
  if (fingers.empty()) return;
  const size_t nFingers = fingers.size();
  static constexpr size_t nTrajectorySteps = 32;
  static constexpr size_t nFingerSteps = 32;
  static constexpr double trajectoryStep = 1. / nTrajectorySteps;
  static constexpr double fingerStep = 1. / nFingerSteps;

  const size_t nKeyframes = trajectory.size();
  const size_t nDOF = trajectory.front().size();
  const size_t nFingerJoints = fingers.front().rows();

  const size_t nFrames = (nKeyframes - 1) * nTrajectorySteps + 1;
  const size_t nEvalsPerFingerPerFrame = (nFingerJoints - 1) * nFingerSteps + 1;
  const size_t nEvals = nFrames * nFingers * nEvalsPerFingerPerFrame;
  Eigen::MatrixXd P(nEvals, 3);
  Eigen::MatrixXi PF(
      (nFrames - 1) * nFingers * (nEvalsPerFingerPerFrame - 1) * 2, 3);
  Eigen::VectorXd Cost(nEvals);

  Eigen::Affine3d finger_trans_inv =
      robots::Forward(trajectory.front()).inverse();

  {
    Eigen::Affine3d curTrans =
        robots::Forward(trajectory.front()) * finger_trans_inv;
    for (size_t i = 0; i < nFingers; i++) {
      const Eigen::MatrixXd& finger = fingers[i];
      Eigen::MatrixXd transformedFinger =
          (curTrans * finger.transpose().colwise().homogeneous()).transpose();
      P.row(i * nEvalsPerFingerPerFrame) = transformedFinger.row(0);
      for (size_t joint = 1; joint < nFingerJoints; joint++) {
        for (size_t kk = 1; kk <= nFingerSteps; kk++) {
          double fingerT = kk * fingerStep;
          Eigen::RowVector3d lerpedFinger =
              transformedFinger.row(joint - 1) * (1. - fingerT) +
              transformedFinger.row(joint) * fingerT;
          P.row(i * nEvalsPerFingerPerFrame + (joint - 1) * nFingerSteps + kk) =
              lerpedFinger;
        }
      }
    }
  }
  for (size_t iKf = 1; iKf < nKeyframes; iKf++) {
    Pose t_lerpedKeyframe;
    for (long long j = 1; j <= nTrajectorySteps; j++) {
      double trajectoryT = j * trajectoryStep;
      for (size_t k = 0; k < kNumDOFs; k++) {
        t_lerpedKeyframe[k] = trajectory[iKf - 1][k] * (1 - trajectoryT) +
                              trajectory[iKf][k] * trajectoryT;
      }
      Eigen::Affine3d curTrans =
          robots::Forward(t_lerpedKeyframe) * finger_trans_inv;
      for (size_t i = 0; i < nFingers; i++) {
        const Eigen::MatrixXd& finger = fingers[i];
        Eigen::MatrixXd transformedFinger =
            (curTrans * finger.transpose().colwise().homogeneous()).transpose();
        P.row(((iKf - 1) * nTrajectorySteps + j) * nFingers *
                  nEvalsPerFingerPerFrame +
              i * nEvalsPerFingerPerFrame) = transformedFinger.row(0);
        for (size_t joint = 1; joint < nFingerJoints; joint++) {
          for (size_t kk = 1; kk <= nFingerSteps; kk++) {
            double fingerT = kk * fingerStep;
            Eigen::RowVector3d lerpedFinger =
                transformedFinger.row(joint - 1) * (1. - fingerT) +
                transformedFinger.row(joint) * fingerT;
            P.row(((iKf - 1) * nTrajectorySteps + j) * nFingers *
                      nEvalsPerFingerPerFrame +
                  i * nEvalsPerFingerPerFrame + (joint - 1) * nFingerSteps +
                  kk) = lerpedFinger;
          }
        }
        for (size_t jj = 1; jj < nEvalsPerFingerPerFrame; jj++) {
          // (nFrames - 1) * nFingers * (nEvalsPerFingerPerFrame - 1) * 2, 2);

          int cur = ((iKf - 1) * nTrajectorySteps + j) * nFingers *
                        nEvalsPerFingerPerFrame +
                    i * nEvalsPerFingerPerFrame + jj;
          int last = ((iKf - 1) * nTrajectorySteps + j - 1) * nFingers *
                         nEvalsPerFingerPerFrame +
                     i * nEvalsPerFingerPerFrame + jj;

          PF.block<2, 3>(((iKf - 1) * nTrajectorySteps + j - 1) * nFingers *
                                 (nEvalsPerFingerPerFrame - 1) * 2 +
                             i * (nEvalsPerFingerPerFrame - 1) * 2 +
                             (jj - 1) * 2,
                         0) = (Eigen::MatrixXi(2, 3) << cur,
                               cur - 1,
                               last - 1,
                               cur,
                               last - 1,
                               last)
                                  .finished();
        }
      }
    }
  }
  for (size_t i = 0; i < nEvals; i++) {
    Cost(i) = EvalAt(
        P.row(i).transpose(), vm_.PSG().GetSettings().cost, vm_.PSG().GetMDR());
  }
  layer.set_mesh(P, PF);
  layer.set_data(Cost);
  layer.set_face_based(true);
  layer.double_sided = true;
}

void MainUI::OnGripperBoundInvalidated() {
  auto& layer = GetLayer(Layer::kGripperBound);
  auto V = cube_V;
  Eigen::Vector3d lb = vm_.PSG().GetTopoOptSettings().lower_bound;
  Eigen::Vector3d ub = vm_.PSG().GetTopoOptSettings().upper_bound;
  V = V.array().rowwise() * (ub - lb).transpose().array();
  V = V.array().rowwise() + lb.transpose().array();
  V = (robots::Forward(vm_.GetCurrentPose()) *
       V.transpose().colwise().homogeneous())
          .transpose();
  layer.set_edges(V, cube_E, colors::kBlue);
  layer.line_width = 2;
}

void MainUI::OnNegVolInvalidated() {
  auto& layer = GetLayer(Layer::kNegVol);
  layer.clear();
  if (vm_.GetNegVolValid()) {
    layer.set_mesh((robots::Forward(vm_.GetCurrentPose()) *
                    vm_.GetNegVolV().transpose().colwise().homogeneous())
                       .transpose(),
                   vm_.GetNegVolF());
  }
}

void MainUI::OnGripperInvalidated() {
  auto& layer = GetLayer(Layer::kGripper);
  layer.clear();
  layer.set_mesh((robots::Forward(vm_.GetCurrentPose()) *
                  vm_.GetGripperV().transpose().colwise().homogeneous())
                     .transpose(),
                 vm_.GetGripperF());
}

inline bool MainUI::IsGuizmoVisible() {
  return vm_.PSG().IsMeshLoaded() && (selected_tools_ == Tools::kMeshPosition ||
                                      selected_tools_ == Tools::kRobot);
}

Eigen::Affine3d MainUI::GetGuizmoTransform() const {
  if (selected_tools_ == Tools::kMeshPosition) {
    return vm_.PSG().GetMeshTrans();
  } else if (selected_tools_ == Tools::kRobot) {
    if (selected_keyframe_index_ == -1) {
      return robots::Forward(vm_.GetCurrentPose());
    } else {
      return robots::Forward(
          vm_.PSG().GetTrajectory()[selected_keyframe_index_]);
    }
  }
  return Eigen::Affine3d::Identity();
}

void MainUI::SetGuizmoTransform(const Eigen::Affine3d& trans) {
  if (selected_tools_ == Tools::kMeshPosition) {
    vm_.PSG().TransformMesh(trans * vm_.PSG().GetMeshTrans().inverse());
  } else {
    vm_.SetCurrentPose(trans);
    if (selected_keyframe_index_ != -1) {
      vm_.PSG().reinit_trajectory = false;
      vm_.PSG().EditKeyframe(selected_keyframe_index_, vm_.GetCurrentPose());
    }
  }
}

bool MainUI::imguizmo_pre_draw() {
  if (IsGuizmoVisible()) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
    ImGuizmo::BeginFrame();
    ImGui::PopStyleVar();
  }
  return false;
}

bool MainUI::imguizmo_post_draw() {
  if (IsGuizmoVisible()) {
    // Don't draw the Viewer's default menu: draw just the ImGuizmo
    Eigen::Matrix4f view = (viewer->core().view / viewer->core().camera_zoom);
    Eigen::Matrix4f proj = viewer->core().proj;
    if (viewer->core().orthographic) {
      view(2, 3) -= 1000; /* Hack depth */
    }
    // ImGuizmo doesn't like a lot of scaling in view, shift it to T
    const float z = viewer->core().camera_base_zoom;
    const Eigen::Matrix4f S =
        (Eigen::Matrix4f() << z, 0, 0, 0, 0, z, 0, 0, 0, 0, z, 0, 0, 0, 0, 1)
            .finished();
    view = (view * S.inverse()).eval();
    Eigen::Matrix4f imguizmo_T = GetGuizmoTransform().matrix().cast<float>();
    const Eigen::Matrix4f T0 = imguizmo_T;
    imguizmo_T = (S * imguizmo_T).eval();
    ImGuiIO& io = ImGui::GetIO();
    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
    ImGuizmo::Manipulate(view.data(),
                         proj.data(),
                         imguizmo_operation,
                         ImGuizmo::LOCAL,
                         imguizmo_T.data(),
                         NULL,
                         NULL);
    // invert scaling that was shifted onto T
    imguizmo_T = (S.inverse() * imguizmo_T).eval();
    const float diff = (imguizmo_T - T0).array().abs().maxCoeff();
    // Only call if actually changed; otherwise, triggers on all mouse events
    if (diff > 2e-6) {
      SetGuizmoTransform(Eigen::Affine3d(imguizmo_T.cast<double>()));
    }
  }
  return false;
}

}  // namespace ui
}  // namespace psg
