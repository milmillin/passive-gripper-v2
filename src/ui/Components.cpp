#include "Components.h"

#include <imgui.h>

namespace psg {
namespace ui {

bool MyInputDouble3(const char* name, double* v, double step, const char* fmt) {
  float w = ImGui::GetContentRegionAvailWidth();
  float p = ImGui::GetStyle().FramePadding.x;
  bool updated = false;
  ImGui::PushID(name);
  ImGui::PushItemWidth((w - 2 * p) / 3.);
  updated |= ImGui::InputDouble("##x", &v[0], step, step, fmt);
  ImGui::SameLine(0, p);
  updated |= ImGui::InputDouble("##y", &v[1], step, step, fmt);
  ImGui::SameLine(0, p);
  updated |= ImGui::InputDouble("##z", &v[2], step, step, fmt);
  ImGui::PopItemWidth();
  ImGui::PopID();
  return updated;
}

bool MyInputDouble3Convert(const char* name,
                           double* v,
                           double factor,
                           double step,
                           const char* fmt) {
  double conv[3];
  conv[0] = v[0] * factor;
  conv[1] = v[1] * factor;
  conv[2] = v[2] * factor;
  float w = ImGui::GetContentRegionAvailWidth();
  float p = ImGui::GetStyle().FramePadding.x;
  bool updated = false;
  ImGui::PushID(name);
  ImGui::PushItemWidth((w - 2 * p) / 3.);
  updated |= ImGui::InputDouble("##x", &conv[0], step, step, fmt);
  ImGui::SameLine(0, p);
  updated |= ImGui::InputDouble("##y", &conv[1], step, step, fmt);
  ImGui::SameLine(0, p);
  updated |= ImGui::InputDouble("##z", &conv[2], step, step, fmt);
  ImGui::PopItemWidth();
  ImGui::PopID();
  if (updated) {
    v[0] = conv[0] / factor;
    v[1] = conv[1] / factor;
    v[2] = conv[2] / factor;
  }
  return updated;
}

}  // namespace ui
}  // namespace psg