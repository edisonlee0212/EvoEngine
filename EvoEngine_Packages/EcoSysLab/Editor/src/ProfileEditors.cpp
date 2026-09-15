#include "ProfileEditors.hpp"
using namespace eco_sys_lab_package;

void eco_sys_lab_package::DrawProfileBoundary(const ProfileBoundary& target, const ImVec2 origin,
                                              const float zoom_factor, ImDrawList* draw_list, ImU32 color,
                                              float thickness) {
  for (int point_index = 0; point_index < target.points.size() - 1; point_index++) {
    const auto& p1 = target.points[point_index];
    const auto& p2 = target.points[point_index + 1];
    draw_list->AddLine(ImVec2(origin.x + p1.x * zoom_factor, origin.y + p1.y * zoom_factor),
                       ImVec2(origin.x + p2.x * zoom_factor, origin.y + p2.y * zoom_factor), color, thickness);
  }

  const auto& p1 = target.points.back();
  const auto& p2 = target.points[0];
  draw_list->AddLine(ImVec2(origin.x + p1.x * zoom_factor, origin.y + p1.y * zoom_factor),
                     ImVec2(origin.x + p2.x * zoom_factor, origin.y + p2.y * zoom_factor), color, thickness);
}

void eco_sys_lab_package::DrawProfileAttractor(const ProfileAttractor& target, ImVec2 origin, float zoom_factor,
                                               ImDrawList* draw_list, ImU32 color, float thickness) {
  if (target.attractor_points.empty())
    return;
  for (const auto& attractor_point : target.attractor_points) {
    const auto& p1 = attractor_point.first;
    const auto& p2 = attractor_point.second;
    draw_list->AddLine(ImVec2(origin.x + p1.x * zoom_factor, origin.y + p1.y * zoom_factor),
                       ImVec2(origin.x + p2.x * zoom_factor, origin.y + p2.y * zoom_factor), color, thickness);
  }
}
