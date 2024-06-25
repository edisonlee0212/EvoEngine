#pragma once

using namespace evo_engine;
namespace eco_sys_lab {
class ProfileBoundary {
 public:
  std::vector<glm::vec2> points{};
  glm::vec2 center = glm::vec2(0.0f);
  void CalculateCenter();
  void RenderBoundary(ImVec2 origin, float zoom_factor, ImDrawList* draw_list, ImU32 color, float thickness) const;
  [[nodiscard]] bool BoundaryValid() const;
  [[nodiscard]] bool InBoundary(const glm::vec2& position) const;
  [[nodiscard]] bool InBoundary(const glm::vec2& position, glm::vec2& closest_point) const;
  static bool Intersect(const glm::vec2& p1, const glm::vec2& q1, const glm::vec2& p2, const glm::vec2& q2);
};
class ProfileAttractor {
 public:
  std::vector<std::pair<glm::vec2, glm::vec2>> attractor_points{};
  void RenderAttractor(ImVec2 origin, float zoom_factor, ImDrawList* draw_list, ImU32 color, float thickness) const;
  glm::vec2 FindClosestPoint(const glm::vec2& position) const;
};
class ProfileConstraints {
 public:
  std::vector<ProfileBoundary> boundaries{};
  std::vector<ProfileAttractor> attractors{};
  [[nodiscard]] int FindBoundary(const glm::vec2& position) const;
  [[nodiscard]] bool Valid(size_t boundary_index) const;
  [[nodiscard]] glm::vec2 GetTarget(const glm::vec2& position) const;
};
}  // namespace eco_sys_lab