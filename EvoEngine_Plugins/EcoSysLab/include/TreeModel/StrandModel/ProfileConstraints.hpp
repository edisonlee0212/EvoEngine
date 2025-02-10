
#pragma once

using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @class ProfileBoundary
 * @brief Represents a boundary profile defined by a set of points.
 */
class ProfileBoundary {
 public:
  /// A list of points defining the boundary.
  std::vector<glm::vec2> points{};

  /// The center of the boundary.
  glm::vec2 center = glm::vec2(0.0f);

  /**
   * @brief Calculates the center of the boundary based on its points.
   */
  void CalculateCenter();

  /**
   * @brief Renders the boundary on the UI.
   * @param origin The origin point for rendering.
   * @param zoom_factor The zoom factor for rendering scale.
   * @param draw_list The ImGui drawing list used for rendering.
   * @param color The color of the boundary.
   * @param thickness The thickness of the rendered boundary.
   */
  void RenderBoundary(ImVec2 origin, float zoom_factor, ImDrawList* draw_list, ImU32 color, float thickness) const;

  /**
   * @brief Checks if the boundary is valid.
   * @return True if the boundary is valid, otherwise false.
   */
  [[nodiscard]] bool BoundaryValid() const;

  /**
   * @brief Checks if a given position is inside the boundary.
   * @param position The position to check.
   * @return True if the position is inside the boundary, otherwise false.
   */
  [[nodiscard]] bool InBoundary(const glm::vec2& position) const;

  /**
   * @brief Checks if a given position is inside the boundary and finds the closest point on the boundary.
   * @param position The position to check.
   * @param closest_point The closest point on the boundary if the position is outside.
   * @return True if the position is inside the boundary, otherwise false.
   */
  [[nodiscard]] bool InBoundary(const glm::vec2& position, glm::vec2& closest_point) const;

  /**
   * @brief Checks if two line segments intersect.
   * @param p1 The first point of the first segment.
   * @param q1 The second point of the first segment.
   * @param p2 The first point of the second segment.
   * @param q2 The second point of the second segment.
   * @return True if the segments intersect, otherwise false.
   */
  static bool Intersect(const glm::vec2& p1, const glm::vec2& q1, const glm::vec2& p2, const glm::vec2& q2);
};

/**
 * @class ProfileAttractor
 * @brief Represents an attractor profile that influences nearby structures.
 */
class ProfileAttractor {
 public:
  /// A list of attractor points, represented as pairs of glm::vec2.
  std::vector<std::pair<glm::vec2, glm::vec2>> attractor_points{};

  /**
   * @brief Renders the attractor points on the UI.
   * @param origin The origin point for rendering.
   * @param zoom_factor The zoom factor for rendering scale.
   * @param draw_list The ImGui drawing list used for rendering.
   * @param color The color of the attractor.
   * @param thickness The thickness of the rendered points.
   */
  void RenderAttractor(ImVec2 origin, float zoom_factor, ImDrawList* draw_list, ImU32 color, float thickness) const;

  /**
   * @brief Finds the closest attractor point to a given position.
   * @param position The position to check.
   * @return The closest point on the attractor to the given position.
   */
  glm::vec2 FindClosestPoint(const glm::vec2& position) const;
};

/**
 * @class ProfileConstraints
 * @brief Represents constraints for procedural profiles such as boundaries and attractors.
 */
class ProfileConstraints {
 public:
  /// A collection of profile boundaries.
  std::vector<ProfileBoundary> boundaries{};

  /// A collection of profile attractors.
  std::vector<ProfileAttractor> attractors{};

  /**
   * @brief Finds the index of the boundary containing a given position.
   * @param position The position to check.
   * @return The index of the boundary containing the position, or -1 if not found.
   */
  [[nodiscard]] int FindBoundary(const glm::vec2& position) const;

  /**
   * @brief Checks if a specified boundary index is valid.
   * @param boundary_index The index of the boundary.
   * @return True if the specified boundary index is valid, otherwise false.
   */
  [[nodiscard]] bool Valid(size_t boundary_index) const;

  /**
   * @brief Gets the target position based on a given input position.
   * @param position The input position.
   * @return The target position influenced by boundaries and attractors.
   */
  [[nodiscard]] glm::vec2 GetTarget(const glm::vec2& position) const;
};

}  // namespace eco_sys_lab_plugin
