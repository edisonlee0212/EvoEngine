#pragma once

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @struct SkeletalGraphSettings
 * @brief Defines visualization settings for the skeletal graph of a tree.
 */
struct SkeletalGraphSettings {
  float line_thickness = 0.0f;          ///< Thickness of the skeletal graph lines.
  float fixed_line_thickness = 0.002f;  ///< Fixed thickness value for lines.
  float branch_point_size = 1.0f;       ///< Size of branch points.
  float junction_point_size = 1.f;      ///< Size of junction points.

  bool fixed_point_size = true;                                    ///< Determines if point size is fixed.
  float fixed_point_size_factor = 0.005f;                          ///< Factor affecting fixed point size.
  glm::vec4 line_color = glm::vec4(1.f, .5f, 0.5f, 1.0f);          ///< Color of skeletal graph lines.
  glm::vec4 branch_point_color = glm::vec4(1.f, 1.f, 0.f, 1.f);    ///< Color of branch points.
  glm::vec4 junction_point_color = glm::vec4(0.f, .7f, 1.f, 1.f);  ///< Color of junction points.

  glm::vec4 line_focus_color = glm::vec4(1.f, 0.f, 0.f, 1.f);    ///< Color when a line is in focus.
  glm::vec4 branch_focus_color = glm::vec4(1.f, 0.f, 0.f, 1.f);  ///< Color when a branch is in focus.

  /**
   * @brief Handles inspection of graphical settings in the editor.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Saves to a YAML emitter.
   * @param name The name of the settings entry.
   * @param out The YAML emitter to serialize data into.
   */
  void Save(const std::string& name, YAML::Emitter& out) const;

  /**
   * @brief Loads from a YAML node.
   * @param name The name of the settings entry.
   * @param in The YAML node containing serialized data.
   */
  void Load(const std::string& name, const YAML::Node& in);
};
}  // namespace eco_sys_lab_plugin