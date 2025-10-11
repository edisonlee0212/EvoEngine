#pragma once
using namespace evo_engine;

namespace eco_sys_lab_plugin {
/**
 * @brief Settings related to tree growth algorithms.
 */
struct TreeGrowthSettings {
  float node_developmental_vigor_filling_rate = 1.0f;  ///< The rate at which developmental vigor is filled.
  bool use_space_colonization = false;                 ///< Whether to use space colonization for branching.
  bool space_colonization_auto_resize = false;         ///< Whether the space colonization method resizes automatically.
  float space_colonization_removal_distance_factor = 2;    ///< Distance factor for removing colonization nodes.
  float space_colonization_detection_distance_factor = 4;  ///< Distance factor for detecting colonization nodes.
  float space_colonization_theta = 90.0f;                  ///< The angle parameter used by space colonization.
  /**
   * @brief Inspects pruning settings in an editor.
   * @param editor_layer The editor layer managing inspection.
   * @return True if data was not modified during inspection.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Saves pruning settings to a YAML emitter.
   * @param name The name of the settings entry.
   * @param out The YAML emitter to serialize data into.
   */
  void Save(const std::string& name, YAML::Emitter& out) const;

  /**
   * @brief Loads pruning settings from a YAML node.
   * @param name The name of the settings entry.
   * @param in The YAML node containing serialized data.
   */
  void Load(const std::string& name, const YAML::Node& in);
};
}  // namespace eco_sys_lab_plugin