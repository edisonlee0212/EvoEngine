#pragma once

using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @class SimulationStats
 * @brief Stores statistics related to the simulation process.
 */
class SimulationStats {
 public:
  float last_used_time = 0.0f;  ///< The time used in the last simulation step.
  float total_time = 0.0f;      ///< The total time elapsed in the simulation.
  int internode_size = 0;       ///< The number of internodes in the tree structure.
  int leaf_size = 0;            ///< The number of leaves generated during the simulation.
  int fruit_size = 0;           ///< The number of fruits generated during the simulation.
  int shoot_stem_size = 0;      ///< The number of shoot stems in the simulation.
  int root_node_size = 0;       ///< The number of root nodes in the tree structure.
  int root_stem_size = 0;       ///< The number of root stems in the tree structure.

  /**
   * @brief Inspects the simulation statistics in the editor.
   * @param editor_layer A reference to the editor layer handling the inspection.
   * @return True if the statistics are not updated during inspection, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
};

/**
 * @class SimulationSettings
 * @brief Manages various settings related to tree growth simulation.
 */
class SimulationSettings {
 public:
  float delta_time = 30.f;                  ///< The simulation time step.
  bool soil_simulation = false;             ///< Enables or disables soil simulation.
  bool auto_clear_fruit_and_leaves = true;  ///< Clears fruits and leaves automatically if enabled.
  float crown_shyness_distance = 0.15f;     ///< Distance threshold for crown shyness effect.
  int max_node_count = 0;                   ///< Maximum number of nodes allowed in the simulation.
  int max_flow_count = 0;                   ///< Maximum number of flow interactions.
  float skylight_intensity = 1.f;           ///< Intensity of skylight in the simulation.

  float shadow_distance_loss = 1.f;  ///< Loss factor for shadow intensity over distance.
  float detection_radius = 0.5f;     ///< Radius used for detecting nearby objects.

  float environment_light_intensity = 0.01f;  ///< Intensity of general environmental light.

  int blur_iteration = 0;                     ///< Number of iterations used for blurring effects.
  bool auto_generate_skeletal_graph = false;  ///< Automatically generates a skeletal tree graph if enabled.

  /**
   * @brief Saves simulation settings to a YAML emitter.
   * @param name A string representing the name of the settings block.
   * @param out The YAML emitter to store the settings.
   */
  void Save(const std::string& name, YAML::Emitter& out) const;

  /**
   * @brief Loads simulation settings from a YAML node.
   * @param name A string representing the name of the settings block.
   * @param in The YAML node containing the settings.
   */
  void Load(const std::string& name, const YAML::Node& in);

  /**
   * @brief Serializes the simulation settings into a YAML emitter.
   * @param out The YAML emitter to store serialized data.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes simulation settings from a YAML node.
   * @param in The YAML node containing serialized data.
   */
  void Deserialize(const YAML::Node& in);

  /**
   * @brief Inspects the simulation settings in the editor.
   * @param editor_layer A reference to the editor layer handling the inspection.
   * @return True if the settings are not modified during inspection, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
};

}  // namespace eco_sys_lab_plugin
