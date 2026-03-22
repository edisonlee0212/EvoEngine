#pragma once
#include "MaizeDescriptor.hpp"
namespace digital_agriculture_plugin {
using namespace evo_engine;

/**
 * @class Maize
 * @brief Represents a procedural model for generating 3D maize models.
 *
 * This class provides functionality to manage the maize's growth stages,
 * geometry generation, serialization, and editing within the EvoEngine framework.
 */
class Maize final : public IPrivateComponent {
 public:
  /** @brief Reference to the maize generator asset. */
  AssetRef maize_generator;

  /** @brief Reference to the asset containing different maize growth stages. */
  AssetRef maize_growth_stages;

  /** @brief Reference to the current state of the maize model. */
  AssetRef maize_state;

  /** @brief Reference to the maize descriptor asset. */
  AssetRef maize_descriptor;

  /**
   * @brief The current age of the maize plant (0.0 to 1.0).
   */
  float plant_age = 1.0f;

  /**
   * @brief Seed for procedural generation.
   */
  int seed = 1;

  /**
   * @brief Clears the generated geometry entities.
   *
   * This function removes any existing generated geometry entities within the scene.
   */
  void ClearGeometryEntities() const;

  /**
   * @brief Generates the geometry entities for the maize model.
   *
   * @param maize_mesh_generator_settings The settings used for generating the maize mesh.
   */
  void GenerateGeometryEntities(const MaizeMeshGeneratorSettings& maize_mesh_generator_settings);

  /**
   * @brief Handles cleanup operations when the Maize instance is destroyed.
   */
  void OnDestroy() override;

  /**
   * @brief Serializes the maize instance to a YAML emitter.
   *
   * @param out The YAML emitter where the data will be serialized.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the maize instance from a given YAML node.
   *
   * @param in The YAML node containing serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Handles the inspection process in the editor layer.
   *
   * This function allows users to modify asset properties during inspection.
   * It returns true if the asset's content is not modified during inspection.
   *
   * @param editor_layer The editor layer instance.
   * @return true if the asset's content remains unmodified, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Collects all asset references used by the maize instance.
   *
   * @param list The list to which asset references will be added.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) override;
  /**
   * \brief Get number of leaves for current maize.
   * \return Number of leaves.
   */
  uint32_t GetLeafSize();

  void Regenerate();
};

}  // namespace digital_agriculture_plugin
