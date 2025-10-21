
#pragma once

#include "ClimateModel.hpp"
namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @class ClimateDescriptor
 * @brief Represents a descriptor for climate parameters, which can be serialized and inspected in the editor.
 */
class ClimateDescriptor : public IAsset {
 public:
  /// The parameters defining the climate conditions.
  ClimateParameters climate_parameters;

  /**
   * @brief Inspects the climate descriptor in the editor.
   * @param editor_layer The editor layer handling the inspection.
   * @return True if the asset's content is not modified during inspection, otherwise false.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Generates a thumbnail texture representing the climate descriptor.
   * @return A shared pointer to the generated texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;

  /**
   * @brief Serializes the climate descriptor into a YAML emitter.
   * @param out The YAML emitter to serialize data into.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the climate descriptor from a YAML node.
   * @param in The YAML node containing the serialized data.
   */
  void Deserialize(const YAML::Node& in) override;
};

/**
 * @class Climate
 * @brief Represents a climate system with a model and a reference to a climate descriptor.
 */
class Climate : public IPrivateComponent {
 public:
  /// The climate model used for simulation.
  ClimateModel climate_model;

  /// Reference to the associated climate descriptor asset.
  AssetRef climate_descriptor_ref;

  /**
   * @brief Inspects the climate component in the editor.
   * @param editor_layer The editor layer handling the inspection.
   * @return True if the asset's content is not modified during inspection, otherwise false.
   *
   * @note ImGui menu goes here. You can also handle visualization with Gizmos here.
   *       The visualization will only be activated while inspecting the soil private component in the entity inspector.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Serializes the climate component into a YAML emitter.
   * @param out The YAML emitter to serialize data into.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the climate component from a YAML node.
   * @param in The YAML node containing the serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Collects asset references used by this component.
   * @param list The list to store asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  /**
   * @brief Initializes the climate model with relevant parameters.
   */
  void InitializeClimateModel();

  /**
   * @brief Prepares the climate system for the growth phase.
   */
  void PrepareForGrowth();
};

}  // namespace eco_sys_lab_plugin
