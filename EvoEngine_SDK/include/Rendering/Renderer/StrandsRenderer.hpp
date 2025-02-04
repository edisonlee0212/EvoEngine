
#pragma once
#include "IPrivateComponent.hpp"
#include "Material.hpp"
#include "Strands.hpp"

namespace evo_engine {

/**
 * @class StrandsRenderer
 * @brief Responsible for rendering strands and managing associated properties.
 */
class StrandsRenderer : public IPrivateComponent {
  /**
   * @brief Renders the bounding area of the strands with the specified color.
   * @param editor_layer A shared pointer to the editor layer.
   * @param color The color to use for rendering the bound.
   */
  void RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, glm::vec4& color);

 public:
  /**
   * @brief Determines whether the strands cast shadows.
   */
  bool cast_shadow = true;

  /**
   * @brief Reference to the strands asset.
   */
  AssetRef strands;

  /**
   * @brief Reference to the material asset.
   */
  AssetRef material;

  /**
   * @brief Displays the component properties in the inspector.
   * @param editor_layer A shared pointer to the editor layer.
   * @return True if the inspection resulted in changes to the component.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Called when the component is created.
   */
  void OnCreate() override;

  /**
   * @brief Serializes the component data into a YAML emitter.
   * @param out A YAML::Emitter to store the serialized data.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the component data from a YAML node.
   * @param in A YAML::Node containing the serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Called when the component is destroyed.
   */
  void OnDestroy() override;

  /**
   * @brief Collects all asset references used by this component.
   * @param list A vector to store the collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  /**
   * @brief Performs actions required after cloning the component.
   * @param target A shared pointer to the cloned component.
   */
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;
};

}  // namespace evo_engine
