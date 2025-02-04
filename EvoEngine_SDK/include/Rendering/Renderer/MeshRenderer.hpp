
#pragma once
#include "IPrivateComponent.hpp"

namespace evo_engine {

/**
 * @brief A class responsible for rendering meshes with materials in the engine.
 */
class MeshRenderer final : public IPrivateComponent {
  /**
   * @brief Renders the bounding outline of the mesh in the editor.
   *
   * @param editor_layer A shared pointer to the editor layer.
   * @param color A color (vec4) to use for the boundary rendering.
   */
  void RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, const glm::vec4& color);

 public:
  /**
   * @brief Indicates whether the mesh casts shadows.
   */
  bool cast_shadow = true;

  /**
   * @brief Reference to the mesh asset to be rendered.
   */
  AssetRef mesh;

  /**
   * @brief Reference to the material asset applied to the mesh.
   */
  AssetRef material;

  /**
   * @brief Called for the inspection of properties during editing in the editor.
   *
   * @param editor_layer A shared pointer to the editor layer.
   * @return true if the inspection modified any property, otherwise false.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Serializes the MeshRenderer data to a YAML emitter.
   *
   * @param out The YAML emitter to write data to.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the MeshRenderer data from a YAML node.
   *
   * @param in The YAML node containing the serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Called when the MeshRenderer is destroyed.
   */
  void OnDestroy() override;

  /**
   * @brief Collects references to all assets this MeshRenderer depends on.
   *
   * @param list A vector to which the asset references will be added.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  /**
   * @brief Executes actions required after cloning this component.
   *
   * @param target A shared pointer to the newly cloned component.
   */
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;
};

}  // namespace evo_engine
