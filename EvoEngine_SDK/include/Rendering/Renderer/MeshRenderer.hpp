
#pragma once
#include "IPrivateComponent.hpp"

namespace evo_engine {

/**
 * @brief A class responsible for rendering meshes with materials in the engine.
 */
class MeshRenderer final : public IPrivateComponent {
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
   * @brief Called when the MeshRenderer is destroyed.
   */
  void OnDestroy() override;

  /**
   * @brief Collects references to all assets this MeshRenderer depends on.
   *
   * @param list A vector to which the asset references will be added.
   */
  void CollectAssetRef(std::vector<AssetRef>& list);

  /**
   * @brief Executes actions required after cloning this component.
   *
   * @param target A shared pointer to the newly cloned component.
   */
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;
};

}  // namespace evo_engine
