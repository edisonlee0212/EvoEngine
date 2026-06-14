
#pragma once
#include "Material.hpp"
#include "Mesh.hpp"
#include "Scene.hpp"

namespace evo_engine {

/**
 * @class Particles
 * @brief Represents a particle system for use in the engine. Handles particle information,
 *        bounding box calculation, and interaction with the editor and assets.
 */
class Particles : public IPrivateComponent {
 public:
  /**
   * @brief Called when the component is created.
   *        Can be overridden to define initialization logic.
   */
  void OnCreate() override;

  /**
   * @brief The bounding box of the particles. Used for spatial calculations.
   */
  Bound bounding_box;

  /**
   * @brief Determines whether the particles cast shadows in the scene.
   */
  bool cast_shadow = true;

  /**
   * @brief Reference to the particle information list asset.
   */
  AssetRef particle_info_list;

  /**
   * @brief Reference to the mesh asset used by the particles.
   */
  AssetRef mesh;

  /**
   * @brief Reference to the material asset used by the particles.
   */
  AssetRef material;

  /**
   * @brief Recalculates the bounding box for the particles. Should be
   *        called when the particle system is updated.
   */
  void RecalculateBoundingBox();

  /**
   * @brief Post-clone action executed after cloning the component. Allows
   *        additional processing for the cloned component.
   * @param target Shared pointer to the cloned component.
   */
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;

  /**
   * @brief Collects all asset references used by this component into a list.
   * @param list Vector to store collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list);

  /**
   * @brief Called when the component is destroyed. Can be overridden
   *        to define cleanup logic.
   */
  void OnDestroy() override;
};

}  // namespace evo_engine
