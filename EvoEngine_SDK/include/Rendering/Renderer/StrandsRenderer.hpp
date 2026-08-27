
#pragma once
#include "IPrivateComponent.hpp"
#include "Material.hpp"
#include "Strands.hpp"

namespace evo_engine {

/**
 * @class StrandsRenderer
 * @brief Responsible for rendering strands and managing associated properties.
 */
class EVOENGINE_API StrandsRenderer : public IPrivateComponent {
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
   * @brief Called when the component is created.
   */
  void OnCreate() override;

  /**
   * @brief Called when the component is destroyed.
   */
  void OnDestroy() override;

  /**
   * @brief Collects all asset references used by this component.
   * @param list A vector to store the collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list);

  /**
   * @brief Performs actions required after cloning the component.
   * @param target A shared pointer to the cloned component.
   */
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;
};

}  // namespace evo_engine
