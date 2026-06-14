#pragma once
#include "Skeleton.hpp"
#include "TreeDescriptor.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;
/**
 * @class BasicFoliageDescriptor
 * @brief Represents a foliage descriptor for procedural tree generation.
 *
 * This class defines parameters for generating foliage on a procedural tree model.
 * It includes attributes for leaf size, count, branching angles, and material references.
 */
class BasicFineRootDescriptor : public IFineRootDescriptor {
 public:
  /// Reference to the leaf material asset.
  AssetRef fine_root_material_ref;
  /**
   * @brief Inspects the foliage descriptor in the editor.
   * @param[in] editor_layer Shared pointer to the editor layer.
   * @return True if the asset's content is not modified during inspection.
   */
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Collects asset references from this foliage descriptor.
   * @param[out] list Vector to collect asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list);

  /**
   * \brief Prepares a FoliageController using current growth parameters.
   * \param foliage_controller The controller to configure.
   */
  void PrepareController(FineRootController& foliage_controller) const override;
};
}  // namespace eco_sys_lab_package
