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
   * @brief Serializes the foliage descriptor to a YAML emitter.
   * @param[out] out YAML emitter to store the serialized data.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the foliage descriptor from a YAML node.
   * @param[in] in YAML node containing the serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Inspects the foliage descriptor in the editor.
   * @param[in] editor_layer Shared pointer to the editor layer.
   * @return True if the asset's content is not modified during inspection.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Collects asset references from this foliage descriptor.
   * @param[out] list Vector to collect asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  /**
   * \brief Prepares a FoliageController using current growth parameters.
   * \param foliage_controller The controller to configure.
   */
  void PrepareController(FineRootController& foliage_controller) const override;
};
}  // namespace eco_sys_lab_package