#pragma once
#include "TreeDescriptor.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @class BasicBarkDescriptor
 * @brief Represents the properties of a bark descriptor used in procedural tree generation.
 *
 * This class defines parameters that control the bark appearance on procedurally generated trees.
 */
class BasicBarkDescriptor : public IBarkDescriptor {
 public:
  /**
   * @brief Frequency of the bark pattern along the X-axis.
   */
  float bark_x_frequency = 3.0f;

  /**
   * @brief Frequency of the bark pattern along the Y-axis.
   */
  float bark_y_frequency = 5.0f;

  /**
   * @brief Depth of the bark texture.
   */
  float bark_depth = 0.1f;

  /**
   * @brief Base frequency of the bark pattern.
   */
  float base_frequency = 1.0f;

  /**
   * @brief The maximum distance at which the base effect is applied.
   */
  float base_max_distance = 1.f;

  /**
   * @brief Factor by which the base effect decreases with distance.
   */
  float base_distance_decrease_factor = 2.f;

  /**
   * @brief Depth of the base structure.
   */
  float base_depth = .1f;

  /**
   * @brief Reference to the bark material asset.
   */
  AssetRef bark_material_ref;

  /**
   * @brief Inspects and modifies the asset's properties in the editor.
   * @param editor_layer The current editor layer.
   * @return True if the asset's content remains unmodified during inspection.
   */
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Computes a bark pattern value based on input parameters.
   * @param x_factor A factor affecting the bark pattern along the X-axis.
   * @param distance_to_root Distance from the root of the tree.
   * @return The computed bark pattern value.
   */
  float GetValue(float x_factor, float distance_to_root) const override;

  /**
   * @brief Collects asset references used within this descriptor.
   * @param list A list to store the collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list);
};
}  // namespace eco_sys_lab_package
