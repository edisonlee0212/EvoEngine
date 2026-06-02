
#pragma once
#include "CropShootModel.hpp"
#include "SorghumDescriptor.hpp"
namespace digital_agriculture_package {
using namespace evo_engine;

/**
 * @class Sorghum
 * @brief Represents a procedural model for generating 3D sorghum models.
 *
 * This class provides functionality to manage the sorghum's growth stages,
 * geometry generation, serialization, and editing within the EvoEngine framework.
 */
class Sorghum final : public IPrivateComponent {
 public:
  /** @brief Reference to the sorghum generator asset. */
  AssetRef sorghum_generator;

  /** @brief Reference to the asset containing different sorghum growth stages. */
  AssetRef sorghum_growth_stages;

  /** @brief Reference to the current state of the sorghum model. */
  AssetRef sorghum_state;

  /** @brief Reference to the sorghum descriptor asset. */
  AssetRef sorghum_descriptor;

  /** @brief Reference to the crop descriptor asset (developmental model genotype). */
  AssetRef crop_descriptor;

#ifdef ECOSYSLAB_PACKAGE
  /** @brief Developmental shoot model driven by the crop descriptor. */
  CropShootModel crop_shoot_model;
#endif

  /**
   * @brief Clears the generated geometry entities.
   *
   * This function removes any existing generated geometry entities within the scene.
   */
  void ClearGeometryEntities() const;

  /**
   * @brief Generates the geometry entities for the sorghum model.
   *
   * @param sorghum_mesh_generator_settings The settings used for generating the sorghum mesh.
   */
  void GenerateGeometryEntities(const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings);

  /**
   * @brief Handles cleanup operations when the Sorghum instance is destroyed.
   */
  void OnDestroy() override;

  /**
   * @brief Collects all asset references used by the sorghum instance.
   *
   * @param list The list to which asset references will be added.
   */
  void CollectAssetRef(std::vector<AssetRef>& list);
  /**
   * \brief Get number of leaves for current sorghum.
   * \return Number of leaves.
   */
  uint32_t GetLeafSize();

#ifdef ECOSYSLAB_PACKAGE
  /**
   * @brief Grow the crop model to the specified cumulative GDD and regenerate geometry.
   * Reinitializes from zero each call — suitable for the interactive slider.
   * @param target_gdd Target cumulative growing degree-days.
   * @param daily_temperature Daily mean temperature used for each growth step (°C).
   */
  void GrowCropToGdd(float target_gdd, float daily_temperature = 25.0f);

  /**
   * @brief Advance the crop model by delta_gdd from its current state (incremental).
   * Does NOT reinitialize — designed for per-frame timelapse playback.
   * @param delta_gdd GDD to advance from the current cumulative GDD.
   * @param daily_temperature Daily mean temperature used for each growth step (°C).
   */
  void GrowCropByGdd(float delta_gdd, float daily_temperature = 25.0f);
#endif
};

}  // namespace digital_agriculture_package
