#pragma once

#include "IPrivateComponent.hpp"
#include "SurfaceIlluminationResult.hpp"

#include <map>

namespace evo_engine {

/** A deterministic placement produced by PrefabField before assets are instantiated. */
struct PrefabFieldPlacement {
  size_t asset_index = 0;
  glm::vec3 position = glm::vec3(0.0f);
  float yaw_degrees = 0.0f;
};

/** Central measurement cells in the full field's zero-based grid. */
struct PrefabFieldMeasurementRegion {
  int row_start = 0;
  int column_start = 0;
  int rows = 0;
  int columns = 0;

  [[nodiscard]] bool Contains(int row, int column) const {
    return row >= row_start && row < row_start + rows && column >= column_start && column < column_start + columns;
  }
};

/** OptiX sampling parameters stored with a field so editor and Python runs can share them. */
struct PrefabFieldIlluminationSettings {
  int samples = 64;
  int bounces = 4;
  int seed = 0;
  float push_normal_distance = 0.0001f;
};

/** Illumination measured over one generated plant/model root. */
struct PrefabFieldIlluminationResult {
  size_t model_index = 0;
  size_t asset_index = 0;
  int row = 0;
  int column = 0;
  float total_area = 0.0f;
  glm::vec3 total_flux = glm::vec3(0.0f);
  glm::vec3 average_flux = glm::vec3(0.0f);
  std::map<std::string, SurfaceIlluminationResult> organs;
};

/** Field-level totals plus the original per-plant measurements. */
struct PrefabFieldIlluminationSummary {
  std::vector<PrefabFieldIlluminationResult> models;
  float total_area = 0.0f;
  glm::vec3 total_flux = glm::vec3(0.0f);
  glm::vec3 area_weighted_average_flux = glm::vec3(0.0f);
  glm::vec3 mean_model_average_flux = glm::vec3(0.0f);
  std::map<std::string, SurfaceIlluminationResult> organs;
};

/**
 * Builds a rectangular field by sampling supported prefab/models from an Assets subfolder.
 * The component is crop-agnostic: switching folders is enough to use another model library.
 */
class PrefabField : public IPrivateComponent {
 public:
  std::filesystem::path asset_folder;
  int rows = 4;
  int columns = 10;
  glm::vec2 spacing = glm::vec2(0.25f, 0.75f);
  uint32_t seed = 0;
  glm::vec3 model_rotation_degrees = glm::vec3(0.0f);
  glm::vec3 model_scale = glm::vec3(1.0f);
  float random_yaw_degrees = 0.0f;
  bool recursive = false;
  float illumination_roi = 1.0f;
  PrefabFieldIlluminationSettings illumination_settings;
  mutable PrefabFieldIlluminationSummary last_illumination;

  /** Returns the supported model files currently available below asset_folder. */
  [[nodiscard]] std::vector<std::filesystem::path> CollectAssetPaths() const;

  /** Builds deterministic, centered grid placements without changing the scene. */
  [[nodiscard]] std::vector<PrefabFieldPlacement> BuildPlacements(size_t asset_count) const;

  /** ROI is a fraction of each grid dimension; round to nearest cell (ties up), minimum one cell. */
  [[nodiscard]] PrefabFieldMeasurementRegion GetMeasurementRegion() const;

  /** Removes the generated hierarchy owned by this component. */
  void ClearField() const;

  /** Replaces the generated hierarchy and returns its plant/model roots. */
  [[nodiscard]] std::vector<Entity> GenerateField() const;

  /** Returns plant/model roots from the current generated hierarchy. */
  [[nodiscard]] std::vector<Entity> GetGeneratedEntities() const;

  /** Aggregates completed per-model measurements without performing ray tracing. */
  [[nodiscard]] static PrefabFieldIlluminationSummary SummarizeIllumination(
      std::vector<PrefabFieldIlluminationResult> models);
};

}  // namespace evo_engine
