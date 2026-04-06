#pragma once
#include "SorghumField.hpp"

using namespace evo_engine;

namespace digital_agriculture_plugin {

struct FieldIlluminationStats {
  float total_area = 0.0f;
  glm::vec3 total_flux = glm::vec3(0.0f);
  glm::vec3 average_flux = glm::vec3(0.0f);
};

class SorghumFieldGrid final : public IPrivateComponent {
 public:
  AssetRef sorghum_field_asset;

  int rows = 10;
  int columns = 10;
  float row_spacing = 0.76f;
  float column_spacing = 0.1f;
  float row_spacing_std = 0.0f;
  float column_spacing_std = 0.0f;

  float sorghum_size = 1.0f;
  int size_limit = 2000;
  uint32_t base_seed = 0;

  FieldIlluminationStats illumination_stats;

  void RecreateField();
  void CalculateIlluminationForField();
  bool CalculateAndExportFieldIlluminationTest();

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};

}  // namespace digital_agriculture_plugin
