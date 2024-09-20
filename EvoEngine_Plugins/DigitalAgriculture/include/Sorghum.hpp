#pragma once
#include "SorghumDescriptor.hpp"
using namespace evo_engine;
namespace digital_agriculture_plugin {
class Sorghum final : public IPrivateComponent {
 public:
  AssetRef sorghum_state_generator;
  AssetRef sorghum_growth_stages;
  AssetRef sorghum_descriptor;
  void ClearGeometryEntities() const;
  void GenerateGeometryEntities(const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings);

  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};
}  // namespace eco_sys_lab_plugin