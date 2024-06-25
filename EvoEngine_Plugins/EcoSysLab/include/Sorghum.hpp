#pragma once
#include "SorghumState.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
class Sorghum : public IPrivateComponent {
 public:
  AssetRef m_sorghumDescriptor;
  AssetRef m_sorghumGrowthDescriptor;
  AssetRef m_sorghumState;
  void ClearGeometryEntities();
  void GenerateGeometryEntities(const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings);

  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};
}  // namespace eco_sys_lab