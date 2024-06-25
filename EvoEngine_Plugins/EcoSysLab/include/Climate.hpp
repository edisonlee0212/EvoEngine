#pragma once

#include "ClimateModel.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
class ClimateDescriptor : public IAsset {
 public:
  ClimateParameters climate_parameters;

  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

  void Serialize(YAML::Emitter& out) const override;

  void Deserialize(const YAML::Node& in) override;
};
class Climate : public IPrivateComponent {
 public:
  ClimateModel climate_model;
  AssetRef climate_descriptor;

  /**ImGui menu goes here. Also, you can take care you visualization with Gizmos here.
   * Note that the visualization will only be activated while you are inspecting the soil private component in the
   * entity inspector.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
  void Serialize(YAML::Emitter& out) const override;

  void Deserialize(const YAML::Node& in) override;

  void CollectAssetRef(std::vector<AssetRef>& list) override;

  void InitializeClimateModel();

  void PrepareForGrowth();
};
}  // namespace eco_sys_lab
