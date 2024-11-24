#pragma once
#include "Curve.hpp"
#include "Plot2D.hpp"
#include "SorghumState.hpp"
using namespace evo_engine;
namespace digital_agriculture_plugin {
class SorghumDescriptor;

class SorghumGrowthStagePair {
  void LeafStateHelper(SorghumLeafState& left, SorghumLeafState& right, float& a, int leaf_index) const;

 public:
  SorghumState left_stage = SorghumState();
  SorghumState right_stage = SorghumState();
  int state_mode = static_cast<int>(StateMode::Default);
  [[nodiscard]] int GetLeafSize(float a) const;
  [[nodiscard]] float GetStemLength(float a) const;
  [[nodiscard]] glm::vec3 GetStemDirection(float a) const;
  [[nodiscard]] glm::vec3 GetStemPoint(float a, float point) const;
  void ApplyPanicle(const std::shared_ptr<SorghumDescriptor>& target_state, float a) const;
  void ApplyStem(const std::shared_ptr<SorghumDescriptor>& target_state, float a) const;
  void Apply(const std::shared_ptr<SorghumDescriptor>& target_sorghum_descriptor, float a) const;
  void ApplyLeaves(const std::shared_ptr<SorghumDescriptor>& target_descriptor, float a) const;
  void ApplyLeaf(const std::shared_ptr<SorghumDescriptor>& target_descriptor, float a, int leaf_index) const;
};

class SorghumGrowthStages : public IAsset {
 public:
  std::vector<std::pair<float, SorghumState>> sorghum_growth_stages;
  int state_mode = static_cast<int>(StateMode::Default);
  [[nodiscard]] bool ImportCsv(const std::filesystem::path& file_path);
  [[nodiscard]] float GetCurrentStartTime() const;
  [[nodiscard]] float GetCurrentEndTime() const;
  void Add(float time, const SorghumState& state);
  void ResetTime(float previous_time, float new_time);
  void Remove(float time);
  void Apply(const std::shared_ptr<SorghumDescriptor>& target_sorghum_descriptor, float time) const;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;

  [[nodiscard]] Entity CreateEntity(float time = 0.0f) const;
};
}  // namespace digital_agriculture_plugin