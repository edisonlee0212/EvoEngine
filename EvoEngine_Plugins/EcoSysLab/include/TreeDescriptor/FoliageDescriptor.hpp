#pragma once
#include "Skeleton.hpp"

using namespace evo_engine;

namespace eco_sys_lab {


class FoliageDescriptor : public IAsset {
 public:
  glm::vec2 leaf_size = glm::vec2(0.04f, 0.08f);
  int leaf_count_per_internode = 5;
  float position_variance = 0.175f;
  float rotation_variance = 10.f;
  float branching_angle = 30.f;
  float max_node_thickness = 1.0f;
  float min_root_distance = 0.0f;
  float max_end_distance = 0.2f;

  float horizontal_tropism = 0.f;
  float gravitropism = 0.f;

  AssetRef leaf_material;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  void GenerateFoliageMatrices(std::vector<glm::mat4>& matrices, const SkeletonNodeInfo& internodeInfo,
                               const float treeSize) const;
};

}  // namespace eco_sys_lab