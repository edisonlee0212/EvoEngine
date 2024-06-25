#pragma once

#include "Skeleton.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
struct TreeGraphNode {
  glm::vec3 start;
  float length;
  float thickness;
  int id;
  int parent_id;
  bool from_apical_bud = false;
  glm::quat global_rotation;
  glm::vec3 position;
  std::weak_ptr<TreeGraphNode> parent;
  std::vector<std::shared_ptr<TreeGraphNode>> children;
};

class TreeGraph : public IAsset {
  void CollectChild(const std::shared_ptr<TreeGraphNode>& node,
                    std::vector<std::vector<std::shared_ptr<TreeGraphNode>>>& graph_nodes, int current_layer) const;

 public:
  bool enable_instantiate_length_limit = false;
  float instantiate_length_limit = 8.0f;
  std::shared_ptr<TreeGraphNode> root;
  std::string name;
  int layer_size;
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  void Serialize(YAML::Emitter& out) const override;

  void Deserialize(const YAML::Node& in) override;

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};

class TreeGraphV2 : public IAsset {
  void CollectChild(const std::shared_ptr<TreeGraphNode>& node,
                    std::vector<std::vector<std::shared_ptr<TreeGraphNode>>>& graph_nodes, int current_layer) const;

 public:
  bool enable_instantiate_length_limit = false;
  float instantiate_length_limit = 8.0f;
  std::shared_ptr<TreeGraphNode> m_root;
  std::string name;
  int layer_size;
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  void Serialize(YAML::Emitter& out) const override;

  void Deserialize(const YAML::Node& in) override;

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
}  // namespace eco_sys_lab