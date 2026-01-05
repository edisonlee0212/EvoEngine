#pragma once
#include "Skeleton.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

struct TreeStatistics {
  float dbh = 0.f;
  float volume = 0.f;
  float height = 0.f;
  /**
   * @brief Inspects pruning settings in an editor.
   * @param editor_layer The editor layer managing inspection.
   * @return True if data was not modified during inspection.
   */
  void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Saves pruning settings to a YAML emitter.
   * @param name The name of the settings entry.
   * @param out The YAML emitter to serialize data into.
   */
  void Save(const std::string& name, YAML::Emitter& out) const;

  /**
   * @brief Loads pruning settings from a YAML node.
   * @param name The name of the settings entry.
   * @param in The YAML node containing serialized data.
   */
  void Load(const std::string& name, const YAML::Node& in);
  void Export(const std::filesystem::path& path) const;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);

  /**
   * @brief Calculate stats from a skeleton.
   * @tparam SrcSkeletonData Data type for the source skeleton structure.
   * @tparam SrcFlowData Data type for the source skeleton flow.
   * @tparam SrcNodeData Data type for the source skeleton node.
   * @param skeleton The source skeleton.
   */
  template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
  void Calculate(const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& skeleton);
};

template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
void TreeStatistics::Calculate(const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& skeleton) {
  const auto& sorted_node_list = skeleton.PeekSortedNodeList();
  volume = dbh = height = 0.f;
  for (const auto& node_handle : sorted_node_list) {
    const auto& node = skeleton.PeekNode(node_handle);
    volume += node.info.length * node.info.thickness * node.info.thickness * glm::pi<float>() * .25f;
    if (node.info.global_position.y <= 1.35f && node.info.GetGlobalEndPosition().y >= 1.35f) {
      dbh += node.info.thickness;
    }
  }
  height = skeleton.max.y - skeleton.min.y;
}
}  // namespace eco_sys_lab_plugin