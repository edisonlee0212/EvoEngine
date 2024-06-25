#pragma once

#include "Skeleton.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
template <typename SkeletonData, typename FlowData, typename NodeData>
class SkeletonSerializer {
 public:
  static void Serialize(
      YAML::Emitter& out, const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
      const std::function<void(YAML::Emitter& node_out, const NodeData& node_data)>& node_func,
      const std::function<void(YAML::Emitter& flow_out, const FlowData& flow_data)>& flow_func,
      const std::function<void(YAML::Emitter& skeleton_out, const SkeletonData& skeleton_data)>& skeleton_func);

  static void Deserialize(
      const YAML::Node& in, Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
      const std::function<void(const YAML::Node& node_in, NodeData& node_data)>& node_func,
      const std::function<void(const YAML::Node& flow_in, FlowData& flow_data)>& flow_func,
      const std::function<void(const YAML::Node& skeleton_in, SkeletonData& skeleton_data)>& skeleton_func);
};

template <typename SkeletonData, typename FlowData, typename NodeData>
void SkeletonSerializer<SkeletonData, FlowData, NodeData>::Serialize(
    YAML::Emitter& out, const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
    const std::function<void(YAML::Emitter& node_out, const NodeData& node_data)>& node_func,
    const std::function<void(YAML::Emitter& flow_out, const FlowData& flow_data)>& flow_func,
    const std::function<void(YAML::Emitter& skeleton_out, const SkeletonData& skeleton_data)>& skeleton_func) {
  out << YAML::Key << "max_node_index_" << YAML::Value << skeleton.max_node_index_;
  out << YAML::Key << "max_flow_index_" << YAML::Value << skeleton.max_flow_index_;
  out << YAML::Key << "new_version_" << YAML::Value << skeleton.new_version_;
  out << YAML::Key << "min" << YAML::Value << skeleton.min;
  out << YAML::Key << "max" << YAML::Value << skeleton.max;

  const auto node_size = skeleton.nodes_.size();
  auto node_recycled_list = std::vector<int>(node_size);
  auto node_flow_handle_list = std::vector<SkeletonFlowHandle>(node_size);
  auto node_parent_handle_list = std::vector<SkeletonNodeHandle>(node_size);
  auto node_apical_list = std::vector<int>(node_size);
  auto node_index_list = std::vector<int>(node_size);

  auto info_global_position_list = std::vector<glm::vec3>(node_size);
  auto info_global_rotation_list = std::vector<glm::quat>(node_size);
  auto info_length_list = std::vector<float>(node_size);
  auto info_thickness_list = std::vector<float>(node_size);
  auto info_color_list = std::vector<glm::vec4>(node_size);
  auto info_locked_list = std::vector<int>(node_size);

  auto info_leaves_list = std::vector<int>(node_size);
  auto info_fruits_list = std::vector<int>(node_size);
  for (int node_index = 0; node_index < node_size; node_index++) {
    const auto& node = skeleton.nodes_[node_index];
    node_recycled_list[node_index] = node.recycled_ ? 1 : 0;
    node_flow_handle_list[node_index] = node.flow_handle_;
    node_parent_handle_list[node_index] = node.parent_handle_;
    node_apical_list[node_index] = node.apical_ ? 1 : 0;
    node_index_list[node_index] = node.index_;

    info_global_position_list[node_index] = node.info.global_position;
    info_global_rotation_list[node_index] = node.info.global_rotation;
    info_length_list[node_index] = node.info.length;
    info_thickness_list[node_index] = node.info.thickness;
    info_color_list[node_index] = node.info.color;
    info_locked_list[node_index] = node.info.locked ? 1 : 0;

    info_leaves_list[node_index] = node.info.leaves;
    info_fruits_list[node_index] = node.info.fruits;
  }
  out << YAML::Key << "nodes_.recycled_" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(node_recycled_list.data()),
                      node_recycled_list.size() * sizeof(int));
  out << YAML::Key << "nodes_.flow_handle_" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(node_flow_handle_list.data()),
                      node_flow_handle_list.size() * sizeof(SkeletonFlowHandle));
  out << YAML::Key << "nodes_.parent_handle_" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(node_parent_handle_list.data()),
                      node_parent_handle_list.size() * sizeof(SkeletonNodeHandle));
  out << YAML::Key << "nodes_.apical_" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(node_apical_list.data()),
                      node_apical_list.size() * sizeof(int));
  out << YAML::Key << "nodes_.index_" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(node_index_list.data()),
                      node_index_list.size() * sizeof(int));

  out << YAML::Key << "nodes_.info.global_position" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(info_global_position_list.data()),
                      info_global_position_list.size() * sizeof(glm::vec3));
  out << YAML::Key << "nodes_.info.global_rotation" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(info_global_rotation_list.data()),
                      info_global_rotation_list.size() * sizeof(glm::quat));
  out << YAML::Key << "nodes_.info.length" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(info_length_list.data()),
                      info_length_list.size() * sizeof(float));
  out << YAML::Key << "nodes_.info.thickness" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(info_thickness_list.data()),
                      info_thickness_list.size() * sizeof(float));
  out << YAML::Key << "nodes_.info.color" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(info_color_list.data()),
                      info_color_list.size() * sizeof(glm::vec4));
  out << YAML::Key << "nodes_.info.locked" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(info_locked_list.data()),
                      info_locked_list.size() * sizeof(int));

  out << YAML::Key << "nodes_.info.leaves" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(info_leaves_list.data()),
                      info_leaves_list.size() * sizeof(float));
  out << YAML::Key << "nodes_.info.fruits" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(info_fruits_list.data()),
                      info_fruits_list.size() * sizeof(float));

  out << YAML::Key << "nodes_.info" << YAML::Value << YAML::BeginSeq;
  for (size_t node_index = 0; node_index < node_size; node_index++) {
    const auto& node = skeleton.nodes_[node_index];
    out << YAML::BeginMap;
    {
      if (!node.info.wounds.empty()) {
        out << YAML::Key << "wounds" << YAML::Value
            << YAML::Binary(reinterpret_cast<const unsigned char*>(node.info.wounds.data()),
                            node.info.wounds.size() * sizeof(SkeletonNodeWound));
      }
    }
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;

  out << YAML::Key << "nodes_.data" << YAML::Value << YAML::BeginSeq;
  for (size_t node_index = 0; node_index < node_size; node_index++) {
    const auto& node = skeleton.nodes_[node_index];
    out << YAML::BeginMap;
    { node_func(out, node.data); }
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;

  const auto flow_size = skeleton.flows_.size();
  auto flow_recycled_list = std::vector<int>(flow_size);
  auto flow_parent_handle_list = std::vector<SkeletonFlowHandle>(flow_size);
  auto flow_apical_list = std::vector<int>(flow_size);
  auto flow_index_list = std::vector<int>(flow_size);
  for (int flow_index = 0; flow_index < flow_size; flow_index++) {
    const auto& flow = skeleton.flows_[flow_index];
    flow_recycled_list[flow_index] = flow.recycled_ ? 1 : 0;
    flow_parent_handle_list[flow_index] = flow.parent_handle_;
    flow_apical_list[flow_index] = flow.apical_ ? 1 : 0;
    flow_index_list[flow_index] = flow.index_;
  }
  out << YAML::Key << "flows_.recycled_" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(flow_recycled_list.data()),
                      flow_recycled_list.size() * sizeof(int));
  out << YAML::Key << "flows_.parent_handle_" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(flow_parent_handle_list.data()),
                      flow_parent_handle_list.size() * sizeof(SkeletonFlowHandle));
  out << YAML::Key << "flows_.apical_" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(flow_apical_list.data()),
                      flow_apical_list.size() * sizeof(int));
  out << YAML::Key << "flows_.index_" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(flow_index_list.data()),
                      flow_index_list.size() * sizeof(int));

  out << YAML::Key << "flows_" << YAML::Value << YAML::BeginSeq;
  for (int flow_index = 0; flow_index < flow_size; flow_index++) {
    const auto& flow = skeleton.flows_[flow_index];
    out << YAML::BeginMap;
    {
      if (!flow.nodes_.empty()) {
        out << YAML::Key << "nodes_" << YAML::Value
            << YAML::Binary(reinterpret_cast<const unsigned char*>(flow.nodes_.data()),
                            flow.nodes_.size() * sizeof(SkeletonNodeHandle));
      }
      out << YAML::Key << "data" << YAML::Value << YAML::BeginMap;
      { flow_func(out, flow.data); }
      out << YAML::EndMap;
    }
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;

  out << YAML::Key << "data" << YAML::Value << YAML::BeginMap;
  skeleton_func(out, skeleton.data);
  out << YAML::EndMap;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
void SkeletonSerializer<SkeletonData, FlowData, NodeData>::Deserialize(
    const YAML::Node& in, Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
    const std::function<void(const YAML::Node& node_in, NodeData& node_data)>& node_func,
    const std::function<void(const YAML::Node& flow_in, FlowData& flow_data)>& flow_func,
    const std::function<void(const YAML::Node& skeleton_in, SkeletonData& skeleton_data)>& skeleton_func) {
  if (in["max_node_index_"])
    skeleton.max_node_index_ = in["max_node_index_"].as<int>();
  if (in["max_flow_index_"])
    skeleton.max_flow_index_ = in["max_flow_index_"].as<int>();
  if (in["new_version_"])
    skeleton.new_version_ = in["new_version_"].as<int>();
  skeleton.version_ = -1;
  if (in["min"])
    skeleton.min = in["min"].as<glm::vec3>();
  if (in["max"])
    skeleton.max = in["max"].as<glm::vec3>();

  if (in["nodes_.recycled_"]) {
    auto node_recycled_list = std::vector<int>();
    const auto data = in["nodes_.recycled_"].as<YAML::Binary>();
    node_recycled_list.resize(data.size() / sizeof(int));
    std::memcpy(node_recycled_list.data(), data.data(), data.size());

    skeleton.nodes_.resize(node_recycled_list.size());
    for (size_t i = 0; i < node_recycled_list.size(); i++) {
      skeleton.nodes_[i].recycled_ = node_recycled_list[i] == 1;
    }
  }

  if (in["nodes_.flow_handle_"]) {
    auto node_flow_handle_list = std::vector<SkeletonFlowHandle>();
    const auto data = in["nodes_.flow_handle_"].as<YAML::Binary>();
    node_flow_handle_list.resize(data.size() / sizeof(SkeletonFlowHandle));
    std::memcpy(node_flow_handle_list.data(), data.data(), data.size());

    for (size_t i = 0; i < node_flow_handle_list.size(); i++) {
      skeleton.nodes_[i].flow_handle_ = node_flow_handle_list[i];
    }
  }

  if (in["nodes_.parent_handle_"]) {
    auto node_parent_handle_list = std::vector<SkeletonNodeHandle>();
    const auto data = in["nodes_.parent_handle_"].as<YAML::Binary>();
    node_parent_handle_list.resize(data.size() / sizeof(SkeletonFlowHandle));
    std::memcpy(node_parent_handle_list.data(), data.data(), data.size());

    for (size_t i = 0; i < node_parent_handle_list.size(); i++) {
      skeleton.nodes_[i].parent_handle_ = node_parent_handle_list[i];
    }
  }

  if (in["nodes_.apical_"]) {
    auto node_apical_list = std::vector<int>();
    const auto data = in["nodes_.apical_"].as<YAML::Binary>();
    node_apical_list.resize(data.size() / sizeof(int));
    std::memcpy(node_apical_list.data(), data.data(), data.size());

    for (size_t i = 0; i < node_apical_list.size(); i++) {
      skeleton.nodes_[i].apical_ = node_apical_list[i] == 1;
    }
  }

  if (in["nodes_.index_"]) {
    auto node_index_list = std::vector<int>();
    const auto data = in["nodes_.index_"].as<YAML::Binary>();
    node_index_list.resize(data.size() / sizeof(int));
    std::memcpy(node_index_list.data(), data.data(), data.size());

    for (size_t i = 0; i < node_index_list.size(); i++) {
      skeleton.nodes_[i].index_ = node_index_list[i];
    }
  }

  if (in["nodes_.info.global_position"]) {
    auto info_global_position_list = std::vector<glm::vec3>();
    const auto data = in["nodes_.info.global_position"].as<YAML::Binary>();
    info_global_position_list.resize(data.size() / sizeof(glm::vec3));
    std::memcpy(info_global_position_list.data(), data.data(), data.size());

    for (size_t i = 0; i < info_global_position_list.size(); i++) {
      skeleton.nodes_[i].info.global_position = info_global_position_list[i];
    }
  }

  if (in["nodes_.info.global_rotation"]) {
    auto info_global_rotation_list = std::vector<glm::quat>();
    const auto data = in["nodes_.info.global_rotation"].as<YAML::Binary>();
    info_global_rotation_list.resize(data.size() / sizeof(glm::quat));
    std::memcpy(info_global_rotation_list.data(), data.data(), data.size());

    for (size_t i = 0; i < info_global_rotation_list.size(); i++) {
      skeleton.nodes_[i].info.global_rotation = info_global_rotation_list[i];
    }
  }

  if (in["nodes_.info.length"]) {
    auto info_length_list = std::vector<float>();
    const auto data = in["nodes_.info.length"].as<YAML::Binary>();
    info_length_list.resize(data.size() / sizeof(float));
    std::memcpy(info_length_list.data(), data.data(), data.size());

    for (size_t i = 0; i < info_length_list.size(); i++) {
      skeleton.nodes_[i].info.length = info_length_list[i];
    }
  }

  if (in["nodes_.info.thickness"]) {
    auto list = std::vector<float>();
    const auto data = in["nodes_.info.thickness"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(float));
    std::memcpy(list.data(), data.data(), data.size());

    for (size_t i = 0; i < list.size(); i++) {
      skeleton.nodes_[i].info.thickness = list[i];
    }
  }

  if (in["nodes_.info.color"]) {
    auto list = std::vector<glm::vec4>();
    const auto data = in["nodes_.info.color"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(glm::vec4));
    std::memcpy(list.data(), data.data(), data.size());

    for (size_t i = 0; i < list.size(); i++) {
      skeleton.nodes_[i].info.color = list[i];
    }
  }

  if (in["nodes_.info.locked"]) {
    auto list = std::vector<int>();
    const auto data = in["nodes_.info.locked"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(int));
    std::memcpy(list.data(), data.data(), data.size());

    for (size_t i = 0; i < list.size(); i++) {
      skeleton.nodes_[i].info.locked = list[i] == 1;
    }
  }

  if (in["nodes_.info.leaves"]) {
    auto list = std::vector<float>();
    const auto data = in["nodes_.info.leaves"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(float));
    std::memcpy(list.data(), data.data(), data.size());

    for (size_t i = 0; i < list.size(); i++) {
      skeleton.nodes_[i].info.leaves = list[i];
    }
  }

  if (in["nodes_.info.fruits"]) {
    auto list = std::vector<float>();
    const auto data = in["nodes_.info.fruits"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(float));
    std::memcpy(list.data(), data.data(), data.size());

    for (size_t i = 0; i < list.size(); i++) {
      skeleton.nodes_[i].info.fruits = list[i];
    }
  }

  if (in["nodes_.info"]) {
    const auto& in_nodes = in["nodes_.info"];
    SkeletonNodeHandle node_handle = 0;
    for (const auto& in_node_info : in_nodes) {
      if (in_node_info["wounds"]) {
        auto& node = skeleton.nodes_[node_handle];
        const auto data = in_node_info["wounds"].as<YAML::Binary>();
        node.info.wounds.resize(data.size() / sizeof(SkeletonNodeWound));
        std::memcpy(node.info.wounds.data(), data.data(), data.size());
      }
    }
  }

  if (in["nodes_.data"]) {
    const auto& in_nodes = in["nodes_.data"];
    SkeletonNodeHandle node_handle = 0;
    for (const auto& in_node_data : in_nodes) {
      auto& node = skeleton.nodes_[node_handle];
      node.handle_ = node_handle;
      node_func(in_node_data, node.data);
      node_handle++;
    }
  }

  if (in["flows_.recycled_"]) {
    auto flow_recycled_list = std::vector<int>();
    const auto data = in["flows_.recycled_"].as<YAML::Binary>();
    flow_recycled_list.resize(data.size() / sizeof(int));
    std::memcpy(flow_recycled_list.data(), data.data(), data.size());

    skeleton.flows_.resize(flow_recycled_list.size());
    for (size_t i = 0; i < flow_recycled_list.size(); i++) {
      skeleton.flows_[i].recycled_ = flow_recycled_list[i] == 1;
    }
  }

  if (in["flows_.parent_handle_"]) {
    auto flow_parent_handle_list = std::vector<SkeletonFlowHandle>();
    const auto data = in["flows_.parent_handle_"].as<YAML::Binary>();
    flow_parent_handle_list.resize(data.size() / sizeof(SkeletonFlowHandle));
    std::memcpy(flow_parent_handle_list.data(), data.data(), data.size());

    for (size_t i = 0; i < flow_parent_handle_list.size(); i++) {
      skeleton.flows_[i].parent_handle_ = flow_parent_handle_list[i];
    }
  }

  if (in["flows_.apical_"]) {
    auto flow_apical_list = std::vector<int>();
    const auto data = in["flows_.apical_"].as<YAML::Binary>();
    flow_apical_list.resize(data.size() / sizeof(int));
    std::memcpy(flow_apical_list.data(), data.data(), data.size());

    for (size_t i = 0; i < flow_apical_list.size(); i++) {
      skeleton.flows_[i].apical_ = flow_apical_list[i] == 1;
    }
  }

  if (in["flows_.index_"]) {
    auto flow_index_list = std::vector<int>();
    const auto data = in["flows_.index_"].as<YAML::Binary>();
    flow_index_list.resize(data.size() / sizeof(int));
    std::memcpy(flow_index_list.data(), data.data(), data.size());

    for (size_t i = 0; i < flow_index_list.size(); i++) {
      skeleton.flows_[i].index_ = flow_index_list[i];
    }
  }

  if (in["flows_"]) {
    const auto& in_flows = in["flows_"];
    SkeletonFlowHandle flow_handle = 0;
    for (const auto& in_flow : in_flows) {
      auto& flow = skeleton.flows_[flow_handle];
      flow.handle_ = flow_handle;
      if (in_flow["nodes_"]) {
        const auto nodes = in_flow["nodes_"].as<YAML::Binary>();
        flow.nodes_.resize(nodes.size() / sizeof(SkeletonNodeHandle));
        std::memcpy(flow.nodes_.data(), nodes.data(), nodes.size());
      }
      if (in_flow["data"]) {
        const auto& in_flow_data = in_flow["data"];
        flow_func(in_flow_data, flow.data);
      }
      flow_handle++;
    }
  }
  skeleton.node_pool_ = {};
  skeleton.flow_pool_ = {};
  for (const auto& node : skeleton.nodes_) {
    if (node.recycled_) {
      skeleton.node_pool_.emplace(node.handle_);
    } else if (node.parent_handle_ != -1) {
      skeleton.nodes_[node.parent_handle_].child_handles_.emplace_back(node.handle_);
    }
  }
  for (auto& node : skeleton.nodes_) {
    node.end_node_ = node.child_handles_.empty();
  }
  for (const auto& flow : skeleton.flows_) {
    if (flow.recycled_) {
      skeleton.flow_pool_.emplace(flow.handle_);
    } else if (flow.parent_handle_ != -1) {
      skeleton.flows_[flow.parent_handle_].child_handles_.emplace_back(flow.handle_);
    }
  }
  skeleton.SortLists();

  skeleton.CalculateDistance();
  skeleton.CalculateFlows();
  skeleton.CalculateRegulatedGlobalRotation();

  if (in["data"])
    skeleton_func(in["data"], skeleton.data);
}
}  // namespace eco_sys_lab
