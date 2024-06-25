#include "TreeGraph.hpp"

using namespace eco_sys_lab;

void TreeGraph::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "name" << YAML::Value << name;
  out << YAML::Key << "layersize" << YAML::Value << layer_size;
  out << YAML::Key << "layers" << YAML::Value << YAML::BeginMap;
  std::vector<std::vector<std::shared_ptr<TreeGraphNode>>> graph_nodes;
  graph_nodes.resize(layer_size);
  CollectChild(root, graph_nodes, 0);
  for (int layer_index = 0; layer_index < layer_size; layer_index++) {
    out << YAML::Key << std::to_string(layer_index) << YAML::Value << YAML::BeginMap;
    {
      auto& layer = graph_nodes[layer_index];
      out << YAML::Key << "internodesize" << YAML::Value << layer.size();
      for (int node_index = 0; node_index < layer.size(); node_index++) {
        auto node = layer[node_index];
        out << YAML::Key << std::to_string(node_index) << YAML::Value << YAML::BeginMap;
        {
          out << YAML::Key << "id" << YAML::Value << node->id;
          out << YAML::Key << "parent" << YAML::Value << node->parent_id;
          out << YAML::Key << "quat" << YAML::Value << YAML::BeginSeq;
          for (int i = 0; i < 4; i++) {
            out << YAML::BeginMap;
            out << std::to_string(node->global_rotation[i]);
            out << YAML::EndMap;
          }
          out << YAML::EndSeq;

          out << YAML::Key << "position" << YAML::Value << YAML::BeginSeq;
          for (int i = 0; i < 3; i++) {
            out << YAML::BeginMap;
            out << std::to_string(node->position[i]);
            out << YAML::EndMap;
          }
          out << YAML::EndSeq;

          out << YAML::Key << "thickness" << YAML::Value << node->thickness;
          out << YAML::Key << "length" << YAML::Value << node->length;
        }
        out << YAML::EndMap;
      }
    }
    out << YAML::EndMap;
  }

  out << YAML::EndMap;
}

void TreeGraph::CollectAssetRef(std::vector<AssetRef>& list) {
}

struct GraphNode {
  int id;
  int parent;
  glm::vec3 end_position;
  float radius;
};

void TreeGraph::Deserialize(const YAML::Node& in) {
  name = in["name"].as<std::string>();
  layer_size = in["layersize"].as<int>();
  auto layers = in["layers"];
  auto root_layer = layers["0"];
  std::unordered_map<int, std::shared_ptr<TreeGraphNode>> previous_nodes;
  root = std::make_shared<TreeGraphNode>();
  root->start = glm::vec3(0, 0, 0);
  int root_index = 0;
  auto root_node = root_layer["0"];
  root->length = root_node["length"].as<float>();
  root->thickness = root_node["thickness"].as<float>();
  root->id = root_node["id"].as<int>();
  root->parent_id = -1;
  root->from_apical_bud = true;
  int index = 0;
  for (const auto& component : root_node["quat"]) {
    root->global_rotation[index] = component.as<float>();
    index++;
  }
  index = 0;
  for (const auto& component : root_node["position"]) {
    root->position[index] = component.as<float>();
    index++;
  }
  previous_nodes[root->id] = root;
  for (int layer_index = 1; layer_index < layer_size; layer_index++) {
    auto layer = layers[std::to_string(layer_index)];
    auto internode_size = layer["internodesize"].as<int>();
    for (int node_index = 0; node_index < internode_size; node_index++) {
      auto node = layer[std::to_string(node_index)];
      auto parent_node_id = node["parent"].as<int>();
      if (parent_node_id == -1)
        parent_node_id = 0;
      auto& parent_node = previous_nodes[parent_node_id];
      auto new_node = std::make_shared<TreeGraphNode>();
      new_node->id = node["id"].as<int>();
      new_node->start = parent_node->start +
                         parent_node->length * (glm::normalize(parent_node->global_rotation) * glm::vec3(0, 0, -1));
      new_node->thickness = node["thickness"].as<float>();
      new_node->length = node["length"].as<float>();
      new_node->parent_id = parent_node_id;
      if (new_node->parent_id == 0)
        new_node->parent_id = -1;
      index = 0;
      for (const auto& component : node["quat"]) {
        new_node->global_rotation[index] = component.as<float>();
        index++;
      }
      index = 0;
      for (const auto& component : node["position"]) {
        new_node->position[index] = component.as<float>();
        index++;
      }
      if (parent_node->children.empty())
        new_node->from_apical_bud = true;
      previous_nodes[new_node->id] = new_node;
      parent_node->children.push_back(new_node);
      new_node->parent = parent_node;
    }
  }
}

bool TreeGraph::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::Checkbox("Length limit", &enable_instantiate_length_limit);
  if (enable_instantiate_length_limit)
    ImGui::DragFloat("Length limit", &instantiate_length_limit, 0.1f);

  return false;
}

void TreeGraph::CollectChild(const std::shared_ptr<TreeGraphNode>& node,
                             std::vector<std::vector<std::shared_ptr<TreeGraphNode>>>& graph_nodes,
                             const int current_layer) const {
  graph_nodes[current_layer].push_back(node);
  for (const auto& i : node->children) {
    CollectChild(i, graph_nodes, current_layer + 1);
  }
}

void TreeGraphV2::Serialize(YAML::Emitter& out) const {
}

void TreeGraphV2::CollectAssetRef(std::vector<AssetRef>& list) {
}

void TreeGraphV2::Deserialize(const YAML::Node& in) {
  SetUnsaved();
  name = GetTitle();
  int id = 0;
  std::vector<GraphNode> nodes;
  while (in[std::to_string(id)]) {
    auto& in_node = in[std::to_string(id)];
    nodes.emplace_back();
    auto& node = nodes.back();
    node.id = id;
    node.parent = in_node["parent"].as<int>();
    int index = 0;
    for (const auto& component : in_node["position"]) {
      node.end_position[index] = component.as<float>();
      index++;
    }
    index = 0;
    node.radius = in_node["radius"].as<float>();
    id++;
  }
  std::unordered_map<int, std::shared_ptr<TreeGraphNode>> previous_nodes;
  m_root = std::make_shared<TreeGraphNode>();
  m_root->start = glm::vec3(0, 0, 0);
  m_root->thickness = nodes[0].radius;
  m_root->id = 0;
  m_root->parent_id = -1;
  m_root->from_apical_bud = true;
  m_root->position = nodes[0].end_position;
  m_root->length = glm::length(nodes[0].end_position);
  auto direction = glm::normalize(nodes[0].end_position);
  m_root->global_rotation = glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x));
  previous_nodes[0] = m_root;
  for (id = 1; id < nodes.size(); id++) {
    auto& node = nodes[id];
    auto parent_node_id = node.parent;
    auto& parent_node = previous_nodes[parent_node_id];
    auto new_node = std::make_shared<TreeGraphNode>();
    new_node->id = id;
    new_node->start = parent_node->position;
    new_node->thickness = node.radius;
    new_node->parent_id = parent_node_id;
    new_node->position = node.end_position;
    new_node->length = glm::length(new_node->position - new_node->start);
    auto direction = glm::normalize(new_node->position - new_node->start);
    new_node->global_rotation = glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x));
    if (parent_node->children.empty())
      new_node->from_apical_bud = true;
    previous_nodes[id] = new_node;
    parent_node->children.push_back(new_node);
    new_node->parent = parent_node;
  }
}

bool TreeGraphV2::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::Checkbox("Length limit", &enable_instantiate_length_limit);
  ImGui::DragFloat("Length limit", &instantiate_length_limit, 0.1f);
  return changed;
}

void TreeGraphV2::CollectChild(const std::shared_ptr<TreeGraphNode>& node,
                               std::vector<std::vector<std::shared_ptr<TreeGraphNode>>>& graph_nodes,
                               int current_layer) const {
  graph_nodes[current_layer].push_back(node);
  for (const auto& i : node->children) {
    CollectChild(i, graph_nodes, current_layer + 1);
  }
}
