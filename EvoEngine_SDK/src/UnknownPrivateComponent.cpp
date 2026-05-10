//
// Created by lllll on 8/23/2021.
//

#include "UnknownPrivateComponent.hpp"

#include <algorithm>

using namespace evo_engine;

void UnknownRuntimePayload::SerializePayload(YAML::Emitter& out, const std::vector<std::string>& skipped_keys) const {
  if (!serialized_node_ || !serialized_node_.IsMap()) {
    return;
  }
  for (const auto& item : serialized_node_) {
    if (!item.first.IsScalar()) {
      continue;
    }
    const auto key = item.first.as<std::string>();
    if (std::find(skipped_keys.begin(), skipped_keys.end(), key) != skipped_keys.end()) {
      continue;
    }
    out << YAML::Key << key << YAML::Value << item.second;
  }
}

void UnknownRuntimePayload::SetOriginalTypeName(const std::string& type_name) {
  original_type_name_ = type_name;
}

const std::string& UnknownRuntimePayload::GetOriginalTypeName() const {
  return original_type_name_;
}

void UnknownRuntimePayload::SetSerializedNode(const YAML::Node& node) {
  serialized_node_ = node;
}

const YAML::Node& UnknownRuntimePayload::GetSerializedNode() const {
  return serialized_node_;
}

bool UnknownPrivateComponent::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Text("Missing private component type: %s", original_type_name_.c_str());

  return false;
}

void UnknownPrivateComponent::Serialize(YAML::Emitter& out) const {
  SerializePayload(out, {"tn", "e"});
}

void UnknownPrivateComponent::Deserialize(const YAML::Node& in) {
  if (in["tn"]) {
    original_type_name_ = in["tn"].as<std::string>();
  }
  serialized_node_ = in;
}

bool UnknownAsset::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Text("Missing asset type: %s", original_type_name_.c_str());
  return false;
}

void UnknownAsset::Serialize(YAML::Emitter& out) const {
  SerializePayload(out, {"type_name", "handle"});
}

void UnknownAsset::Deserialize(const YAML::Node& in) {
  if (in["type_name"]) {
    original_type_name_ = in["type_name"].as<std::string>();
  }
  serialized_node_ = in;
}

bool UnknownSystem::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Text("Missing system type: %s", original_type_name_.c_str());
  return false;
}

void UnknownSystem::Serialize(YAML::Emitter& out) const {
  SerializePayload(out, {"type_name", "enabled_", "rank_", "handle_"});
}

void UnknownSystem::Deserialize(const YAML::Node& in) {
  if (in["type_name"]) {
    original_type_name_ = in["type_name"].as<std::string>();
  }
  serialized_node_ = in;
}

void UnknownLayer::SetOriginalTypeName(const std::string& type_name) {
  original_type_name_ = type_name;
}

const std::string& UnknownLayer::GetOriginalTypeName() const {
  return original_type_name_;
}

void UnknownLayer::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Text("Missing layer type: %s", original_type_name_.c_str());
}
