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

void UnknownLayer::SetOriginalTypeName(const std::string& type_name) {
  original_type_name_ = type_name;
}

const std::string& UnknownLayer::GetOriginalTypeName() const {
  return original_type_name_;
}
