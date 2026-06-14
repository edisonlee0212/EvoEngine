#pragma once

#include "SorghumPointCloudScanner.hpp"
#include "TreePointCloudScanner.hpp"

namespace dataset_generation_package {
void SerializeTreePointCloudScanner(YAML::Emitter& out, const TreePointCloudScanner& target);
void DeserializeTreePointCloudScanner(const YAML::Node& in, TreePointCloudScanner& target);
void SerializeSorghumPointCloudScanner(YAML::Emitter& out, const SorghumPointCloudScanner& target);
void DeserializeSorghumPointCloudScanner(const YAML::Node& in, SorghumPointCloudScanner& target);
}  // namespace dataset_generation_package
