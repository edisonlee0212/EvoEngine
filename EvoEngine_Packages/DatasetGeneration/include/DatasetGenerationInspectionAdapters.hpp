#pragma once

#include "InspectorRegistry.hpp"

namespace dataset_generation_package {
class TreePointCloudScanner;
class SorghumPointCloudScanner;

bool InspectTreePointCloudScanner(evo_engine::InspectorContext& context, TreePointCloudScanner& scanner);
bool InspectSorghumPointCloudScanner(evo_engine::InspectorContext& context, SorghumPointCloudScanner& scanner);
}  // namespace dataset_generation_package
