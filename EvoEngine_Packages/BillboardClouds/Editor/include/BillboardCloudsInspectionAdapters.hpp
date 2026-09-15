#pragma once
#include "BillboardCloud.hpp"
#include "Entity.hpp"
#include "InspectorRegistry.hpp"
namespace billboard_clouds_package {
class BillboardCloudsConverter;
struct BillboardCloudsInspectorState {
  BillboardCloud::GenerateSettings settings{};
  evo_engine::AssetRef mesh_ref, prefab_ref;
  evo_engine::EntityRef billboard_entity, point_entity, color_entity;
  std::vector<glm::vec3> points;
};
bool InspectBillboardCloudsConverter(evo_engine::InspectorContext& context, BillboardCloudsConverter& target,
                                     BillboardCloudsInspectorState& state);
}  // namespace billboard_clouds_package
