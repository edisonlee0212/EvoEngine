#pragma once
#include "BillboardCloud.hpp"
namespace billboard_clouds_package {
bool InspectBillboardSettings(BillboardCloud::OriginalClusterizationSettings& settings);
bool InspectBillboardSettings(BillboardCloud::FoliageClusterizationSettings& settings);
bool InspectBillboardSettings(BillboardCloud::ClusterizationSettings& settings);
bool InspectBillboardSettings(BillboardCloud::ProjectSettings& settings);
bool InspectBillboardSettings(BillboardCloud::JoinSettings& settings);
bool InspectBillboardSettings(BillboardCloud::RasterizeSettings& settings);
bool InspectBillboardSettings(BillboardCloud::GenerateSettings& settings, const std::string& title);
}  // namespace billboard_clouds_package
