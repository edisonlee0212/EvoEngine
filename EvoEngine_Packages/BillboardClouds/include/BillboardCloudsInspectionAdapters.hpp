#pragma once

#include "InspectorRegistry.hpp"

namespace billboard_clouds_package {
class BillboardCloudsConverter;

bool InspectBillboardCloudsConverter(evo_engine::InspectorContext& context,
                                     BillboardCloudsConverter& billboard_clouds_converter);
}  // namespace billboard_clouds_package
