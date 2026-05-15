#pragma once
#include "AssetRef.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

struct DsMaterials {
  AssetRef bark_material_ref;          ///< Reference to bark material asset.
  AssetRef inner_wood_material_ref;    ///< Reference to inner wood material asset.
  AssetRef splinter_material_ref;      ///< Reference to splinter material asset.
  AssetRef leaf_material_ref;          ///< Reference to leaf material asset.
  AssetRef snow_material_ref;          ///< Reference to snow material asset.
  AssetRef segment_pair_material_ref;  ///< Reference to segment pair material asset.
  AssetRef wireframe_material_ref;     ///< Reference to wireframe material asset.
};
}  // namespace eco_sys_lab_package