#pragma once

#include "IPrivateComponent.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

class BranchRenderer : public IPrivateComponent {
 public:
  AssetRef material_ref;
  bool screen_space_tessellation = true;
  bool camera_impostors = true;
  float impostor_diameter_pixels = 3.0f;
  float pixel_error = 0.5f;
  float segment_length_pixels = 12.0f;

  void CollectAssetRef(std::vector<AssetRef>& list) {
    list.push_back(material_ref);
  }
};
}  // namespace eco_sys_lab_package
