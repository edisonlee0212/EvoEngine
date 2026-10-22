#pragma once

#include "IPrivateComponent.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

class FoliageRenderer : public IPrivateComponent {
 public:
  AssetRef material_ref;

  void CollectAssetRef(std::vector<AssetRef>& list) {
    list.push_back(material_ref);
  }
};
}  // namespace eco_sys_lab_package
