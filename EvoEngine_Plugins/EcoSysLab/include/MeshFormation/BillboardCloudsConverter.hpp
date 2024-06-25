#pragma once

#include "BillboardCloud.hpp"
#include "Skeleton.hpp"
using namespace evo_engine;

namespace eco_sys_lab {
class BillboardCloudsConverter : public IPrivateComponent {
 public:
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
}  // namespace eco_sys_lab