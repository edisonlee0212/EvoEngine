#pragma once

#include "BillboardCloud.hpp"

namespace billboard_clouds_plugin {
using namespace evo_engine;
class BillboardCloudsConverter : public IPrivateComponent {
 public:
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
}  // namespace billboard_clouds_plugin
