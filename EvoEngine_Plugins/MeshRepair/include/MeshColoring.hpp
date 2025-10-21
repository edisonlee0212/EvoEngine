#pragma once

namespace mesh_repair_plugin {
using namespace evo_engine;
class MeshColoring : public IPrivateComponent {
 public:
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
}  // namespace mesh_repair_plugin
