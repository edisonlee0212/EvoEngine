#pragma once
using namespace evo_engine;
namespace mesh_repair_plugin {
class MeshColoring : public IPrivateComponent{
public:
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
}