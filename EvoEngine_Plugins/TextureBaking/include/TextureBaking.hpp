#pragma once

namespace evo_engine {
class TextureBaking : public IPrivateComponent {
 public:
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  PrivateComponentRef target_mesh_renderer_ref;
  PrivateComponentRef reference_mesh_renderer_ref;

  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) override;
};
}  // namespace evo_engine