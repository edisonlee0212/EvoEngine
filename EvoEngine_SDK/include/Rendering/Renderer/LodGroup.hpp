#pragma once

#include "IPrivateComponent.hpp"
#include "Mesh.hpp"
#include "PrivateComponentRef.hpp"

namespace evo_engine {
class Lod {
 public:
  int index = 0;
  std::vector<PrivateComponentRef> renderers;
  float lod_offset = 0.f;
  float transition_width = 0.f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
};

class LodGroup : public IPrivateComponent {
 public:
  std::vector<Lod> lods;
  bool override_lod_factor = false;
  float lod_factor = 0.f;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) override;
};
}  // namespace evo_engine
