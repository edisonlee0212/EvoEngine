#pragma once

namespace texture_baking_package {
using namespace evo_engine;

class TextureBaking : public IPrivateComponent {
 public:
  PrivateComponentRef target_mesh_renderer_ref;
  PrivateComponentRef reference_mesh_renderer_ref;

  void Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene);
};
}  // namespace texture_baking_package
