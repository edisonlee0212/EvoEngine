#include "TextureBakingSerializationAdapters.hpp"

using namespace evo_engine;
using namespace texture_baking_package;

void texture_baking_package::SerializeTextureBaking(YAML::Emitter& out, const TextureBaking& target) {
  target.target_mesh_renderer_ref.Save("target_mesh_renderer_ref", out);
  target.reference_mesh_renderer_ref.Save("reference_mesh_renderer_ref", out);
}

void texture_baking_package::DeserializeTextureBaking(const YAML::Node& in, TextureBaking& target) {
  const auto scene = target.GetScene();
  target.target_mesh_renderer_ref.Load("target_mesh_renderer_ref", in, scene);
  target.reference_mesh_renderer_ref.Load("reference_mesh_renderer_ref", in, scene);
}

void TextureBaking::Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) {
  target_mesh_renderer_ref.Relink(map, scene);
  reference_mesh_renderer_ref.Relink(map, scene);
}
