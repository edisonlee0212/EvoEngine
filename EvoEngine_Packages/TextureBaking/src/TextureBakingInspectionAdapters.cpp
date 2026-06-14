#include "TextureBakingInspectionAdapters.hpp"

#include "TextureBaker.hpp"
#include "TextureBaking.hpp"

using namespace evo_engine;
using namespace texture_baking_package;

bool texture_baking_package::InspectTextureBaking(InspectorContext& context, TextureBaking& texture_baking) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (editor_layer->DragAndDropButton<MeshRenderer>(texture_baking.reference_mesh_renderer_ref, "Reference entity"))
    changed = true;
  if (editor_layer->DragAndDropButton<MeshRenderer>(texture_baking.target_mesh_renderer_ref, "Target entity"))
    changed = true;

  if (texture_baking.reference_mesh_renderer_ref.Get<MeshRenderer>() &&
      texture_baking.target_mesh_renderer_ref.Get<MeshRenderer>()) {
    const auto reference_mmr = texture_baking.reference_mesh_renderer_ref.Get<MeshRenderer>();
    const auto target_mmr = texture_baking.target_mesh_renderer_ref.Get<MeshRenderer>();
    const auto ref_mesh = reference_mmr->mesh.Get<Mesh>();
    const auto ref_material = reference_mmr->material.Get<Material>();
    const auto target_mesh = target_mmr->mesh.Get<Mesh>();
    if (ref_mesh && ref_material && target_mesh) {
      static TextureBaker::Parameters texture_baker_params{};
      if (ImGui::TreeNodeEx("Texture baking parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Discard alpha", &texture_baker_params.discard_alpha);
        static glm::ivec2 texture_resolution = {512, 512};
        if (ImGui::DragInt2("Overwrite resolution", &texture_resolution.x, 1, 1, 8192)) {
          texture_baker_params.texture_resolution = glm::clamp(texture_resolution, {1, 1}, {8192, 8192});
        }
        ImGui::Checkbox("Diffuse", &texture_baker_params.diffuse_enabled);
        ImGui::Checkbox("Normal", &texture_baker_params.normal_enabled);
        ImGui::Checkbox("Roughness", &texture_baker_params.roughness_enabled);
        ImGui::Checkbox("Metallic", &texture_baker_params.metallic_enabled);
        ImGui::Checkbox("Ao", &texture_baker_params.ao_enabled);
        ImGui::Checkbox("Force rewrite uv", &texture_baker_params.force_rewrite_uv);
        ImGui::DragFloat("Ray casting range", &texture_baker_params.ray_casting_range, 0.01f, 0.01f, 1.f);
        ImGui::DragFloat("Thin scale factor", &texture_baker_params.thin_scale_factor, 0.001f, 0.0f, 1.f);
        ImGui::Checkbox("Cull back face", &texture_baker_params.cull_back_face);
        ImGui::ColorEdit4("Empty space color", &texture_baker_params.empty_space_color.x);
        static unsigned proposed_sampling_method = 0;
        if (ImGui::Combo("Sampling method", {"Nearest", "Bilinear"}, proposed_sampling_method)) {
          texture_baker_params.sampling_method = static_cast<TextureBaker::SamplingMethod>(proposed_sampling_method);
        }
        static unsigned proposed_ray_casting_direction_mode = 0;
        if (ImGui::Combo("Ray casting mode", {"Default", "AggressiveSmoothing"}, proposed_ray_casting_direction_mode)) {
          texture_baker_params.ray_casting_direction_mode =
              static_cast<TextureBaker::RayCastingDirectionMode>(proposed_ray_casting_direction_mode);
        }

        static unsigned proposed_unresolved_pixel_mode = 0;
        if (ImGui::Combo("Unresolved pixel mode", {"Default", "Dilate"}, proposed_unresolved_pixel_mode)) {
          texture_baker_params.unresolved_pixel_mode =
              static_cast<TextureBaker::UnresolvedPixelMode>(proposed_unresolved_pixel_mode);
        }
        if (texture_baker_params.unresolved_pixel_mode == TextureBaker::UnresolvedPixelMode::Default) {
          ImGui::ColorEdit4("Unresolved pixel color", &texture_baker_params.unresolved_pixel_color.x);
        }
        if (ImGui::DragInt("Post dilate distance", &texture_baker_params.post_dilate_distance, 1, -1, 1024)) {
          texture_baker_params.post_dilate_distance = glm::clamp(texture_baker_params.post_dilate_distance, -1, 1024);
        }

        ImGui::TreePop();
      }

      if (ImGui::Button("Bake")) {
        const auto target_material = AssetManager::CreateTemporaryAsset<Material>();
        TextureBaker::Execute(texture_baker_params, ref_mesh, ref_material, target_mesh, target_material);
        target_mmr->material = target_material;
      }
    }
  }

  return changed;
}
