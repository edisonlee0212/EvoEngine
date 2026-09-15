#include <SorghumLayer.hpp>
#include "Application.hpp"
#include "AssetPreviewRegistry.hpp"
#include "BtfMaterial.hpp"
#include "BtfMeshRenderer.hpp"
#include "CBTFGroup.hpp"
#include "CBTFImporter.hpp"
#include "DigitalAgricultureInspectionAdapters.hpp"
#include "DigitalAgricultureInspectorStates.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "EditorFileDialogs.hpp"
#include "Material.hpp"
#include "PARSensorGroup.hpp"
#include "Platform.hpp"
#include "Serialization.hpp"
#include "SkyIlluminance.hpp"
#include "Sorghum.hpp"
#include "SorghumCoordinates.hpp"
#include "SorghumDescriptor.hpp"
#include "SorghumGenerator.hpp"
#include "Times.hpp"
using namespace digital_agriculture_package;
using namespace evo_engine;
bool digital_agriculture_package::InspectSorghumLayer(InspectorContext& context, SorghumLayer& layer) {
  const auto& editor_layer = context.editor_layer;
  auto& enable_compressed_btf = layer.enable_compressed_btf;
  auto& sorghum_mesh_generator_settings = layer.sorghum_mesh_generator_settings;
  auto& leaf_albedo_texture = layer.leaf_albedo_texture;
  auto& leaf_normal_texture = layer.leaf_normal_texture;
  auto& leaf_material = layer.leaf_material;
  auto& vertical_subdivision_length = layer.vertical_subdivision_length;
  auto& horizontal_subdivision_step = layer.horizontal_subdivision_step;
  auto& skeleton_width = layer.skeleton_width;
  auto& skeleton_color = layer.skeleton_color;
  auto& leaf_cbtf_group = layer.leaf_cbtf_group;
  const auto window_title = layer.GetLayerName();
  bool open = layer.enable_inspection;
  if (!ImGui::Begin(window_title.c_str(), &open)) {
    ImGui::End();
    layer.enable_inspection = open;
    return false;
  }
  const auto scene = layer.GetScene();
  ImGui::Checkbox("Enable BTF", &enable_compressed_btf);
  if (enable_compressed_btf) {
    editor_layer->DragAndDropButton<CBTFGroup>(leaf_cbtf_group, "Leaf CBTFGroup");
  }
  ImGui::Separator();
  DrawSorghumMeshGeneratorSettingsGui(sorghum_mesh_generator_settings);
  if (ImGui::Button("Generate mesh for all sorghums")) {
    layer.GenerateMeshForAllSorghums(sorghum_mesh_generator_settings);
  }
  if (ImGui::DragFloat("Vertical subdivision max unit length", &vertical_subdivision_length, 0.001f, 0.001f, 1.0f,
                       "%.4f")) {
    vertical_subdivision_length = glm::max(0.0001f, vertical_subdivision_length);
  }

  if (ImGui::DragInt("Horizontal subdivision step", &horizontal_subdivision_step)) {
    horizontal_subdivision_step = glm::max(2, horizontal_subdivision_step);
  }

  if (ImGui::DragFloat("Skeleton width", &skeleton_width, 0.001f, 0.001f, 1.0f, "%.4f")) {
    skeleton_width = glm::max(0.0001f, skeleton_width);
  }
  ImGui::ColorEdit3("Skeleton color", &skeleton_color.x);

  if (editor_layer->DragAndDropButton<Texture2D>(leaf_albedo_texture, "Replace Leaf Albedo Texture")) {
    auto tex = leaf_albedo_texture.Get<Texture2D>();
    if (tex) {
      leaf_material.Get<Material>()->SetTexture(&GltfShadeMaterial::pbr_base_color_texture, tex);
      if (const std::vector<Entity>* sorghum_entities = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
          sorghum_entities && !sorghum_entities->empty()) {
        for (const auto& sorghum_entity : *sorghum_entities) {
          for (const auto child : scene->GetChildren(sorghum_entity)) {
            if (scene->HasPrivateComponent<MeshRenderer>(child)) {
              scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock()->material.Get<Material>()->SetTexture(
                  &GltfShadeMaterial::pbr_base_color_texture, tex);
            }
          }
        }
      }
    }
  }

  if (editor_layer->DragAndDropButton<Texture2D>(leaf_normal_texture, "Replace Leaf Normal Texture")) {
    auto tex = leaf_normal_texture.Get<Texture2D>();
    if (tex) {
      leaf_material.Get<Material>()->SetTexture(&GltfShadeMaterial::normal_texture, tex);
      if (const std::vector<Entity>* sorghum_entities = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
          sorghum_entities && !sorghum_entities->empty()) {
        for (const auto& sorghum_entity : *sorghum_entities) {
          for (const auto child : scene->GetChildren(sorghum_entity)) {
            if (scene->HasPrivateComponent<MeshRenderer>(child)) {
              scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock()->material.Get<Material>()->SetTexture(
                  &GltfShadeMaterial::normal_texture, tex);
            }
          }
        }
      }
    }
  }

  EditorFileDialogs::SaveFile(
      "Export OBJ for all sorghums", "3D Model", {".obj"},
      [&layer](const std::filesystem::path& path) {
        layer.ExportAllSorghumsModel(path.string());
      },
      false);

  ImGui::End();
  layer.enable_inspection = open;
  return false;
}
