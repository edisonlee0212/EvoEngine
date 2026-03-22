#ifdef CUDA_MODULE_PLUGIN
#  include <TriangleIlluminationEstimator.hpp>
#  include "BtfMeshRenderer.hpp"
#  include "RayTracerLayer.hpp"
#endif
#include <MaizeLayer.hpp>
#include "ClassRegistry.hpp"
#include "Platform.hpp"
#include "SkyIlluminance.hpp"
#include "MaizeGenerator.hpp"
#include "MaizeGrowthStages.hpp"
#include "Times.hpp"

#include "Material.hpp"
#include "Maize.hpp" // Component
// #include "MaizeCoordinates.hpp" // Not created
#include "MaizeDescriptor.hpp"
#ifdef CUDA_MODULE_PLUGIN
#  include "CBTFGroup.hpp"
#  include "PARSensorGroup.hpp"
#endif
using namespace digital_agriculture_plugin;
using namespace evo_engine;

// Registrations
AssetRegistration<MaizeDescriptor> maize_descriptor_registry("MaizeDescriptor", {".maize"});
PrivateComponentRegistration<Maize> maize_registry("Maize");

AssetRegistration<MaizeGrowthStages> mgt_registry("MaizeGrowthStages", {".mgs"});
AssetRegistration<MaizeState> ms_registry("MaizeState", {".ms"});

AssetRegistration<MaizeGenerator> mg_registry("MaizeGenerator", {".mg"}); // Assuming user wants separation

// AssetRegistration<MaizeField> mf_registry("MaizeField", {".maizefield"}); // Not creating MaizeField yet

void MaizeLayer::OnCreate() {
  if (!leaf_material.Get<Material>()) {
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    leaf_material = material;
    material->SetAlbedoTexture(leaf_albedo_texture.Get<Texture2D>());
    material->material_properties.albedo_color = glm::vec3(113.0f / 255, 169.0f / 255, 44.0f / 255);
    material->material_properties.roughness = 0.8f;
    material->material_properties.metallic = 0.1f;
  }

  if (!leaf_bottom_face_material.Get<Material>()) {
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    leaf_bottom_face_material = material;
    material->SetAlbedoTexture(leaf_albedo_texture.Get<Texture2D>());
    material->material_properties.albedo_color = glm::vec3(113.0f / 255, 169.0f / 255, 44.0f / 255);
    material->material_properties.roughness = 0.8f;
    material->material_properties.metallic = 0.1f;
  }

  if (!panicle_material.Get<Material>()) {
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    panicle_material = material;
    material->material_properties.albedo_color = glm::vec3(255.0 / 255, 210.0 / 255, 0.0 / 255);
    material->material_properties.roughness = 0.5f;
    material->material_properties.metallic = 0.0f;
  }

  for (auto& i : segmented_leaf_materials) {
    if (!i.Get<Material>()) {
      const auto material = AssetManager::CreateTemporaryAsset<Material>();
      i = material;
      material->material_properties.albedo_color = glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f));
      material->material_properties.roughness = 1.0f;
      material->material_properties.metallic = 0.0f;
    }
  }
}

void MaizeLayer::GenerateMeshForAllMaizes(
    const MaizeMeshGeneratorSettings& maize_mesh_generator_settings) const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* maize_entities = scene->UnsafeGetPrivateComponentOwnersList<Maize>();
      maize_entities && !maize_entities->empty()) {
    for (const auto& maize_entity : *maize_entities) {
      const auto maize = scene->GetOrSetPrivateComponent<Maize>(maize_entity).lock();
      maize->GenerateGeometryEntities(maize_mesh_generator_settings);
    }
  }
}

void MaizeLayer::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  const auto scene = GetScene();
#ifdef CUDA_MODULE_PLUGIN
  if (ImGui::TreeNodeEx("Illumination Estimation")) {
    ImGui::DragInt("Seed", &m_seed);
    ImGui::DragFloat("Push distance along normal", &push_distance, 0.0001f, -1.0f, 1.0f, "%.5f");
    ray_properties.OnInspect();

    if (ImGui::Button("Calculate illumination")) {
      CalculateIlluminationFrameByFrame();
    }
    if (ImGui::Button("Calculate illumination instantly")) {
      CalculateIllumination();
    }

    static bool show_probes = false;

    ImGui::Checkbox("Show probes", &show_probes);

    if (show_probes) {
      static bool depth_test = true;
      static std::shared_ptr<ParticleInfoList> probe_debug_info_list;
      static float node_render_size = .01f;
      static float energy_scale_factor = 1.f;
      if (!probe_debug_info_list)
        probe_debug_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
      if (ImGui::TreeNode("Debug settings")) {
        ImGui::DragFloat("Probe size", &node_render_size, 0.001f, 0.0f, 1.f);
        ImGui::DragFloat("Energy scale factor", &energy_scale_factor, 0.01f, 0.0f, 100.f);
        if (ImGui::Button("Refresh debug info")) {
          if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<Maize>()) {
            std::vector<ParticleInfo> particle_infos;
            for (const auto maize_entity : *owners) {
              if (const auto tie =
                      scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(maize_entity).lock()) {
                const auto& probes = tie->PeekProbes(); 
                const auto start_index = particle_infos.size();
                particle_infos.resize(start_index + probes.light_probes.size());
                for (int i = 0; i < probes.light_probes.size(); i++) {
                  const auto& probe = probes.light_probes.at(i);
                  auto& matrix = particle_infos[start_index + i].instance_matrix;
                  matrix.value = glm::translate(probe.GetCenter()) * glm::scale(glm::vec3(node_render_size));
                  particle_infos[start_index + i].instance_color =
                      glm::vec4(glm::vec3(glm::length(probe.energy) * energy_scale_factor), 1.0f);
                }
              }
            }
            probe_debug_info_list->SetParticleInfos(particle_infos);
          }
        }
        ImGui::Checkbox("Depth test", &depth_test);
        ImGui::TreePop();
      }

      GizmoSettings gizmo_settings{};
      gizmo_settings.depth_test = depth_test;
      editor_layer->DrawGizmoCubes(probe_debug_info_list, glm::mat4(1), 1, gizmo_settings);
    }

    ImGui::TreePop();
  }
  ImGui::Checkbox("Enable BTF", &enable_compressed_btf);
  if (enable_compressed_btf) {
    editor_layer->DragAndDropButton<CBTFGroup>(leaf_cbtf_group, "Leaf CBTFGroup");
  }
#endif
  ImGui::Separator();
  maize_mesh_generator_settings.OnInspect(editor_layer);
  if (ImGui::Button("Generate mesh for all maizes")) {
    GenerateMeshForAllMaizes(maize_mesh_generator_settings);
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
      leaf_material.Get<Material>()->SetAlbedoTexture(leaf_albedo_texture.Get<Texture2D>());
      if (const std::vector<Entity>* maize_entities = scene->UnsafeGetPrivateComponentOwnersList<Maize>();
          maize_entities && !maize_entities->empty()) {
        for (const auto& maize_entity : *maize_entities) {
          for (const auto child : scene->GetChildren(maize_entity)) {
            if (scene->HasPrivateComponent<MeshRenderer>(child)) {
              scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock()->material.Get<Material>()->SetAlbedoTexture(
                  leaf_albedo_texture.Get<Texture2D>());
            }
          }
        }
      }
    }
  }

  if (editor_layer->DragAndDropButton<Texture2D>(leaf_normal_texture, "Replace Leaf Normal Texture")) {
    auto tex = leaf_normal_texture.Get<Texture2D>();
    if (tex) {
      leaf_material.Get<Material>()->SetNormalTexture(leaf_normal_texture.Get<Texture2D>());
      if (const std::vector<Entity>* maize_entities = scene->UnsafeGetPrivateComponentOwnersList<Maize>();
          maize_entities && !maize_entities->empty()) {
        for (const auto& maize_entity : *maize_entities) {
          for (const auto child : scene->GetChildren(maize_entity)) {
            if (scene->HasPrivateComponent<MeshRenderer>(child)) {
              scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock()->material.Get<Material>()->SetNormalTexture(
                  leaf_albedo_texture.Get<Texture2D>());
            }
          }
        }
      }
    }
  }

  FileUtils::SaveFile(
      "Export OBJ for all maizes", "3D Model", {".obj"},
      [this](const std::filesystem::path& path) {
        ExportAllMaizesModel(path.string());
      },
      false);

  static bool opened = false;
#ifdef CUDA_MODULE_PLUGIN
  if (processing && !opened) {
    ImGui::OpenPopup("Illumination Estimation");
    opened = true;
  }
  if (ImGui::BeginPopupModal("Illumination Estimation", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("Progress: ");
    const float fraction = 1.0f - static_cast<float>(processing_index) / processing_entities.size();
    const std::string text = std::to_string(static_cast<int>(fraction * 100.0f)) + "% - " +
                             std::to_string(processing_entities.size() - processing_index) + "/" +
                             std::to_string(processing_entities.size());
    ImGui::ProgressBar(fraction, ImVec2(240, 0), text.c_str());
    ImGui::SetItemDefaultFocus();
    ImGui::Text(("Estimation time for 1 plant: " + std::to_string(per_plant_calculation_time) + " seconds").c_str());
    if (ImGui::Button("Cancel") || processing == false) {
      processing = false;
      opened = false;
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
#endif
}

void MaizeLayer::ExportMaize(const Entity& maize, std::ofstream& of, unsigned& start_index) {
  const auto scene = Application::GetActiveScene();
  const std::string start = "#Maize\n";
  of.write(start.c_str(), start.size());
  of.flush();
  const auto position = scene->GetDataComponent<GlobalTransform>(maize).GetPosition();

  const auto stem_mesh = scene->GetOrSetPrivateComponent<MeshRenderer>(maize).lock()->mesh.Get<Mesh>();
  ObjExportHelper(position, stem_mesh, of, start_index);

  scene->ForEachDescendant(maize, [&](const Entity child) {
    if (!scene->HasPrivateComponent<MeshRenderer>(child))
      return;
    const auto leaf_mesh = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock()->mesh.Get<Mesh>();
    ObjExportHelper(position, leaf_mesh, of, start_index);
  });
}

void MaizeLayer::ObjExportHelper(glm::vec3 position, const std::shared_ptr<Mesh>& mesh, std::ofstream& of,
                                   unsigned& start_index) {
  if (mesh && !mesh->UnsafeGetTriangles().empty()) {
    std::string header = "#Vertices: " + std::to_string(mesh->GetVerticesAmount()) +
                         ", tris: " + std::to_string(mesh->GetTriangleAmount());
    header += "\n";
    of.write(header.c_str(), header.size());
    of.flush();
    std::string o = "o ";
    o += "[" + std::to_string(position.x) + "," + std::to_string(position.z) + "]" + "\n";
    of.write(o.c_str(), o.size());
    of.flush();
    std::string data;
#pragma region Data collection

    for (auto& i : mesh->UnsafeGetVertices()) {
      const auto& vertex_position = i.position;
      const auto& color = i.color;
      data += "v " + std::to_string(vertex_position.x + position.x) + " " +
              std::to_string(vertex_position.y + position.y) + " " + std::to_string(vertex_position.z + position.z) +
              " " + std::to_string(color.x) + " " + std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
    }
    for (const auto& vertex : mesh->UnsafeGetVertices()) {
      data += "vn " + std::to_string(vertex.normal.x) + " " + std::to_string(vertex.normal.y) + " " +
              std::to_string(vertex.normal.z) + "\n";
    }

    for (const auto& vertex : mesh->UnsafeGetVertices()) {
      data += "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
    }
    // data += "s off\n";
    data += "# List of indices for faces vertices, with (x, y, z).\n";
    const auto& triangles = mesh->UnsafeGetTriangles();
    for (auto i = 0; i < mesh->GetTriangleAmount(); i++) {
      const auto triangle = triangles[i];
      const auto f1 = triangle.x + start_index;
      const auto f2 = triangle.y + start_index;
      const auto f3 = triangle.z + start_index;
      data += "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
              std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " + std::to_string(f3) +
              "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
    }
    start_index += mesh->GetVerticesAmount();
#pragma endregion
    of.write(data.c_str(), data.size());
    of.flush();
  }
}

void MaizeLayer::ExportAllMaizesModel(const std::string& filename) const {
  std::ofstream of;
  of.open(filename, std::ofstream::out | std::ofstream::trunc);
  if (of.is_open()) {
    std::string start = "#Maize field, by Bosheng Li";
    start += "\n";
    of.write(start.c_str(), start.size());
    of.flush();
    const auto scene = GetScene();
    if (const std::vector<Entity>* maize_entities = scene->UnsafeGetPrivateComponentOwnersList<Maize>();
        maize_entities && !maize_entities->empty()) {
      unsigned start_index = 1;
      for (const auto& maize_entity : *maize_entities) {
        ExportMaize(maize_entity, of, start_index);
      }
    }
    of.close();
    EVOENGINE_LOG("Maizes saved as " + filename);
  } else {
    EVOENGINE_ERROR("Can't open file!");
  }
}

#ifdef CUDA_MODULE_PLUGIN
void MaizeLayer::CalculateIlluminationFrameByFrame() {
  const auto scene = GetScene();
  const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<Maize>();
  if (!owners)
    return;
  processing_entities.clear();

  processing_entities.insert(processing_entities.begin(), owners->begin(), owners->end());
  processing_index = processing_entities.size();
  processing = true;
}
void MaizeLayer::CalculateIllumination() {
  const auto scene = GetScene();
  const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<Maize>();
  if (!owners)
    return;
  processing_entities.clear();

  processing_entities.insert(processing_entities.begin(), owners->begin(), owners->end());
  processing_index = processing_entities.size();
  while (processing) {
    processing_index--;
    if (processing_index == -1) {
      processing = false;
    } else {
      const float timer = Times::Now();
      const auto estimator =
          scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(processing_entities[processing_index]).lock();
      estimator->PrepareLightProbeGroup();
      estimator->SampleLightProbeGroup(ray_properties, m_seed, push_distance);
    }
  }
}
#endif
void MaizeLayer::Update() {
  const auto scene = GetScene();
#ifdef CUDA_MODULE_PLUGIN
  if (processing) {
    processing_index--;
    if (processing_index == -1) {
      processing = false;
    } else {
      const float timer = Times::Now();
      const auto estimator =
          scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(processing_entities[processing_index]).lock();
      estimator->PrepareLightProbeGroup();
      estimator->SampleLightProbeGroup(ray_properties, m_seed, push_distance);
      per_plant_calculation_time = Times::Now() - timer;
    }
  }
#endif
}
