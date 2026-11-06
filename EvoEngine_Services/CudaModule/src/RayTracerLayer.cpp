#include "RayTracerLayer.hpp"
#include "Application.hpp"
#include "BasicPointCloudScanner.hpp"
#include "CudaSerializationAdapters.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalLightingResolver.hpp"
#include "EnvironmentalMap.hpp"
#include "InspectorRegistry.hpp"
#include "MeshRenderer.hpp"
#include "OptiXRayTracer.hpp"
#include "Particles.hpp"
#include "PrefabFieldIllumination.hpp"
#include "RayTracerCamera.hpp"
#include "Resources.hpp"
#include "SDKInspectionAdapters.hpp"
#include "Serialization.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "StrandsRenderer.hpp"
#include "Times.hpp"
#include "TriangleIlluminationEstimator.hpp"
#include "VolumetricCloudPresets.hpp"
#include "VoxelIlluminationEstimator.hpp"
using namespace evo_engine;

std::shared_ptr<RayTracerCamera> RayTracerLayer::ray_tracer_camera_;

namespace {
bool DrawViewportCloudSettings(VolumetricCloudSettings& s) {
  bool changed = ImGui::Checkbox("Volumetric clouds", &s.enabled);
  if (!s.enabled || !ImGui::TreeNodeEx("Cloud settings", ImGuiTreeNodeFlags_DefaultOpen))
    return changed;

  if (ImGui::Button("Broad cloud bank")) {
    ApplyCloudFormation(s, CloudFormation::BroadBank);
    changed = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Scattered clouds")) {
    ApplyCloudFormation(s, CloudFormation::Scattered);
    changed = true;
  }
  ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.38f);
  changed |= ImGui::SliderFloat("Cloud coverage", &s.coverage, 0.f, 1.f);
  changed |= ImGui::SliderFloat("Density", &s.density, 0.f, 10.f);
  changed |= ImGui::SliderFloat("Extinction scale", &s.extinction_scale, 0.00001f, 1.f, "%.5f",
                                ImGuiSliderFlags_Logarithmic);
  changed |= ImGui::DragFloat("Bottom altitude", &s.bottom_altitude, 1.f);
  changed |= ImGui::DragFloat("Top altitude", &s.top_altitude, 1.f);
  changed |= ImGui::SliderFloat("Cloud type (stratus to cumulus)", &s.cloud_type, 0.f, 1.f);

  if (ImGui::TreeNode("Shape and wind")) {
    changed |= ImGui::SliderFloat("Base noise scale", &s.base_noise_scale, 0.00001f, 10.f, "%.5f",
                                  ImGuiSliderFlags_Logarithmic);
    changed |= ImGui::SliderFloat("Detail noise scale", &s.detail_noise_scale, 0.00001f, 10.f, "%.5f",
                                  ImGuiSliderFlags_Logarithmic);
    changed |= ImGui::DragFloat("Curl strength", &s.curl_strength, 0.1f);
    float angle = glm::atan(s.wind_direction.y, s.wind_direction.x);
    if (ImGui::SliderAngle("Wind direction", &angle)) {
      s.wind_direction = {glm::cos(angle), glm::sin(angle)};
      changed = true;
    }
    changed |= ImGui::DragFloat("Wind speed", &s.wind_speed, 0.1f);
    ImGui::TextDisabled("Wind offsets are sampled at a fixed time for accumulation.");
    changed |= ImGui::Checkbox("Spherical atmosphere", &s.use_spherical_atmosphere);
    if (s.use_spherical_atmosphere)
      changed |= ImGui::DragFloat("Atmosphere radius", &s.atmosphere_radius, 10.f);
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Cloud lighting")) {
    changed |= ImGui::DragFloat("Sun lighting intensity", &s.lighting_intensity, 0.05f);
    changed |= ImGui::DragFloat("Ambient lighting strength", &s.ambient_lighting_strength, 0.01f);
    changed |= ImGui::SliderFloat("Phase anisotropy", &s.phase_anisotropy, -0.99f, 0.99f);
    changed |= ImGui::Checkbox("Surface shadow preview", &s.enable_cloud_shadows);
    if (s.enable_cloud_shadows) {
      changed |= ImGui::SliderFloat("Shadow strength", &s.cloud_shadow_strength, 0.f, 1.f);
      changed |= ImGui::SliderInt("Shadow steps", &s.cloud_shadow_step_count, 1, 64);
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Ray marching")) {
    changed |= ImGui::DragFloat("Maximum march distance", &s.max_march_distance, 1.f);
    changed |= ImGui::SliderInt("Primary steps", &s.primary_step_count, 1, 512);
    changed |= ImGui::SliderInt("Light steps", &s.light_step_count, 1, 128);
    changed |= ImGui::SliderFloat("Coarse step fraction", &s.coarse_step_fraction, 0.0001f, 1.f, "%.4f",
                                  ImGuiSliderFlags_Logarithmic);
    changed |= ImGui::SliderFloat("Fine step scale", &s.fine_step_scale, 0.01f, 1.f);
    changed |= ImGui::SliderInt("Empty steps before fallback", &s.empty_step_fallback_count, 1, 64);
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Diagnostics")) {
    changed |= ImGui::Checkbox("Debug visualization", &s.debug_visualization);
    if (s.debug_visualization) {
      const char* modes[] = {"Composite", "Mean density", "Transmittance", "March distance",
                             "Base shape", "Detail erosion", "Weather coverage"};
      changed |= ImGui::Combo("Debug view", &s.debug_mode, modes, IM_ARRAYSIZE(modes));
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Cloud resolution and history")) {
    const char* resolutions[] = {"Full", "Half", "Quarter"};
    int index = s.resolution_divisor == 4 ? 2 : s.resolution_divisor == 2 ? 1 : 0;
    if (ImGui::Combo("Cloud resolution", &index, resolutions, IM_ARRAYSIZE(resolutions))) {
      s.resolution_divisor = 1 << index;
      changed = true;
    }
    changed |= ImGui::Checkbox("Temporal reprojection", &s.enable_temporal_reprojection);
    if (s.enable_temporal_reprojection)
      changed |= ImGui::SliderFloat("History blend", &s.temporal_blend_factor, 0.f, 0.98f);
    ImGui::TreePop();
  }
  ImGui::PopItemWidth();
  ImGui::TreePop();
  return changed;
}

uint64_t MakeRayTracedInstanceKey(const Handle entity_handle, const Handle component_handle) {
  auto key = entity_handle.GetValue();
  key ^= component_handle.GetValue() + 0x9e3779b97f4a7c15ULL + (key << 6U) + (key >> 2U);
  return key;
}

bool InspectPrefabFieldWithIllumination(InspectorContext& context, PrefabField& field) {
  bool changed = DrawPrefabFieldGui(context, field);
  ImGui::Separator();
  ImGui::TextUnformatted("OptiX illumination");

  auto& settings = field.illumination_settings;
  changed = ImGui::DragInt("Illumination samples", &settings.samples, 1.0f, 1, 1000000) || changed;
  changed = ImGui::DragInt("Illumination bounces", &settings.bounces, 1.0f, 0, 100) || changed;
  changed = ImGui::DragInt("Illumination seed", &settings.seed) || changed;
  changed = ImGui::DragFloat("Normal offset", &settings.push_normal_distance, 0.00001f, 0.0f, 1.0f, "%.6f") || changed;
  settings.samples = std::max(settings.samples, 1);
  settings.bounces = std::max(settings.bounces, 0);
  settings.push_normal_distance = std::max(settings.push_normal_distance, 0.0f);

  if (ImGui::Button("Measure plants in ROI")) {
    field.last_illumination = EstimatePrefabFieldIllumination(field, settings);
  }

  const auto& summary = field.last_illumination;
  ImGui::Text("Measured plants: %zu", summary.models.size());
  ImGui::Text("Total surface area: %.6g", summary.total_area);
  ImGui::Text("Total flux RGB: %.6g, %.6g, %.6g", summary.total_flux.x, summary.total_flux.y, summary.total_flux.z);
  ImGui::Text("Area-weighted average RGB: %.6g, %.6g, %.6g", summary.area_weighted_average_flux.x,
              summary.area_weighted_average_flux.y, summary.area_weighted_average_flux.z);
  ImGui::Text("Plant-mean average RGB: %.6g, %.6g, %.6g", summary.mean_model_average_flux.x,
              summary.mean_model_average_flux.y, summary.mean_model_average_flux.z);

  if (ImGui::TreeNode("Per-organ results (field)")) {
    for (const auto& [name, organ] : summary.organs) {
      ImGui::Text("%s: area %.6g", name.c_str(), organ.total_area);
      ImGui::Text("  Total incident flux RGB: %.6g, %.6g, %.6g", organ.total_flux.x, organ.total_flux.y,
                  organ.total_flux.z);
      ImGui::Text("  Average RGB: %.6g, %.6g, %.6g", organ.average_flux.x, organ.average_flux.y, organ.average_flux.z);
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Per-organ results (plants)")) {
    for (const auto& model : summary.models) {
      ImGui::PushID(static_cast<int>(model.model_index));
      if (ImGui::TreeNode("Plant", "Plant %zu (row %d, column %d)", model.model_index, model.row, model.column)) {
        for (const auto& [name, organ] : model.organs) {
          ImGui::Text("%s: area %.6g, average RGB %.6g, %.6g, %.6g", name.c_str(), organ.total_area,
                      organ.average_flux.x, organ.average_flux.y, organ.average_flux.z);
          ImGui::Text("  Total incident flux RGB: %.6g, %.6g, %.6g", organ.total_flux.x, organ.total_flux.y,
                      organ.total_flux.z);
        }
        ImGui::TreePop();
      }
      ImGui::PopID();
    }
    ImGui::TreePop();
  }

  if (!summary.models.empty() && ImGui::TreeNode("Per-plant results")) {
    if (ImGui::BeginTable("PrefabFieldIlluminationResults", 8,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable)) {
      ImGui::TableSetupColumn("Index");
      ImGui::TableSetupColumn("Asset");
      ImGui::TableSetupColumn("Row");
      ImGui::TableSetupColumn("Column");
      ImGui::TableSetupColumn("Area");
      ImGui::TableSetupColumn("Average R");
      ImGui::TableSetupColumn("Average G");
      ImGui::TableSetupColumn("Average B");
      ImGui::TableHeadersRow();
      for (const auto& model : summary.models) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("%zu", model.model_index);
        ImGui::TableNextColumn();
        ImGui::Text("%zu", model.asset_index);
        ImGui::TableNextColumn();
        ImGui::Text("%d", model.row);
        ImGui::TableNextColumn();
        ImGui::Text("%d", model.column);
        ImGui::TableNextColumn();
        ImGui::Text("%.6g", model.total_area);
        ImGui::TableNextColumn();
        ImGui::Text("%.6g", model.average_flux.x);
        ImGui::TableNextColumn();
        ImGui::Text("%.6g", model.average_flux.y);
        ImGui::TableNextColumn();
        ImGui::Text("%.6g", model.average_flux.z);
      }
      ImGui::EndTable();
    }
    ImGui::TreePop();
  }
  return changed;
}
}  // namespace

void RayTracerLayer::UpdateMeshesStorage(const std::shared_ptr<Scene>& scene,
                                         std::unordered_map<uint64_t, RayTracedMaterial>& material_storage,
                                         std::unordered_map<uint64_t, RayTracedGeometry>& geometry_storage,
                                         std::unordered_map<uint64_t, RayTracedInstance>& instance_storage,
                                         bool& rebuild_instances, bool& update_shader_binding_table) const {
  for (auto& i : instance_storage)
    i.second.remove_flag = true;
  for (auto& i : geometry_storage)
    i.second.remove_flag = true;
  for (auto& i : material_storage)
    i.second.remove_flag = true;

  if (const auto* ray_traced_entities = scene->UnsafeGetPrivateComponentOwnersList<StrandsRenderer>();
      ray_traced_entities && render_strands_renderer) {
    for (auto entity : *ray_traced_entities) {
      if (!scene->IsEntityEnabled(entity))
        continue;
      auto strands_renderer_renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(entity).lock();
      if (!strands_renderer_renderer->IsEnabled())
        continue;
      auto strands = strands_renderer_renderer->strands.Get<Strands>();
      auto material = strands_renderer_renderer->material.Get<Material>();
      if (!material || !strands || strands->UnsafeGetStrandPoints().empty() || strands->UnsafeGetSegments().empty())
        continue;
      auto global_transform = scene->GetDataComponent<GlobalTransform>(entity).value;
      bool need_instance_update = false;
      bool need_material_update = false;

      auto entity_handle = scene->GetEntityHandle(entity);
      auto geometry_handle = strands->GetHandle();
      auto material_handle = material->GetHandle();
      auto& ray_traced_instance =
          instance_storage[MakeRayTracedInstanceKey(entity_handle, strands_renderer_renderer->GetHandle())];
      auto& ray_traced_geometry = geometry_storage[geometry_handle];
      auto& ray_traced_material = material_storage[material_handle];
      ray_traced_instance.remove_flag = false;
      ray_traced_material.remove_flag = false;
      ray_traced_geometry.remove_flag = false;

      if (ray_traced_instance.entity_handle != entity_handle ||
          ray_traced_instance.private_component_handle != strands_renderer_renderer->GetHandle().GetValue() ||
          ray_traced_instance.version != strands_renderer_renderer->GetVersion() ||
          global_transform != ray_traced_instance.global_transform) {
        need_instance_update = true;
      }
      if (ray_traced_geometry.handle == 0 || ray_traced_geometry.version != strands->GetVersion()) {
        ray_traced_geometry.update_flag = true;
        need_instance_update = true;
        ray_traced_geometry.renderer_type = RendererType::Curve;
        ray_traced_geometry.curve_segments = &strands->UnsafeGetSegments();
        ray_traced_geometry.curve_points = &strands->UnsafeGetStrandPoints();
        ray_traced_geometry.version = strands->GetVersion();
        ray_traced_geometry.geometry_type = PrimitiveType::CubicBSpline;
        ray_traced_geometry.handle = geometry_handle;
      }
      if (CheckMaterial(ray_traced_material, material))
        need_instance_update = true;
      if (need_instance_update) {
        ray_traced_instance.entity_handle = entity_handle;
        ray_traced_instance.private_component_handle = strands_renderer_renderer->GetHandle().GetValue();
        ray_traced_instance.version = strands_renderer_renderer->GetVersion();
        ray_traced_instance.global_transform = global_transform;
        ray_traced_instance.geometry_map_key = geometry_handle;
        ray_traced_instance.material_map_key = material_handle;
      }
      update_shader_binding_table = update_shader_binding_table || need_material_update;
      rebuild_instances = rebuild_instances || need_instance_update;
    }
  }
  if (const auto* ray_traced_entities = scene->UnsafeGetPrivateComponentOwnersList<MeshRenderer>();
      ray_traced_entities && render_mesh_renderer) {
    for (auto entity : *ray_traced_entities) {
      if (!scene->IsEntityEnabled(entity))
        continue;
      auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
      if (!mesh_renderer->IsEnabled())
        continue;
      auto mesh = mesh_renderer->mesh.Get<Mesh>();
      auto material = mesh_renderer->material.Get<Material>();
      if (!material || !mesh || mesh->UnsafeGetVertices().empty())
        continue;
      auto global_transform = scene->GetDataComponent<GlobalTransform>(entity).value;
      bool need_instance_update = false;
      bool need_material_update = false;

      auto entity_handle = scene->GetEntityHandle(entity);
      auto geometry_handle = mesh->GetHandle();
      auto material_handle = material->GetHandle();
      auto& ray_traced_instance = instance_storage[MakeRayTracedInstanceKey(entity_handle, mesh_renderer->GetHandle())];
      auto& ray_traced_geometry = geometry_storage[geometry_handle];
      auto& ray_traced_material = material_storage[material_handle];
      ray_traced_instance.remove_flag = false;
      ray_traced_material.remove_flag = false;
      ray_traced_geometry.remove_flag = false;

      if (ray_traced_instance.entity_handle != entity_handle ||
          ray_traced_instance.private_component_handle != mesh_renderer->GetHandle().GetValue() ||
          ray_traced_instance.version != mesh_renderer->GetVersion() ||
          global_transform != ray_traced_instance.global_transform) {
        need_instance_update = true;
      }
      if (ray_traced_geometry.handle == 0 || ray_traced_geometry.version != mesh->GetVersion()) {
        ray_traced_geometry.update_flag = true;
        need_instance_update = true;
        ray_traced_geometry.renderer_type = RendererType::Default;
        ray_traced_geometry.triangles = &mesh->UnsafeGetTriangles();
        ray_traced_geometry.vertices = &mesh->UnsafeGetVertices();
        ray_traced_geometry.version = mesh->GetVersion();
        ray_traced_geometry.geometry_type = PrimitiveType::Triangle;
        ray_traced_geometry.handle = geometry_handle;
      }
      if (CheckMaterial(ray_traced_material, material))
        need_instance_update = true;
      if (need_instance_update) {
        ray_traced_instance.entity_handle = entity_handle;
        ray_traced_instance.private_component_handle = mesh_renderer->GetHandle().GetValue();
        ray_traced_instance.version = mesh_renderer->GetVersion();
        ray_traced_instance.global_transform = global_transform;
        ray_traced_instance.geometry_map_key = geometry_handle;
        ray_traced_instance.material_map_key = material_handle;
      }
      update_shader_binding_table = update_shader_binding_table || need_material_update;
      rebuild_instances = rebuild_instances || need_instance_update;
    }
  }
  if (const auto* ray_traced_entities = scene->UnsafeGetPrivateComponentOwnersList<SkinnedMeshRenderer>();
      ray_traced_entities && render_skinned_mesh_renderer) {
    for (auto entity : *ray_traced_entities) {
      if (!scene->IsEntityEnabled(entity))
        continue;
      auto skinned_mesh_renderer = scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(entity).lock();
      if (!skinned_mesh_renderer->IsEnabled())
        continue;
      auto mesh = skinned_mesh_renderer->skinned_mesh.Get<SkinnedMesh>();
      auto material = skinned_mesh_renderer->material.Get<Material>();
      if (!material || !mesh || mesh->UnsafeGetSkinnedVertices().empty() ||
          skinned_mesh_renderer->bone_matrices->value.empty())
        continue;
      auto global_transform =
          skinned_mesh_renderer->RagDoll() ? glm::mat4(1.0f) : scene->GetDataComponent<GlobalTransform>(entity).value;
      bool need_instance_update = false;
      bool need_material_update = false;

      auto entity_handle = scene->GetEntityHandle(entity);
      auto geometry_handle = skinned_mesh_renderer->GetHandle().GetValue();
      auto material_handle = material->GetHandle();
      auto& ray_traced_instance =
          instance_storage[MakeRayTracedInstanceKey(entity_handle, skinned_mesh_renderer->GetHandle())];
      auto& ray_traced_geometry = geometry_storage[geometry_handle];
      auto& ray_traced_material = material_storage[material_handle];
      ray_traced_instance.remove_flag = false;
      ray_traced_material.remove_flag = false;
      ray_traced_geometry.remove_flag = false;

      if (ray_traced_instance.entity_handle != entity_handle ||
          ray_traced_instance.private_component_handle != skinned_mesh_renderer->GetHandle().GetValue() ||
          ray_traced_instance.version != skinned_mesh_renderer->GetVersion() ||
          global_transform != ray_traced_instance.global_transform) {
        need_instance_update = true;
      }

      if (ray_traced_geometry.handle == 0 || ray_traced_instance.version != skinned_mesh_renderer->GetVersion() ||
          ray_traced_geometry.version != mesh->GetVersion() ||
          ray_traced_instance.data_version != skinned_mesh_renderer->bone_matrices->GetVersion() || true) {
        ray_traced_geometry.update_flag = true;
        need_instance_update = true;
        ray_traced_geometry.geometry_type = PrimitiveType::Triangle;
        ray_traced_geometry.renderer_type = RendererType::Skinned;
        ray_traced_geometry.triangles = &mesh->UnsafeGetTriangles();
        ray_traced_geometry.skinned_vertices = &mesh->UnsafeGetSkinnedVertices();
        ray_traced_geometry.bone_matrices = &skinned_mesh_renderer->bone_matrices->value;
        ray_traced_geometry.version = mesh->GetVersion();
        ray_traced_instance.data_version = skinned_mesh_renderer->bone_matrices->GetVersion();
        ray_traced_geometry.handle = geometry_handle;
      }
      if (CheckMaterial(ray_traced_material, material))
        need_instance_update = true;
      if (need_instance_update) {
        ray_traced_instance.entity_handle = entity_handle;
        ray_traced_instance.private_component_handle = skinned_mesh_renderer->GetHandle().GetValue();
        ray_traced_instance.version = skinned_mesh_renderer->GetVersion();
        ray_traced_instance.global_transform = global_transform;
        ray_traced_instance.geometry_map_key = geometry_handle;
        ray_traced_instance.material_map_key = material_handle;
      }
      update_shader_binding_table = update_shader_binding_table || need_material_update;
      rebuild_instances = rebuild_instances || need_instance_update;
    }
  }
  if (const auto* ray_traced_entities = scene->UnsafeGetPrivateComponentOwnersList<Particles>();
      ray_traced_entities && render_particles) {
    for (auto entity : *ray_traced_entities) {
      if (!scene->IsEntityEnabled(entity))
        continue;
      auto particles = scene->GetOrSetPrivateComponent<Particles>(entity).lock();
      if (!particles->IsEnabled())
        continue;
      auto mesh = particles->mesh.Get<Mesh>();
      auto material = particles->material.Get<Material>();
      auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
      if (!material || !mesh || !particle_info_list || mesh->UnsafeGetVertices().empty() ||
          particle_info_list->PeekParticleInfoList().empty())
        continue;
      auto global_transform = scene->GetDataComponent<GlobalTransform>(entity).value;
      bool need_instance_update = false;
      bool need_material_update = false;

      auto entity_handle = scene->GetEntityHandle(entity);
      auto geometry_handle = particles->GetHandle().GetValue();
      auto material_handle = material->GetHandle();
      auto& ray_traced_instance = instance_storage[MakeRayTracedInstanceKey(entity_handle, particles->GetHandle())];
      auto& ray_traced_geometry = geometry_storage[geometry_handle];
      auto& ray_traced_material = material_storage[material_handle];
      ray_traced_instance.remove_flag = false;
      ray_traced_material.remove_flag = false;
      ray_traced_geometry.remove_flag = false;

      if (ray_traced_instance.entity_handle != entity_handle ||
          ray_traced_instance.private_component_handle != particles->GetHandle().GetValue() ||
          ray_traced_instance.version != particles->GetVersion() ||
          ray_traced_instance.data_version != particle_info_list->GetVersion() ||
          global_transform != ray_traced_instance.global_transform) {
        need_instance_update = true;
      }
      if (need_instance_update || ray_traced_geometry.handle == 0 ||
          ray_traced_instance.version != particles->GetVersion() || ray_traced_geometry.version != mesh->GetVersion()) {
        ray_traced_geometry.update_flag = true;
        need_instance_update = true;
        ray_traced_geometry.geometry_type = PrimitiveType::Triangle;
        ray_traced_geometry.renderer_type = RendererType::Instanced;
        ray_traced_geometry.triangles = &mesh->UnsafeGetTriangles();
        ray_traced_geometry.vertices = &mesh->UnsafeGetVertices();
        auto* pointer = &(particle_info_list->PeekParticleInfoList());
        ray_traced_geometry.instance_matrices = (std::vector<InstanceMatrix>*)(pointer);
        ray_traced_geometry.version = mesh->GetVersion();
        ray_traced_geometry.handle = geometry_handle;
        ray_traced_instance.data_version = particle_info_list->GetVersion();
      }
      if (CheckMaterial(ray_traced_material, material))
        need_instance_update = true;
      if (need_instance_update) {
        ray_traced_instance.entity_handle = entity_handle;
        ray_traced_instance.private_component_handle = particles->GetHandle().GetValue();
        ray_traced_instance.version = particles->GetVersion();
        ray_traced_instance.global_transform = global_transform;
        ray_traced_instance.geometry_map_key = geometry_handle;
        ray_traced_instance.material_map_key = material_handle;
      }
      update_shader_binding_table = update_shader_binding_table || need_material_update;
      rebuild_instances = rebuild_instances || need_instance_update;
    }
  }
  for (auto& i : instance_storage)
    if (i.second.remove_flag)
      rebuild_instances = true;
}

bool RayTracerLayer::UpdateScene(const std::shared_ptr<Scene>& scene) {
  bool rebuild_acceleration_structure = false;
  bool update_shader_binding_table = false;
  auto& instance_storage = CudaModule::GetRayTracer()->instances;
  auto& material_storage = CudaModule::GetRayTracer()->materials;
  auto& geometry_storage = CudaModule::GetRayTracer()->geometries;
  UpdateMeshesStorage(scene, material_storage, geometry_storage, instance_storage, rebuild_acceleration_structure,
                      update_shader_binding_table);
  if (environment_properties.environmental_lighting_type == EnvironmentalLightingType::Scene) {
    const auto lighting = ResolveEnvironmentalLighting(scene);
    const auto& source = lighting.indirect_environment_source;
    const bool use_env_map =
        source.kind == ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::EnvironmentalMap;
    auto environmental_map_ref = source.environmental_map;
    const auto requested_environmental_map = environmental_map_ref.GetAssetHandle();
    auto env_map = environmental_map_ref.Get<EnvironmentalMap>();
    if (!env_map && use_env_map) {
      env_map = Resources::GetInstance().GetDefaultEnvironmentalMap();
    }
    if (env_map) {
      env_map->EnsureEnvironmentSource();
    }
    // Incoming radiance must use the unfiltered source, not a scene-baked reflection probe.
    const auto source_cubemap = env_map ? env_map->environment_cubemap.Get<Cubemap>() : nullptr;
    // Handle{} generates a new ID; a missing source must keep a stable cache key across frames.
    const Handle requested_cubemap = source_cubemap ? source_cubemap->GetHandle() : Handle(0);
    if (environmental_map_handle != requested_environmental_map || environment_cubemap_handle != requested_cubemap) {
      environmental_map_handle = requested_environmental_map;
      environment_cubemap_handle = requested_cubemap;
      environment_properties.environmental_map = 0;
      environmental_map_image.reset();
      update_shader_binding_table = true;
    }
    if (use_env_map && !environmental_map_image) {
      const auto imported = source_cubemap ? CudaModule::ImportCubemap(source_cubemap) : nullptr;
      if (imported) {
        environmental_map_image = imported;
        environment_properties.environmental_map = imported->texture_object;
      }
      update_shader_binding_table = true;
    }
    const bool use_imported_env_map = use_env_map && environmental_map_image;
    if (environment_properties.use_environmental_map != use_imported_env_map) {
      environment_properties.use_environmental_map = use_imported_env_map;
      update_shader_binding_table = true;
    }
    const glm::vec3 background_color =
        source.kind == ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::Color
            ? source.color
            : environment_properties.color;
    if (background_color != environment_properties.color) {
      environment_properties.color = background_color;
      update_shader_binding_table = true;
    }
    const float skylight_intensity = std::max(lighting.environment_lighting_intensity, 0.0f);
    if (environment_properties.skylight_intensity != skylight_intensity) {
      environment_properties.skylight_intensity = skylight_intensity;
      update_shader_binding_table = true;
    }
    if (environment_properties.environment_rotation != source.rotation) {
      environment_properties.environment_rotation = source.rotation;
      update_shader_binding_table = true;
    }
    if (environment_properties.gamma != source.gamma) {
      environment_properties.gamma = source.gamma;
      update_shader_binding_table = true;
    }
  }

  CudaModule::GetRayTracer()->scene_modified = false;
  if (rebuild_acceleration_structure && !instance_storage.empty()) {
    CudaModule::GetRayTracer()->BuildIas();
    return true;
  }
  if (update_shader_binding_table) {
    CudaModule::GetRayTracer()->scene_modified = true;
    return true;
  }
  return false;
}

void RayTracerLayer::RegisterTypes(Application& application) {
  application.RegisterPrivateComponent<TriangleIlluminationEstimator>("TriangleIlluminationEstimator");
  application.RegisterPrivateComponent<VoxelIlluminationEstimator>("VoxelIlluminationEstimator");
  application.RegisterPrivateComponent<RayTracerCamera>("RayTracerCamera");
  application.RegisterPrivateComponent<BasicPointCloudScanner>("BasicPointCloudScanner");
  Serialization::RegisterSerializationHandler<TriangleIlluminationEstimator>(SerializeTriangleIlluminationEstimator,
                                                                             DeserializeTriangleIlluminationEstimator,
                                                                             {}, "TriangleIlluminationEstimator");
  Serialization::RegisterSerializationHandler<VoxelIlluminationEstimator>(
      SerializeVoxelIlluminationEstimator, DeserializeVoxelIlluminationEstimator, {}, "VoxelIlluminationEstimator");
  Serialization::RegisterSerializationHandler<RayTracerCamera>(SerializeRayTracerCamera, DeserializeRayTracerCamera, {},
                                                               "RayTracerCamera");
  Serialization::RegisterSerializationHandler<BasicPointCloudScanner>(
      SerializeBasicPointCloudScanner, DeserializeBasicPointCloudScanner, {}, "BasicPointCloudScanner");
  InspectorRegistry::GetInstance().RegisterInspector<TriangleIlluminationEstimator>(
      [](InspectorContext& context, TriangleIlluminationEstimator& estimator) {
        return estimator.DrawGui(context.editor_layer);
      },
      {}, "TriangleIlluminationEstimator");
  InspectorRegistry::GetInstance().RegisterInspector<VoxelIlluminationEstimator>(
      [](InspectorContext& context, VoxelIlluminationEstimator& estimator) {
        return estimator.DrawGui(context.editor_layer);
      },
      {}, "VoxelIlluminationEstimator");
  InspectorRegistry::GetInstance().RegisterInspector<PrefabField>(InspectPrefabFieldWithIllumination, {},
                                                                  "PrefabField");
  InspectorRegistry::GetInstance().RegisterInspector<RayTracerCamera>(
      [](InspectorContext& context, RayTracerCamera& camera) {
        return camera.DrawGui(context.editor_layer);
      },
      {}, "RayTracerCamera");
  InspectorRegistry::GetInstance().RegisterInspector<BasicPointCloudScanner>(
      [](InspectorContext& context, BasicPointCloudScanner& scanner) {
        return scanner.DrawGui(context.editor_layer);
      },
      {}, "BasicPointCloudScanner");
  InspectorRegistry::GetInstance().RegisterInspector<RayTracerLayer>(
      [](InspectorContext& context, RayTracerLayer& layer) {
        layer.DrawGui(context.editor_layer);
        return false;
      },
      {}, "RayTracerLayer");
}

void RayTracerLayer::SetCloudSettings(VolumetricCloudSettings settings, bool affect_illumination_probes) {
  settings.ClampSettings();
  viewport_clouds = settings;
  clouds_affect_illumination = affect_illumination_probes;
  if (scene_camera)
    scene_camera->camera_properties_.modified = true;
}

void RayTracerLayer::OnCreate() {
  CudaModule::Init();

  scene_camera = Serialization::ProduceSerializable<RayTracerCamera>();
  scene_camera->OnCreate();
  viewport_clouds.enabled = true;
  viewport_clouds.resolution_divisor = 1;
  viewport_clouds.enable_temporal_reprojection = false;
  viewport_clouds.enable_cloud_shadows = false;
  ApplicationContext::Get().RegisterPostAttachSceneFunction([&](const std::shared_ptr<Scene>& scene) {
    ray_tracer_camera_.reset();
  });
}

void RayTracerLayer::PreUpdate() {
  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
      show_scene_window && editor_layer && rendering_enabled) {
    scene_camera->Ready(editor_layer->GetSceneCameraPosition(), editor_layer->GetSceneCameraRotation());
  }
}

void RayTracerLayer::LateUpdate() {
  const auto scene = GetScene();
  if (!scene)
    return;
  bool ray_tracer_updated = UpdateScene(scene);
  if (!CudaModule::GetRayTracer()->instances.empty()) {
    if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
        show_scene_window && editor_layer && rendering_enabled) {
      scene_camera->rendered_ = CudaModule::GetRayTracer()->RenderToCamera(
          environment_properties, scene_camera->camera_properties_, scene_camera->ray_properties, viewport_clouds);
    }
    const auto* entities = scene->UnsafeGetPrivateComponentOwnersList<RayTracerCamera>();
    ray_tracer_camera_.reset();
    if (entities) {
      bool check = false;
      for (const auto& entity : *entities) {
        if (!scene->IsEntityEnabled(entity))
          continue;
        const auto ray_tracer_camera = scene->GetOrSetPrivateComponent<RayTracerCamera>(entity).lock();
        if (!ray_tracer_camera->IsEnabled())
          continue;
        auto global_transform = scene->GetDataComponent<GlobalTransform>(ray_tracer_camera->GetOwner()).value;
        ray_tracer_camera->Ready(global_transform[3], glm::quat_cast(global_transform));
        ray_tracer_camera->rendered_ = CudaModule::GetRayTracer()->RenderToCamera(
            environment_properties, ray_tracer_camera->camera_properties_, ray_tracer_camera->ray_properties);

        if (!check) {
          if (ray_tracer_camera->main_camera_) {
            ray_tracer_camera_ = ray_tracer_camera;
            check = true;
          }
        } else {
          ray_tracer_camera->main_camera_ = false;
        }
      }
    }
  }
}

void RayTracerLayer::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (show_camera_window)
    RayCameraWindow();
  if (show_scene_window)
    SceneCameraWindow();

  if (!ImGui::Begin(GetLayerName().c_str(), &enable_inspection)) {
    ImGui::End();
    return;
  }
  if (ImGui::TreeNode("Editor")) {
    if (ImGui::TreeNode("Scene")) {
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Camera")) {
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }

  ImGui::Checkbox("Scene (RT) Window", &show_scene_window);
  if (show_scene_window)
    ImGui::Checkbox("Scene (RT) Window Info", &show_scene_info);
  ImGui::Checkbox("Clouds affect illumination probes", &clouds_affect_illumination);
  if ((show_scene_window || clouds_affect_illumination) && DrawViewportCloudSettings(viewport_clouds))
    SetCloudSettings(viewport_clouds, clouds_affect_illumination);
  ImGui::Checkbox("Camera (RT) Window", &show_camera_window);

  ImGui::Checkbox("Mesh Renderer", &render_mesh_renderer);
  ImGui::Checkbox("Strand Renderer", &render_strands_renderer);
  ImGui::Checkbox("Particles", &render_particles);
  ImGui::Checkbox("Skinned Mesh Renderer", &render_skinned_mesh_renderer);

  if (ImGui::TreeNode("Scene Camera Settings")) {
    scene_camera->DrawGui(editor_layer);
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Environment Properties", ImGuiTreeNodeFlags_DefaultOpen)) {
    environment_properties.DrawGui();
    ImGui::TreePop();
  }

  ImGui::End();
}

void RayTracerLayer::OnDestroy() {
  environmental_map_image.reset();
  scene_camera.reset();
  CudaModule::Terminate();
}

void RayTracerLayer::SceneCameraWindow() {
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  if (!editor_layer)
    return;
  auto scene_camera_rotation = editor_layer->GetSceneCameraRotation();
  auto scene_camera_position = editor_layer->GetSceneCameraPosition();
  const auto scene = GetScene();
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
  if (focus_scene_window) {
    ImGui::SetNextWindowFocus();
    focus_scene_window = false;
  }
  ImGui::SetNextWindowSize(ImVec2(640, 400), ImGuiCond_FirstUseEver);
  if (ImGui::Begin("Scene (RT)")) {
    if (ImGui::BeginChild("RaySceneRenderer", ImVec2(0, 0), false)) {
      static int corner = 1;
      const ImVec2 overlay_pos = ImGui::GetWindowPos();

      ImVec2 view_port_size = ImGui::GetWindowSize();
      scene_camera_resolution_ = glm::ivec2(view_port_size.x, view_port_size.y);
      if (scene_camera->allow_auto_resize)
        scene_camera->frame_size = glm::vec2(view_port_size.x, view_port_size.y) * resolution_multiplier;
      if (scene_camera->rendered_) {
        ImGui::Image(scene_camera->render_texture->GetColorImTextureId(), ImVec2(view_port_size.x, view_port_size.y),
                     ImVec2(0, 1), ImVec2(1, 0));
        editor_layer->CameraWindowDragAndDrop();
      } else
        ImGui::Text("No mesh in the scene!");

      const auto window_pos = ImVec2((corner & 1) ? (overlay_pos.x + view_port_size.x) : (overlay_pos.x),
                                     (corner & 2) ? (overlay_pos.y + view_port_size.y) : (overlay_pos.y));
      if (show_scene_info) {
        const auto window_pos_pivot = ImVec2((corner & 1) ? 1.0f : 0.0f, (corner & 2) ? 1.0f : 0.0f);
        ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
        ImGui::SetNextWindowBgAlpha(0.35f);
        constexpr ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoDocking |
                                                  ImGuiWindowFlags_NoSavedSettings |
                                                  ImGuiWindowFlags_NoFocusOnAppearing;
        if (constexpr ImGuiChildFlags child_flags = ImGuiChildFlags_None;
            ImGui::BeginChild("Info", ImVec2(300, 300), child_flags, window_flags)) {
          ImGui::Text("Info & Settings");
          ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
          std::string draw_call_info = {};
          ImGui::PushItemWidth(100);
          ImGui::DragFloat("Resolution multiplier", &resolution_multiplier, 0.01f, 0.1f, 1.0f);
          scene_camera->camera_properties_.DrawGui();
          scene_camera->ray_properties.DrawGui();
          ImGui::PopItemWidth();
        }
        ImGui::EndChild();
      }

      auto mouse_position = glm::vec2(FLT_MAX, FLT_MIN);
      if (ImGui::IsWindowFocused()) {
        auto mp = ImGui::GetMousePos();
        auto wp = ImGui::GetWindowPos();
        mouse_position = glm::vec2(mp.x - wp.x, mp.y - wp.y);
        static bool is_dragging_previously = false;
        bool mouse_drag = true;
#pragma region Scene Camera Controller
        if (mouse_position.x < 0 || mouse_position.y < 0 || mouse_position.x > view_port_size.x ||
            mouse_position.y > view_port_size.y ||
            editor_layer->GetKey(GLFW_MOUSE_BUTTON_RIGHT) != Input::KeyActionType::Hold) {
          mouse_drag = false;
        }
        static float prev_x = 0;
        static float prev_y = 0;
        if (mouse_drag && !is_dragging_previously) {
          prev_x = mouse_position.x;
          prev_y = mouse_position.y;
        }
        const float x_offset = mouse_position.x - prev_x;
        const float y_offset = mouse_position.y - prev_y;
        prev_x = mouse_position.x;
        prev_y = mouse_position.y;
        is_dragging_previously = mouse_drag;
        if (mouse_drag && !editor_layer->lock_camera) {
          glm::vec3 front = scene_camera_rotation * glm::vec3(0, 0, -1);
          const glm::vec3 right = scene_camera_rotation * glm::vec3(1, 0, 0);
          if (editor_layer->GetKey(GLFW_KEY_W) == Input::KeyActionType::Hold) {
            scene_camera_position +=
                front * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime()) * editor_layer->velocity;
          }
          if (editor_layer->GetKey(GLFW_KEY_S) == Input::KeyActionType::Hold) {
            scene_camera_position -=
                front * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime()) * editor_layer->velocity;
          }
          if (editor_layer->GetKey(GLFW_KEY_A) == Input::KeyActionType::Hold) {
            scene_camera_position -=
                right * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime()) * editor_layer->velocity;
          }
          if (editor_layer->GetKey(GLFW_KEY_D) == Input::KeyActionType::Hold) {
            scene_camera_position +=
                right * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime()) * editor_layer->velocity;
          }
          if (editor_layer->GetKey(GLFW_KEY_LEFT_SHIFT) == Input::KeyActionType::Hold) {
            scene_camera_position.y +=
                editor_layer->velocity * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime());
          }
          if (editor_layer->GetKey(GLFW_KEY_LEFT_CONTROL) == Input::KeyActionType::Hold) {
            scene_camera_position.y -=
                editor_layer->velocity * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime());
          }
          if (x_offset != 0.0f || y_offset != 0.0f) {
            front = glm::rotate(front, glm::radians(-x_offset * editor_layer->sensitivity), glm::vec3(0, 1, 0));
            const glm::vec3 right = glm::normalize(glm::cross(front, glm::vec3(0.0f, 1.0f, 0.0f)));
            if ((front.y < 0.99f && y_offset < 0.0f) || (front.y > -0.99f && y_offset > 0.0f)) {
              front = glm::rotate(front, glm::radians(-y_offset * editor_layer->sensitivity), right);
            }
            const glm::vec3 up = glm::normalize(glm::cross(right, front));
            scene_camera_rotation = glm::quatLookAt(front, up);
          }
          editor_layer->SetSceneCameraPosition(scene_camera_position);
          editor_layer->SetSceneCameraRotation(scene_camera_rotation);
        }
#pragma endregion
      }
    }
    ImGui::EndChild();
    auto* window = ImGui::FindWindowByName("Scene (RT)");
    rendering_enabled = !(window->Hidden && !window->Collapsed);
  }
  ImGui::End();
  ImGui::PopStyleVar();
}

void RayTracerLayer::RayCameraWindow() {
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  if (!editor_layer)
    return;
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
  if (ImGui::Begin("Camera (RT)")) {
    if (ImGui::BeginChild("RayCameraRenderer", ImVec2(0, 0), false, ImGuiWindowFlags_None)) {
      ImVec2 view_port_size = ImGui::GetWindowSize();
      if (ray_tracer_camera_) {
        if (ray_tracer_camera_->allow_auto_resize)
          ray_tracer_camera_->frame_size = glm::vec2(view_port_size.x, view_port_size.y);
        if (ray_tracer_camera_->rendered_) {
          ImGui::Image(ray_tracer_camera_->render_texture->GetColorImTextureId(),
                       ImVec2(view_port_size.x, view_port_size.y), ImVec2(0, 1), ImVec2(1, 0));
          editor_layer->CameraWindowDragAndDrop();
        } else
          ImGui::Text("No mesh in the scene!");
      } else {
        ImGui::Text("No camera attached!");
      }
    }
    ImGui::EndChild();
  }
  ImGui::End();
  ImGui::PopStyleVar();
}

bool RayTracerLayer::CheckMaterial(RayTracedMaterial& ray_tracer_material, const std::shared_ptr<Material>& material) {
  bool changed = false;
  if (ray_tracer_material.material_type == MaterialType::Default && material->vertex_color_only) {
    changed = true;
    ray_tracer_material.material_type = MaterialType::VertexColor;
  } else if (ray_tracer_material.material_type == MaterialType::VertexColor && !material->vertex_color_only) {
    changed = true;
    ray_tracer_material.material_type = MaterialType::Default;
  }

  if (changed || ray_tracer_material.version != material->GetVersion()) {
    ray_tracer_material.handle = material->GetHandle();
    ray_tracer_material.version = material->GetVersion();
    const auto& source = material->material_data.shade_material;
    auto& destination = ray_tracer_material.material_properties;
    destination.albedo_color = glm::vec3(source.pbr_base_color_factor);
    destination.metallic = source.pbr_metallic_factor;
    destination.roughness = source.pbr_roughness_factor;
#if MAT_EXT_SPECULAR
    destination.specular = source.specular_factor;
    destination.specular_tint =
        1.0f - std::min(source.specular_color_factor.x,
                        std::min(source.specular_color_factor.y, source.specular_color_factor.z));
#endif
#if MAT_EXT_CLEARCOAT
    destination.clear_coat = source.clearcoat_factor;
    destination.clear_coat_roughness = source.clearcoat_roughness;
#endif
#if MAT_EXT_IOR
    destination.ior = source.ior;
#endif
#if MAT_EXT_TRANSMISSION
    destination.transmission = source.transmission_factor;
#endif
#if MAT_EXT_SHEEN
    destination.sheen =
        std::max(source.sheen_color_factor.x, std::max(source.sheen_color_factor.y, source.sheen_color_factor.z));
    destination.sheen_tint = source.sheen_roughness_factor;
#endif
    destination.emission =
        std::max(source.emissive_factor.x, std::max(source.emissive_factor.y, source.emissive_factor.z));

    if (const auto albedo_texture = material->GetTexture(&GltfShadeMaterial::pbr_base_color_texture);
        albedo_texture && albedo_texture->GetVkImage() != VK_NULL_HANDLE) {
      ray_tracer_material.albedo_texture = CudaModule::ImportTexture2D(albedo_texture);
    } else {
      ray_tracer_material.albedo_texture = nullptr;
    }
    if (const auto normal_texture = material->GetTexture(&GltfShadeMaterial::normal_texture);
        normal_texture && normal_texture->GetVkImage() != VK_NULL_HANDLE) {
      ray_tracer_material.normal_texture = CudaModule::ImportTexture2D(normal_texture);
    } else {
      ray_tracer_material.normal_texture = nullptr;
    }
    if (const auto metallic_roughness_texture =
            material->GetTexture(&GltfShadeMaterial::pbr_metallic_roughness_texture);
        metallic_roughness_texture && metallic_roughness_texture->GetVkImage() != VK_NULL_HANDLE) {
      ray_tracer_material.roughness_texture = CudaModule::ImportTexture2D(metallic_roughness_texture);
      ray_tracer_material.metallic_texture = ray_tracer_material.roughness_texture;
    } else {
      ray_tracer_material.roughness_texture = nullptr;
      ray_tracer_material.metallic_texture = nullptr;
    }

    changed = true;
  }

  return changed;
}

glm::ivec2 RayTracerLayer::GetSceneCameraResolution() const {
  return scene_camera_resolution_;
}
