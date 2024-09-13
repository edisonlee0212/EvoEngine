#include "RayTracerLayer.hpp"
#include "BTFMeshRenderer.hpp"
#include "BasicPointCloudScanner.hpp"
#include "ClassRegistry.hpp"
#include "CompressedBTF.hpp"
#include "EditorLayer.hpp"
#include "MeshRenderer.hpp"
#include "Particles.hpp"
#include "OptiXRayTracer.hpp"
#include "RayTracerCamera.hpp"
#include "Resources.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "StrandsRenderer.hpp"
#include "Times.hpp"
#include "TriangleIlluminationEstimator.hpp"
using namespace evo_engine;

std::shared_ptr<RayTracerCamera> RayTracerLayer::ray_tracer_camera_;

void RayTracerLayer::UpdateMeshesStorage(const std::shared_ptr<Scene>& scene,
                                         std::unordered_map<uint64_t, RayTracedMaterial>& material_storage,
                                         std::unordered_map<uint64_t, RayTracedGeometry>& geometry_storage,
                                         std::unordered_map<uint64_t, RayTracedInstance>& instance_storage,
                                         bool& rebuild_instances, bool& update_shader_binding_table) const {
  for (auto& i : instance_storage)
    i.second.m_removeFlag = true;
  for (auto& i : geometry_storage)
    i.second.m_removeFlag = true;
  for (auto& i : material_storage)
    i.second.m_removeFlag = true;

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
      auto& ray_traced_instance = instance_storage[strands_renderer_renderer->GetHandle().GetValue()];
      auto& ray_traced_geometry = geometry_storage[geometry_handle];
      auto& ray_traced_material = material_storage[material_handle];
      ray_traced_instance.m_removeFlag = false;
      ray_traced_material.m_removeFlag = false;
      ray_traced_geometry.m_removeFlag = false;

      if (ray_traced_instance.m_entityHandle != entity_handle ||
          ray_traced_instance.m_privateComponentHandle != strands_renderer_renderer->GetHandle().GetValue() ||
          ray_traced_instance.m_version != strands_renderer_renderer->GetVersion() ||
          global_transform != ray_traced_instance.m_globalTransform) {
        need_instance_update = true;
      }
      if (ray_traced_geometry.m_handle == 0 || ray_traced_geometry.m_version != strands->GetVersion()) {
        ray_traced_geometry.m_updateFlag = true;
        need_instance_update = true;
        ray_traced_geometry.m_rendererType = RendererType::Curve;
        ray_traced_geometry.m_curveSegments = &strands->UnsafeGetSegments();
        ray_traced_geometry.m_curvePoints = &strands->UnsafeGetStrandPoints();
        ray_traced_geometry.m_version = strands->GetVersion();
        ray_traced_geometry.m_geometryType = PrimitiveType::CubicBSpline;
        ray_traced_geometry.m_handle = geometry_handle;
      }
      if (CheckMaterial(ray_traced_material, material))
        need_instance_update = true;
      if (need_instance_update) {
        ray_traced_instance.m_entityHandle = entity_handle;
        ray_traced_instance.m_privateComponentHandle = strands_renderer_renderer->GetHandle().GetValue();
        ray_traced_instance.m_version = strands_renderer_renderer->GetVersion();
        ray_traced_instance.m_globalTransform = global_transform;
        ray_traced_instance.m_geometryMapKey = geometry_handle;
        ray_traced_instance.m_materialMapKey = material_handle;
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
      auto& ray_traced_instance = instance_storage[mesh_renderer->GetHandle().GetValue()];
      auto& ray_traced_geometry = geometry_storage[geometry_handle];
      auto& ray_traced_material = material_storage[material_handle];
      ray_traced_instance.m_removeFlag = false;
      ray_traced_material.m_removeFlag = false;
      ray_traced_geometry.m_removeFlag = false;

      if (ray_traced_instance.m_entityHandle != entity_handle ||
          ray_traced_instance.m_privateComponentHandle != mesh_renderer->GetHandle().GetValue() ||
          ray_traced_instance.m_version != mesh_renderer->GetVersion() ||
          global_transform != ray_traced_instance.m_globalTransform) {
        need_instance_update = true;
      }
      if (ray_traced_geometry.m_handle == 0 || ray_traced_geometry.m_version != mesh->GetVersion()) {
        ray_traced_geometry.m_updateFlag = true;
        need_instance_update = true;
        ray_traced_geometry.m_rendererType = RendererType::Default;
        ray_traced_geometry.m_triangles = &mesh->UnsafeGetTriangles();
        ray_traced_geometry.m_vertices = &mesh->UnsafeGetVertices();
        ray_traced_geometry.m_version = mesh->GetVersion();
        ray_traced_geometry.m_geometryType = PrimitiveType::Triangle;
        ray_traced_geometry.m_handle = geometry_handle;
      }
      if (CheckMaterial(ray_traced_material, material))
        need_instance_update = true;
      if (need_instance_update) {
        ray_traced_instance.m_entityHandle = entity_handle;
        ray_traced_instance.m_privateComponentHandle = mesh_renderer->GetHandle().GetValue();
        ray_traced_instance.m_version = mesh_renderer->GetVersion();
        ray_traced_instance.m_globalTransform = global_transform;
        ray_traced_instance.m_geometryMapKey = geometry_handle;
        ray_traced_instance.m_materialMapKey = material_handle;
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
      auto& ray_traced_instance = instance_storage[geometry_handle];
      auto& ray_traced_geometry = geometry_storage[geometry_handle];
      auto& ray_traced_material = material_storage[material_handle];
      ray_traced_instance.m_removeFlag = false;
      ray_traced_material.m_removeFlag = false;
      ray_traced_geometry.m_removeFlag = false;

      if (ray_traced_instance.m_entityHandle != entity_handle ||
          ray_traced_instance.m_privateComponentHandle != skinned_mesh_renderer->GetHandle().GetValue() ||
          ray_traced_instance.m_version != skinned_mesh_renderer->GetVersion() ||
          global_transform != ray_traced_instance.m_globalTransform) {
        need_instance_update = true;
      }

      if (ray_traced_geometry.m_handle == 0 || ray_traced_instance.m_version != skinned_mesh_renderer->GetVersion() ||
          ray_traced_geometry.m_version != mesh->GetVersion() ||
          ray_traced_instance.m_dataVersion != skinned_mesh_renderer->bone_matrices->GetVersion() || true) {
        ray_traced_geometry.m_updateFlag = true;
        need_instance_update = true;
        ray_traced_geometry.m_geometryType = PrimitiveType::Triangle;
        ray_traced_geometry.m_rendererType = RendererType::Skinned;
        ray_traced_geometry.m_triangles = &mesh->UnsafeGetTriangles();
        ray_traced_geometry.m_skinnedVertices = &mesh->UnsafeGetSkinnedVertices();
        ray_traced_geometry.m_boneMatrices = &skinned_mesh_renderer->bone_matrices->value;
        ray_traced_geometry.m_version = mesh->GetVersion();
        ray_traced_instance.m_dataVersion = skinned_mesh_renderer->bone_matrices->GetVersion();
        ray_traced_geometry.m_handle = geometry_handle;
      }
      if (CheckMaterial(ray_traced_material, material))
        need_instance_update = true;
      if (need_instance_update) {
        ray_traced_instance.m_entityHandle = entity_handle;
        ray_traced_instance.m_privateComponentHandle = skinned_mesh_renderer->GetHandle().GetValue();
        ray_traced_instance.m_version = skinned_mesh_renderer->GetVersion();
        ray_traced_instance.m_globalTransform = global_transform;
        ray_traced_instance.m_geometryMapKey = geometry_handle;
        ray_traced_instance.m_materialMapKey = material_handle;
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
      auto& ray_traced_instance = instance_storage[geometry_handle];
      auto& ray_traced_geometry = geometry_storage[geometry_handle];
      auto& ray_traced_material = material_storage[material_handle];
      ray_traced_instance.m_removeFlag = false;
      ray_traced_material.m_removeFlag = false;
      ray_traced_geometry.m_removeFlag = false;

      if (ray_traced_instance.m_entityHandle != entity_handle ||
          ray_traced_instance.m_privateComponentHandle != particles->GetHandle().GetValue() ||
          ray_traced_instance.m_version != particles->GetVersion() ||
          ray_traced_instance.m_dataVersion != particle_info_list->GetVersion() ||
          global_transform != ray_traced_instance.m_globalTransform) {
        need_instance_update = true;
      }
      if (need_instance_update || ray_traced_geometry.m_handle == 0 ||
          ray_traced_instance.m_version != particles->GetVersion() ||
          ray_traced_geometry.m_version != mesh->GetVersion()) {
        ray_traced_geometry.m_updateFlag = true;
        need_instance_update = true;
        ray_traced_geometry.m_geometryType = PrimitiveType::Triangle;
        ray_traced_geometry.m_rendererType = RendererType::Instanced;
        ray_traced_geometry.m_triangles = &mesh->UnsafeGetTriangles();
        ray_traced_geometry.m_vertices = &mesh->UnsafeGetVertices();
        auto* pointer = &(particle_info_list->PeekParticleInfoList());
        ray_traced_geometry.m_instanceMatrices = (std::vector<InstanceMatrix>*)(pointer);
        ray_traced_geometry.m_version = mesh->GetVersion();
        ray_traced_geometry.m_handle = geometry_handle;
        ray_traced_instance.m_dataVersion = particle_info_list->GetVersion();
      }
      if (CheckMaterial(ray_traced_material, material))
        need_instance_update = true;
      if (need_instance_update) {
        ray_traced_instance.m_entityHandle = entity_handle;
        ray_traced_instance.m_privateComponentHandle = particles->GetHandle().GetValue();
        ray_traced_instance.m_version = particles->GetVersion();
        ray_traced_instance.m_globalTransform = global_transform;
        ray_traced_instance.m_geometryMapKey = geometry_handle;
        ray_traced_instance.m_materialMapKey = material_handle;
      }
      update_shader_binding_table = update_shader_binding_table || need_material_update;
      rebuild_instances = rebuild_instances || need_instance_update;
    }
  }
  if (const auto* ray_traced_entities = scene->UnsafeGetPrivateComponentOwnersList<BTFMeshRenderer>();
      ray_traced_entities && render_btf_mesh_renderer) {
    for (auto entity : *ray_traced_entities) {
      if (!scene->IsEntityEnabled(entity))
        continue;
      auto mesh_renderer = scene->GetOrSetPrivateComponent<BTFMeshRenderer>(entity).lock();
      if (!mesh_renderer->IsEnabled())
        continue;
      auto mesh = mesh_renderer->mesh.Get<Mesh>();
      auto material = mesh_renderer->btf.Get<CompressedBTF>();
      if (!material || !material->m_bTFBase.m_hasData || !mesh || mesh->UnsafeGetVertices().empty())
        continue;
      auto global_transform = scene->GetDataComponent<GlobalTransform>(entity).value;
      bool need_instance_update = false;
      bool need_material_update = false;

      auto entity_handle = scene->GetEntityHandle(entity);
      auto geometry_handle = mesh->GetHandle();
      auto material_handle = material->GetHandle();
      auto& ray_traced_instance = instance_storage[mesh_renderer->GetHandle().GetValue()];
      auto& ray_traced_geometry = geometry_storage[geometry_handle];
      auto& ray_traced_material = material_storage[material_handle];
      ray_traced_instance.m_removeFlag = false;
      ray_traced_material.m_removeFlag = false;
      ray_traced_geometry.m_removeFlag = false;

      if (ray_traced_instance.m_entityHandle != entity_handle ||
          ray_traced_instance.m_privateComponentHandle != mesh_renderer->GetHandle().GetValue() ||
          ray_traced_instance.m_version != mesh_renderer->GetVersion() ||
          global_transform != ray_traced_instance.m_globalTransform) {
        need_instance_update = true;
      }
      if (ray_traced_geometry.m_handle == 0 || ray_traced_geometry.m_version != mesh->GetVersion()) {
        ray_traced_geometry.m_updateFlag = true;
        need_instance_update = true;
        ray_traced_geometry.m_rendererType = RendererType::Default;
        ray_traced_geometry.m_triangles = &mesh->UnsafeGetTriangles();
        ray_traced_geometry.m_vertices = &mesh->UnsafeGetVertices();
        ray_traced_geometry.m_version = mesh->GetVersion();
        ray_traced_geometry.m_geometryType = PrimitiveType::Triangle;
        ray_traced_geometry.m_handle = geometry_handle;
      }
      if (CheckCompressedBtf(ray_traced_material, material))
        need_instance_update = true;
      if (need_instance_update) {
        ray_traced_instance.m_entityHandle = entity_handle;
        ray_traced_instance.m_privateComponentHandle = mesh_renderer->GetHandle().GetValue();
        ray_traced_instance.m_version = mesh_renderer->GetVersion();
        ray_traced_instance.m_globalTransform = global_transform;
        ray_traced_instance.m_geometryMapKey = geometry_handle;
        ray_traced_instance.m_materialMapKey = material_handle;
      }
      update_shader_binding_table = update_shader_binding_table || need_material_update;
      rebuild_instances = rebuild_instances || need_instance_update;
    }
  }

  for (auto& i : instance_storage)
    if (i.second.m_removeFlag)
      rebuild_instances = true;
}

bool RayTracerLayer::UpdateScene(const std::shared_ptr<Scene>& scene) {
  bool rebuild_acceleration_structure = false;
  bool update_shader_binding_table = false;
  auto& instance_storage = CudaModule::GetRayTracer()->m_instances;
  auto& material_storage = CudaModule::GetRayTracer()->m_materials;
  auto& geometry_storage = CudaModule::GetRayTracer()->m_geometries;
  UpdateMeshesStorage(scene, material_storage, geometry_storage, instance_storage, rebuild_acceleration_structure,
                      update_shader_binding_table);
  auto& env_settings = scene->environment;
  if (const bool use_env_map = env_settings.environment_type == EnvironmentType::EnvironmentalMap;
      environment_properties.m_useEnvironmentalMap != use_env_map) {
    environment_properties.m_useEnvironmentalMap = use_env_map;
    update_shader_binding_table = true;
  }
  if (environmental_map_handle != env_settings.environmental_map.GetAssetHandle()) {
    environmental_map_handle = env_settings.environmental_map.GetAssetHandle();
    if (auto env_map = env_settings.environmental_map.Get<EnvironmentalMap>()) {
      if (const auto reflection_probe = env_map->reflection_probe.Get<ReflectionProbe>()) {
        environmental_map_image = CudaModule::ImportCubemap(reflection_probe->GetCubemap());
        environment_properties.m_environmentalMap = environmental_map_image->m_textureObject;
      }
    } else {
      env_map = Resources::GetResource<EnvironmentalMap>("DEFAULT_ENVIRONMENTAL_MAP");
      const auto reflection_probe = env_map->reflection_probe.Get<ReflectionProbe>();
      environmental_map_image = CudaModule::ImportCubemap(reflection_probe->GetCubemap());
      environment_properties.m_environmentalMap = environmental_map_image->m_textureObject;
    }
    update_shader_binding_table = true;
  }
  if (env_settings.background_color != environment_properties.m_color) {
    environment_properties.m_color = env_settings.background_color;
    update_shader_binding_table = true;
  }
  if (environment_properties.m_skylightIntensity != env_settings.ambient_light_intensity) {
    environment_properties.m_skylightIntensity = env_settings.ambient_light_intensity;
    update_shader_binding_table = true;
  }
  if (environment_properties.m_gamma != env_settings.environment_gamma) {
    environment_properties.m_gamma = env_settings.environment_gamma;
    update_shader_binding_table = true;
  }

  CudaModule::GetRayTracer()->m_sceneModified = false;
  if (rebuild_acceleration_structure && !instance_storage.empty()) {
    CudaModule::GetRayTracer()->BuildIAS();
    return true;
  }
  if (update_shader_binding_table) {
    CudaModule::GetRayTracer()->m_sceneModified = true;
    return true;
  }
  return false;
}
PrivateComponentRegistration<BTFMeshRenderer> btfmr_registry("BTFMeshRenderer");
PrivateComponentRegistration<TriangleIlluminationEstimator> tie_registry("TriangleIlluminationEstimator");
PrivateComponentRegistration<RayTracerCamera> rtc_registry("RayTracerCamera");
PrivateComponentRegistration<BasicPointCloudScanner> bpcs_registry("BasicPointCloudScanner");
AssetRegistration<CompressedBTF> cbtf_registry("CompressedBTF", {".cbtf"});
void RayTracerLayer::OnCreate() {
  CudaModule::Init();

  scene_camera = Serialization::ProduceSerializable<RayTracerCamera>();
  scene_camera->OnCreate();
  Application::RegisterPostAttachSceneFunction([&](const std::shared_ptr<Scene>& scene) {
    ray_tracer_camera_.reset();
  });
}

void RayTracerLayer::PreUpdate() {
  if (const auto editor_layer = Application::GetLayer<EditorLayer>();
      show_scene_window && editor_layer && rendering_enabled) {
    scene_camera->Ready(editor_layer->GetSceneCameraPosition(), editor_layer->GetSceneCameraRotation());
  }
}

void RayTracerLayer::LateUpdate() {
  const auto scene = GetScene();
  bool ray_tracer_updated = UpdateScene(scene);
  if (!CudaModule::GetRayTracer()->m_instances.empty()) {
    if (const auto editor_layer = Application::GetLayer<EditorLayer>();
        show_scene_window && editor_layer && rendering_enabled) {
      scene_camera->rendered_ = CudaModule::GetRayTracer()->RenderToCamera(
          environment_properties, scene_camera->camera_properties_, scene_camera->ray_properties);
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

void RayTracerLayer::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("View")) {
      if (ImGui::BeginMenu("Editor")) {
        if (ImGui::BeginMenu("Scene")) {
          ImGui::Checkbox("Show Scene (RT) Window", &show_scene_window);
          if (show_scene_window) {
            ImGui::Checkbox("Show Scene (RT) Window Info", &show_scene_info);
          }
          ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Camera")) {
          ImGui::Checkbox("Show Camera (RT) Window", &show_camera_window);
          ImGui::EndMenu();
        }
        ImGui::Checkbox("Ray Tracer Settings", &show_ray_tracer_settings_window);
        ImGui::EndMenu();
      }
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }
  if (show_ray_tracer_settings_window) {
    if (ImGui::Begin("Ray Tracer Settings")) {
      ImGui::Checkbox("Mesh Renderer", &render_mesh_renderer);
      ImGui::Checkbox("Strand Renderer", &render_strands_renderer);
      ImGui::Checkbox("Particles", &render_particles);
      ImGui::Checkbox("Skinned Mesh Renderer", &render_skinned_mesh_renderer);
      ImGui::Checkbox("BTF Mesh Renderer", &render_btf_mesh_renderer);

      if (ImGui::TreeNode("Scene Camera Settings")) {
        scene_camera->OnInspect(editor_layer);
        ImGui::TreePop();
      }
      if (ImGui::TreeNodeEx("Environment Properties", ImGuiTreeNodeFlags_DefaultOpen)) {
        environment_properties.OnInspect();
        ImGui::TreePop();
      }
    }
    ImGui::End();
  }
  if (show_camera_window)
    RayCameraWindow();
  if (show_scene_window)
    SceneCameraWindow();
}

void RayTracerLayer::OnDestroy() {
  CudaModule::Terminate();
}

void RayTracerLayer::SceneCameraWindow() {
  const auto editor_layer = Application::GetLayer<EditorLayer>();
  if (!editor_layer)
    return;
  auto scene_camera_rotation = editor_layer->GetSceneCameraRotation();
  auto scene_camera_position = editor_layer->GetSceneCameraPosition();
  const auto scene = GetScene();
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
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
          scene_camera->camera_properties_.OnInspect();
          scene_camera->ray_properties.OnInspect();
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
            scene_camera_position += front * static_cast<float>(Times::DeltaTime()) * editor_layer->velocity;
          }
          if (editor_layer->GetKey(GLFW_KEY_S) == Input::KeyActionType::Hold) {
            scene_camera_position -= front * static_cast<float>(Times::DeltaTime()) * editor_layer->velocity;
          }
          if (editor_layer->GetKey(GLFW_KEY_A) == Input::KeyActionType::Hold) {
            scene_camera_position -= right * static_cast<float>(Times::DeltaTime()) * editor_layer->velocity;
          }
          if (editor_layer->GetKey(GLFW_KEY_D) == Input::KeyActionType::Hold) {
            scene_camera_position += right * static_cast<float>(Times::DeltaTime()) * editor_layer->velocity;
          }
          if (editor_layer->GetKey(GLFW_KEY_LEFT_SHIFT) == Input::KeyActionType::Hold) {
            scene_camera_position.y += editor_layer->velocity * static_cast<float>(Times::DeltaTime());
          }
          if (editor_layer->GetKey(GLFW_KEY_LEFT_CONTROL) == Input::KeyActionType::Hold) {
            scene_camera_position.y -= editor_layer->velocity * static_cast<float>(Times::DeltaTime());
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
          editor_layer->SetCameraPosition(editor_layer->GetSceneCamera(), scene_camera_position);
          editor_layer->SetCameraRotation(editor_layer->GetSceneCamera(), scene_camera_rotation);
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
  const auto editor_layer = Application::GetLayer<EditorLayer>();
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

bool RayTracerLayer::CheckMaterial(RayTracedMaterial& ray_tracer_material,
                                   const std::shared_ptr<Material>& material) const {
  bool changed = false;
  if (ray_tracer_material.m_materialType == MaterialType::Default && material->vertex_color_only) {
    changed = true;
    ray_tracer_material.m_materialType = MaterialType::VertexColor;
  } else if (ray_tracer_material.m_materialType == MaterialType::VertexColor && !material->vertex_color_only) {
    changed = true;
    ray_tracer_material.m_materialType = MaterialType::Default;
  }

  if (changed || ray_tracer_material.m_version != material->GetVersion()) {
    ray_tracer_material.m_handle = material->GetHandle();
    ray_tracer_material.m_version = material->GetVersion();
    ray_tracer_material.m_materialProperties = material->material_properties;

    if (const auto albedo_texture = material->GetAlbedoTexture();
        albedo_texture && albedo_texture->GetVkImage() != VK_NULL_HANDLE) {
      ray_tracer_material.m_albedoTexture = CudaModule::ImportTexture2D(albedo_texture);
    } else {
      ray_tracer_material.m_albedoTexture = nullptr;
    }
    if (const auto normal_texture = material->GetNormalTexture();
        normal_texture && normal_texture->GetVkImage() != VK_NULL_HANDLE) {
      ray_tracer_material.m_normalTexture = CudaModule::ImportTexture2D(normal_texture);
    } else {
      ray_tracer_material.m_normalTexture = nullptr;
    }
    if (const auto roughness_texture = material->GetRoughnessTexture();
        roughness_texture && roughness_texture->GetVkImage() != VK_NULL_HANDLE) {
      ray_tracer_material.m_roughnessTexture = CudaModule::ImportTexture2D(roughness_texture);
    } else {
      ray_tracer_material.m_roughnessTexture = nullptr;
    }
    if (const auto metallic_texture = material->GetMetallicTexture();
        metallic_texture && metallic_texture->GetVkImage() != VK_NULL_HANDLE) {
      ray_tracer_material.m_metallicTexture = CudaModule::ImportTexture2D(metallic_texture);
    } else {
      ray_tracer_material.m_metallicTexture = nullptr;
    }

    changed = true;
  }

  return changed;
}

bool RayTracerLayer::CheckCompressedBtf(RayTracedMaterial& ray_tracer_material,
                                        const std::shared_ptr<CompressedBTF>& compressed_btf) {
  bool changed = false;
  if (ray_tracer_material.m_materialType != MaterialType::CompressedBTF) {
    changed = true;
    ray_tracer_material.m_materialType = MaterialType::CompressedBTF;
  }
  if (ray_tracer_material.m_version != compressed_btf->m_version) {
    changed = true;
    ray_tracer_material.m_version = compressed_btf->m_version;
    ray_tracer_material.m_btfBase = &compressed_btf->m_bTFBase;
  }
  return changed;
}

glm::ivec2 RayTracerLayer::GetSceneCameraResolution() const {
  return scene_camera_resolution_;
}
