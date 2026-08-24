#include "RayTracerLayer.hpp"
#include "Application.hpp"
#include "BasicPointCloudScanner.hpp"
#include "CudaSerializationAdapters.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalLightingResolver.hpp"
#include "EnvironmentalMap.hpp"
#include "GlobalReflectionProbe.hpp"
#include "InspectorRegistry.hpp"
#include "MeshRenderer.hpp"
#include "OptiXRayTracer.hpp"
#include "Particles.hpp"
#include "RayTracerCamera.hpp"
#include "Resources.hpp"
#include "Serialization.hpp"
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
      auto& ray_traced_instance = instance_storage[strands_renderer_renderer->GetHandle().GetValue()];
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
      auto& ray_traced_instance = instance_storage[mesh_renderer->GetHandle().GetValue()];
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
      auto& ray_traced_instance = instance_storage[geometry_handle];
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
      auto& ray_traced_instance = instance_storage[geometry_handle];
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
    auto reflection_probe = scene->GetGlobalReflectionProbeFallback(false);
    if (!reflection_probe || !reflection_probe->IsRuntimeReady()) {
      env_map = Resources::GetInstance().GetDefaultEnvironmentalMap();
      reflection_probe = Resources::GetInstance().GetDefaultGlobalReflectionProbe();
    }
    if (reflection_probe && !reflection_probe->IsRuntimeReady()) {
      reflection_probe.reset();
    }
    const Handle requested_reflection_probe = reflection_probe ? reflection_probe->GetHandle() : Handle{};
    const uint64_t requested_payload_hash = reflection_probe ? reflection_probe->GetPayloadHash() : 0u;
    if (environmental_map_handle != requested_environmental_map ||
        global_reflection_probe_handle != requested_reflection_probe ||
        global_reflection_probe_payload_hash != requested_payload_hash) {
      environmental_map_handle = requested_environmental_map;
      global_reflection_probe_handle = requested_reflection_probe;
      global_reflection_probe_payload_hash = requested_payload_hash;
      environment_properties.environmental_map = 0;
      environmental_map_image.reset();
      update_shader_binding_table = true;
    }
    if (use_env_map && !environmental_map_image) {
      const auto imported = reflection_probe ? CudaModule::ImportCubemap(reflection_probe->GetCubemap()) : nullptr;
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
    const float ambient_light_intensity = std::max(lighting.diffuse_fallback_intensity, 0.0f);
    if (environment_properties.ambient_light_intensity != ambient_light_intensity) {
      environment_properties.ambient_light_intensity = ambient_light_intensity;
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
  application.RegisterPrivateComponent<RayTracerCamera>("RayTracerCamera");
  application.RegisterPrivateComponent<BasicPointCloudScanner>("BasicPointCloudScanner");
  Serialization::RegisterSerializationHandler<TriangleIlluminationEstimator>(SerializeTriangleIlluminationEstimator,
                                                                             DeserializeTriangleIlluminationEstimator,
                                                                             {}, "TriangleIlluminationEstimator");
  Serialization::RegisterSerializationHandler<RayTracerCamera>(SerializeRayTracerCamera, DeserializeRayTracerCamera, {},
                                                               "RayTracerCamera");
  Serialization::RegisterSerializationHandler<BasicPointCloudScanner>(
      SerializeBasicPointCloudScanner, DeserializeBasicPointCloudScanner, {}, "BasicPointCloudScanner");
  InspectorRegistry::GetInstance().RegisterInspector<TriangleIlluminationEstimator>(
      [](InspectorContext& context, TriangleIlluminationEstimator& estimator) {
        return estimator.DrawGui(context.editor_layer);
      },
      {}, "TriangleIlluminationEstimator");
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

void RayTracerLayer::OnCreate() {
  CudaModule::Init();

  scene_camera = Serialization::ProduceSerializable<RayTracerCamera>();
  scene_camera->OnCreate();
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

void RayTracerLayer::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
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
  if (show_scene_window) {
    ImGui::Checkbox("Scene (RT) Window Info", &show_scene_info);
  }
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

  if (show_camera_window)
    RayCameraWindow();
  if (show_scene_window)
    SceneCameraWindow();
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
