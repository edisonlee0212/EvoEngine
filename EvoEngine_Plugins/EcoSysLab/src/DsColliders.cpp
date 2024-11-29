#include "DsColliders.hpp"

#include "Shader.hpp"

using namespace eco_sys_lab_plugin;

void DsBoxCollider::RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Camera>& editor_camera, const glm::vec4& color) {
  const auto mesh_renderer = mesh_renderer_ref.Get<MeshRenderer>();
  if (!mesh_renderer)
    return;
  if (!mesh_renderer->IsEnabled())
    return;
  const auto mesh = mesh_renderer->mesh.Get<Mesh>();
  const auto material = mesh_renderer->material.Get<Material>();
  if (!mesh || !material)
    return;

  const auto& bound = mesh->GetBound();

  const auto scene = GetScene();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(GetOwner());
  glm::vec3 size = bound.Size() * 2.0f;
  if (size.x < 0.001f)
    size.x = 0.001f;
  if (size.z < 0.001f)
    size.z = 0.001f;
  if (size.y < 0.001f)
    size.y = 0.001f;
  GizmoSettings gizmo_settings;
  gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
  gizmo_settings.draw_settings.blending = true;
  gizmo_settings.draw_settings.polygon_mode = VK_POLYGON_MODE_FILL;
  gizmo_settings.draw_settings.line_width = 1.0f;
  gizmo_settings.depth_test = true;
  editor_layer->DrawGizmoMesh(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), editor_camera, color,
                              glm::translate(bound.Center()) * glm::scale(size) * global_transform.value, 1,
                              gizmo_settings);
}

DsBoxCollider::DsBoxCollider() {
  if (!pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/ContactConstraints/BoxCollider.comp");

    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = shader;
    pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    pipeline->Initialize();
  }
}

bool DsBoxCollider::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (editor_layer->DragAndDropButton<MeshRenderer>(mesh_renderer_ref, "Mesh Renderer")) {
    changed = true;
  }

  if (ImGui::DragFloat("Softness", &softness, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }

  static bool display_bound = true;
  ImGui::Checkbox("Display bounds##DsBoxCollider", &display_bound);
  if (display_bound) {
    static auto display_bound_color = glm::vec4(1.0f, 0.0f, 1.0f, 0.2f);
    ImGui::ColorEdit4("Color:##DsBoxCollider", (float*)(void*)&display_bound_color);
    RenderBound(editor_layer, editor_layer->GetSceneCamera(), display_bound_color);
  }

  return changed;
}

void DsBoxCollider::ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
    const DynamicStrands& target_dynamic_strands) {
  const auto mesh_renderer = mesh_renderer_ref.Get<MeshRenderer>();
  if (!mesh_renderer)
    return;
  if (!mesh_renderer->IsEnabled())
    return;
  const auto mesh = mesh_renderer->mesh.Get<Mesh>();
  const auto material = mesh_renderer->material.Get<Material>();
  if (!mesh || !material)
    return;

  const auto& bound = mesh->GetBound();

  const auto scene = GetScene();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(GetOwner());

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  glm::vec3 size = bound.Size();
  if (size.x < 0.001f)
    size.x = 0.001f;
  if (size.z < 0.001f)
    size.z = 0.001f;
  if (size.y < 0.001f)
    size.y = 0.001f;
  PushConstant segment_push_constant;
  segment_push_constant.obb_center = global_transform.GetPosition() + bound.Center();
  segment_push_constant.obb_scale = size * global_transform.GetScale();
  segment_push_constant.obb_rotation = global_transform.GetRotation();
  segment_push_constant.segment_size = target_dynamic_strands.segments.size();
  segment_push_constant.softness = softness;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    pipeline->Bind(vk_command_buffer);
    pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(segment_push_constant.segment_size, task_work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}


void DsBoxCollider::OnDestroy() {
  mesh_renderer_ref.Clear();
}

void DsBoxCollider::Serialize(YAML::Emitter& out) const {
  mesh_renderer_ref.Save("mesh_renderer_ref", out);
}

void DsBoxCollider::Deserialize(const YAML::Node& in) {
  mesh_renderer_ref.Load("mesh_renderer_ref", in, GetScene());
}

void DsBoxCollider::Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) {
  mesh_renderer_ref.Relink(map, scene);
}


