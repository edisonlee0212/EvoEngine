#include "DsColliders.hpp"

#include "Shader.hpp"

using namespace eco_sys_lab_plugin;

void DsBoxCollider::RenderBound(const std::shared_ptr<EditorLayer>& editor_layer,
                                const std::shared_ptr<Camera>& editor_camera, const glm::vec4& color) {
  const auto scene = GetScene();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(GetOwner());
  glm::vec3 size = scale * global_transform.GetScale() * 2.0f;
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
  editor_layer->DrawGizmoMesh(Resources::GetInstance().GetPrimitives().cube, editor_camera, color,
                              glm::translate(global_transform.GetPosition()) *
                                  glm::mat4_cast(global_transform.GetRotation()) * glm::scale(size),
                              1, gizmo_settings);
}

DsBoxCollider::DsBoxCollider() {
  if (!segment_position_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Colliders/SegmentBoxGround.comp");

    segment_position_pipeline = std::make_shared<ComputePipeline>();
    segment_position_pipeline->compute_shader = shader;
    segment_position_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = segment_position_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentPositionPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_position_pipeline->Initialize();
  }
  if (!leaf_position_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Colliders/LeafBox.comp");

    leaf_position_pipeline = std::make_shared<ComputePipeline>();
    leaf_position_pipeline->compute_shader = shader;
    leaf_position_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = leaf_position_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(LeafPositionPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    leaf_position_pipeline->Initialize();
  }

  if (!segment_velocity_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Velocity/Colliders/SegmentBox.comp");

    segment_velocity_pipeline = std::make_shared<ComputePipeline>();
    segment_velocity_pipeline->compute_shader = shader;
    segment_velocity_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = segment_velocity_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentVelocityPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_velocity_pipeline->Initialize();
  }
  if (!leaf_velocity_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Velocity/Colliders/LeafBox.comp");

    leaf_velocity_pipeline = std::make_shared<ComputePipeline>();
    leaf_velocity_pipeline->compute_shader = shader;
    leaf_velocity_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = leaf_velocity_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(LeafVelocityPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    leaf_velocity_pipeline->Initialize();
  }
}

bool DsBoxCollider::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  static PrivateComponentRef mesh_renderer_ref;
  if (editor_layer->DragAndDropButton<MeshRenderer>(mesh_renderer_ref, "Apply bound from Mesh Renderer")) {
    if (const auto mesh_renderer = mesh_renderer_ref.Get<MeshRenderer>()) {
      if (const auto mesh = mesh_renderer->mesh.Get<Mesh>()) {
        const auto& bound = mesh->GetBound();
        scale = bound.Size();
      }
    }
    mesh_renderer_ref.Clear();
  }

  if (ImGui::DragFloat3("Scale", &scale.x, 0.01f, 0.0f, 10.0f)) {
    changed = true;
  }

  if (ImGui::DragFloat("Softness", &softness, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Friction", &friction, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Rotational friction", &rotational_friction, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Velocity friction", &velocity_friction, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Angular velocity friction", &angular_velocity_friction, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }
  ImGui::ColorEdit4("Bound Color:##DsBoxCollider", (float*)(void*)&bound_color);
  static bool display_bound = true;
  ImGui::Checkbox("Display bounds##DsBoxCollider", &display_bound);
  if (display_bound) {
    RenderBound(editor_layer, editor_layer->GetSceneCamera(), bound_color);
  }
  return changed;
}

void DsBoxCollider::ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                              const DynamicStrands& target_dynamic_strands) {
  const auto scene = GetScene();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(GetOwner());

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  glm::vec3 size = scale * global_transform.GetScale();
  if (size.x < 0.001f)
    size.x = 0.001f;
  if (size.z < 0.001f)
    size.z = 0.001f;
  if (size.y < 0.001f)
    size.y = 0.001f;
  SegmentPositionPushConstant segment_push_constant;
  segment_push_constant.obb_center = global_transform.GetPosition();
  segment_push_constant.obb_scale = size;
  segment_push_constant.obb_rotation = global_transform.GetRotation();
  segment_push_constant.segment_size = target_dynamic_strands.segments.size();
  segment_push_constant.softness = softness;
  segment_push_constant.friction = friction;
  segment_push_constant.rotational_friction = rotational_friction;
  LeafPositionPushConstant leaf_push_constant;
  leaf_push_constant.obb_center = global_transform.GetPosition();
  leaf_push_constant.obb_scale = size;
  leaf_push_constant.obb_rotation = global_transform.GetRotation();
  leaf_push_constant.leaf_size = target_dynamic_strands.foliage.size();
  leaf_push_constant.softness = softness;
  leaf_push_constant.friction = friction;
  leaf_push_constant.rotational_friction = rotational_friction;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    segment_position_pipeline->Bind(vk_command_buffer);
    segment_position_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    segment_position_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);
    segment_position_pipeline->Dispatch(
        vk_command_buffer, Platform::DivUp(segment_push_constant.segment_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    leaf_position_pipeline->Bind(vk_command_buffer);
    leaf_position_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    leaf_position_pipeline->PushConstant(vk_command_buffer, 0, leaf_push_constant);
    leaf_position_pipeline->Dispatch(vk_command_buffer,
                                     Platform::DivUp(leaf_push_constant.leaf_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void DsBoxCollider::ProjectVelocityConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                              const DynamicStrands& target_dynamic_strands) {
  const auto scene = GetScene();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(GetOwner());

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  glm::vec3 size = scale * global_transform.GetScale();
  if (size.x < 0.001f)
    size.x = 0.001f;
  if (size.z < 0.001f)
    size.z = 0.001f;
  if (size.y < 0.001f)
    size.y = 0.001f;
  SegmentVelocityPushConstant segment_push_constant;
  segment_push_constant.obb_center = global_transform.GetPosition();
  segment_push_constant.obb_scale = size;
  segment_push_constant.obb_rotation = global_transform.GetRotation();
  segment_push_constant.segment_size = target_dynamic_strands.segments.size();
  segment_push_constant.velocity_friction = velocity_friction;
  segment_push_constant.angular_velocity_friction = angular_velocity_friction;
  LeafVelocityPushConstant leaf_push_constant;
  leaf_push_constant.obb_center = global_transform.GetPosition();
  leaf_push_constant.obb_scale = size;
  leaf_push_constant.obb_rotation = global_transform.GetRotation();
  leaf_push_constant.leaf_size = target_dynamic_strands.foliage.size();
  leaf_push_constant.velocity_friction = velocity_friction;
  leaf_push_constant.angular_velocity_friction = angular_velocity_friction;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    segment_velocity_pipeline->Bind(vk_command_buffer);
    segment_velocity_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    segment_velocity_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);
    segment_velocity_pipeline->Dispatch(
        vk_command_buffer, Platform::DivUp(segment_push_constant.segment_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    leaf_velocity_pipeline->Bind(vk_command_buffer);
    leaf_velocity_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    leaf_velocity_pipeline->PushConstant(vk_command_buffer, 0, leaf_push_constant);
    leaf_velocity_pipeline->Dispatch(vk_command_buffer,
                                     Platform::DivUp(leaf_push_constant.leaf_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void DsBoxCollider::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "bound_color" << YAML::Value << bound_color;
  out << YAML::Key << "scale" << YAML::Value << scale;
  out << YAML::Key << "softness" << YAML::Value << softness;
}

void DsBoxCollider::Deserialize(const YAML::Node& in) {
  if (in["bound_color"])
    bound_color = in["bound_color"].as<glm::vec4>();
  if (in["scale"])
    scale = in["scale"].as<glm::vec3>();
  if (in["softness"])
    softness = in["softness"].as<float>();
}

void DsCylinderCollider::RenderBound(const std::shared_ptr<EditorLayer>& editor_layer,
                                     const std::shared_ptr<Camera>& editor_camera, const glm::vec4& color) {
  const auto scene = GetScene();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(GetOwner());
  const auto scale = global_transform.GetScale();
  auto size = glm::vec2(radius * glm::max(scale.x, scale.z), height * scale.y) * 2.f;
  if (size.x < 0.001f)
    size.x = 0.001f;
  if (size.y < 0.001f)
    size.y = 0.001f;
  GizmoSettings gizmo_settings;
  gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
  gizmo_settings.draw_settings.blending = true;
  gizmo_settings.draw_settings.polygon_mode = VK_POLYGON_MODE_FILL;
  gizmo_settings.draw_settings.line_width = 1.0f;
  gizmo_settings.depth_test = true;
  editor_layer->DrawGizmoMesh(Resources::GetInstance().GetPrimitives().cylinder, editor_camera, color,
                              glm::translate(global_transform.GetPosition()) *
                                  glm::mat4_cast(global_transform.GetRotation()) *
                                  glm::scale(glm::vec3(size.x, size.y, size.x)),
                              1, gizmo_settings);
}

DsCylinderCollider::DsCylinderCollider() {
  if (!segment_position_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Colliders/SegmentCylinder.comp");

    segment_position_pipeline = std::make_shared<ComputePipeline>();
    segment_position_pipeline->compute_shader = shader;
    segment_position_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = segment_position_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_position_pipeline->Initialize();
  }

  if (!leaf_position_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Colliders/LeafCylinder.comp");

    leaf_position_pipeline = std::make_shared<ComputePipeline>();
    leaf_position_pipeline->compute_shader = shader;
    leaf_position_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = leaf_position_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(LeafPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    leaf_position_pipeline->Initialize();
  }
}

bool DsCylinderCollider::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (ImGui::DragFloat("Radius", &radius, 0.01f, 0.0f, 10.0f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Height", &height, 0.01f, 0.0f, 10.0f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Softness", &softness, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }
  ImGui::ColorEdit4("Bound Color:##DsBoxCollider", (float*)(void*)&bound_color);
  static bool display_bound = true;
  ImGui::Checkbox("Display bounds##DsBoxCollider", &display_bound);
  if (display_bound) {
    RenderBound(editor_layer, editor_layer->GetSceneCamera(), bound_color);
  }
  return changed;
}

void DsCylinderCollider::ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                                   const DynamicStrands& target_dynamic_strands) {
  const auto scene = GetScene();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(GetOwner());

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  const glm::vec3 scale = global_transform.GetScale();
  auto size = glm::vec2(radius * glm::max(scale.x, scale.z), height * scale.y);
  if (size.x < 0.001f)
    size.x = 0.001f;
  if (size.y < 0.001f)
    size.y = 0.001f;
  SegmentPushConstant segment_push_constant;
  segment_push_constant.obb_center = global_transform.GetPosition();
  segment_push_constant.radius = size.x;
  segment_push_constant.height = size.y;
  segment_push_constant.obb_rotation = global_transform.GetRotation();
  segment_push_constant.segment_size = target_dynamic_strands.segments.size();
  segment_push_constant.softness = softness;

  LeafPushConstant leaf_push_constant;
  leaf_push_constant.obb_center = global_transform.GetPosition();
  leaf_push_constant.radius = size.x;
  leaf_push_constant.height = size.y;
  leaf_push_constant.obb_rotation = global_transform.GetRotation();
  leaf_push_constant.leaf_size = target_dynamic_strands.foliage.size();
  leaf_push_constant.softness = softness;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    segment_position_pipeline->Bind(vk_command_buffer);
    segment_position_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    segment_position_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);
    segment_position_pipeline->Dispatch(
        vk_command_buffer, Platform::DivUp(segment_push_constant.segment_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    leaf_position_pipeline->Bind(vk_command_buffer);
    leaf_position_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    leaf_position_pipeline->PushConstant(vk_command_buffer, 0, leaf_push_constant);
    leaf_position_pipeline->Dispatch(vk_command_buffer,
                                     Platform::DivUp(leaf_push_constant.leaf_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void DsCylinderCollider::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "bound_color" << YAML::Value << bound_color;
  out << YAML::Key << "radius" << YAML::Value << radius;
  out << YAML::Key << "height" << YAML::Value << height;
  out << YAML::Key << "softness" << YAML::Value << softness;
}

void DsCylinderCollider::Deserialize(const YAML::Node& in) {
  if (in["bound_color"])
    bound_color = in["bound_color"].as<glm::vec4>();
  if (in["radius"])
    radius = in["radius"].as<float>();
  if (in["height"])
    height = in["height"].as<float>();
  if (in["softness"])
    softness = in["softness"].as<float>();
}

void DsSphereCollider::RenderBound(const std::shared_ptr<EditorLayer>& editor_layer,
                                   const std::shared_ptr<Camera>& editor_camera, const glm::vec4& color) {
  const auto scene = GetScene();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(GetOwner());
  const auto scale = global_transform.GetScale();
  GizmoSettings gizmo_settings;
  gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
  gizmo_settings.draw_settings.blending = true;
  gizmo_settings.draw_settings.polygon_mode = VK_POLYGON_MODE_FILL;
  gizmo_settings.draw_settings.line_width = 1.0f;
  gizmo_settings.depth_test = true;
  editor_layer->DrawGizmoMesh(
      Resources::GetInstance().GetPrimitives().sphere, editor_camera, color,
      glm::translate(global_transform.GetPosition()) *
          glm::scale(glm::vec3(glm::max(0.001f, radius * 2.f)) * glm::max(glm::max(scale.x, scale.y), scale.z)),
      1, gizmo_settings);
}

DsSphereCollider::DsSphereCollider() {
  if (!segment_position_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Colliders/SegmentSphere.comp");

    segment_position_pipeline = std::make_shared<ComputePipeline>();
    segment_position_pipeline->compute_shader = shader;
    segment_position_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = segment_position_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_position_pipeline->Initialize();
  }

  if (!leaf_position_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Colliders/LeafSphere.comp");

    leaf_position_pipeline = std::make_shared<ComputePipeline>();
    leaf_position_pipeline->compute_shader = shader;
    leaf_position_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = leaf_position_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(LeafPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    leaf_position_pipeline->Initialize();
  }
}

bool DsSphereCollider::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat("Radius", &radius, 0.01f, 0.0f, 10.0f)) {
    changed = true;
  }

  if (ImGui::DragFloat("Softness", &softness, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }

  ImGui::ColorEdit4("Bound Color:##DsBoxCollider", (float*)(void*)&bound_color);
  static bool display_bound = true;
  ImGui::Checkbox("Display bounds##DsBoxCollider", &display_bound);
  if (display_bound) {
    RenderBound(editor_layer, editor_layer->GetSceneCamera(), bound_color);
  }

  return changed;
}

void DsSphereCollider::ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                                 const DynamicStrands& target_dynamic_strands) {
  const auto scene = GetScene();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(GetOwner());

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  const glm::vec3 scale = global_transform.GetScale();
  float size = radius * glm::max(glm::max(scale.x, scale.y), scale.z);
  if (size < 0.001f)
    size = 0.001f;

  SegmentPushConstant segment_push_constant;
  segment_push_constant.obb_center = global_transform.GetPosition();
  segment_push_constant.radius = size;
  segment_push_constant.segment_size = target_dynamic_strands.segments.size();
  segment_push_constant.softness = softness;

  LeafPushConstant leaf_push_constant;
  leaf_push_constant.obb_center = global_transform.GetPosition();
  leaf_push_constant.radius = size;
  leaf_push_constant.leaf_size = target_dynamic_strands.foliage.size();
  leaf_push_constant.softness = softness;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    segment_position_pipeline->Bind(vk_command_buffer);
    segment_position_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    segment_position_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);
    segment_position_pipeline->Dispatch(
        vk_command_buffer, Platform::DivUp(segment_push_constant.segment_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    leaf_position_pipeline->Bind(vk_command_buffer);
    leaf_position_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    leaf_position_pipeline->PushConstant(vk_command_buffer, 0, leaf_push_constant);
    leaf_position_pipeline->Dispatch(vk_command_buffer,
                                     Platform::DivUp(leaf_push_constant.leaf_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void DsSphereCollider::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "bound_color" << YAML::Value << bound_color;
  out << YAML::Key << "softness" << YAML::Value << softness;
  out << YAML::Key << "radius" << YAML::Value << radius;
}

void DsSphereCollider::Deserialize(const YAML::Node& in) {
  if (in["bound_color"])
    bound_color = in["bound_color"].as<glm::vec4>();
  if (in["radius"])
    radius = in["radius"].as<float>();
  if (in["softness"])
    softness = in["softness"].as<float>();
}