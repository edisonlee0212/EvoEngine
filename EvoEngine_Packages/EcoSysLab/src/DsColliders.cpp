#include "DsColliders.hpp"

#include "EcoSysLabSerializationAdapters.hpp"
#include "Shader.hpp"

using namespace eco_sys_lab_package;

DsBoxCollider::DsBoxCollider() {
  if (!segment_position_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Colliders/SegmentBoxGround.slang");

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
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Colliders/LeafBox.slang");

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
                           "Shaders/Compute/DynamicStrands/Constraints/Velocity/Colliders/SegmentBox.slang");

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
                           "Shaders/Compute/DynamicStrands/Constraints/Velocity/Colliders/LeafBox.slang");

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

void eco_sys_lab_package::SerializeDsBoxCollider(YAML::Emitter& out, const DsBoxCollider& target) {
  out << YAML::Key << "bound_color" << YAML::Value << target.bound_color;
  out << YAML::Key << "scale" << YAML::Value << target.scale;
  out << YAML::Key << "softness" << YAML::Value << target.softness;
}

void eco_sys_lab_package::DeserializeDsBoxCollider(const YAML::Node& in, DsBoxCollider& target) {
  if (in["bound_color"])
    target.bound_color = in["bound_color"].as<glm::vec4>();
  if (in["scale"])
    target.scale = in["scale"].as<glm::vec3>();
  if (in["softness"])
    target.softness = in["softness"].as<float>();
}

DsCylinderCollider::DsCylinderCollider() {
  if (!segment_position_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Colliders/SegmentCylinder.slang");

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
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Colliders/LeafCylinder.slang");

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

void eco_sys_lab_package::SerializeDsCylinderCollider(YAML::Emitter& out, const DsCylinderCollider& target) {
  out << YAML::Key << "bound_color" << YAML::Value << target.bound_color;
  out << YAML::Key << "radius" << YAML::Value << target.radius;
  out << YAML::Key << "height" << YAML::Value << target.height;
  out << YAML::Key << "softness" << YAML::Value << target.softness;
}

void eco_sys_lab_package::DeserializeDsCylinderCollider(const YAML::Node& in, DsCylinderCollider& target) {
  if (in["bound_color"])
    target.bound_color = in["bound_color"].as<glm::vec4>();
  if (in["radius"])
    target.radius = in["radius"].as<float>();
  if (in["height"])
    target.height = in["height"].as<float>();
  if (in["softness"])
    target.softness = in["softness"].as<float>();
}

DsSphereCollider::DsSphereCollider() {
  if (!segment_position_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Colliders/SegmentSphere.slang");

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
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Colliders/LeafSphere.slang");

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

void eco_sys_lab_package::SerializeDsSphereCollider(YAML::Emitter& out, const DsSphereCollider& target) {
  out << YAML::Key << "bound_color" << YAML::Value << target.bound_color;
  out << YAML::Key << "softness" << YAML::Value << target.softness;
  out << YAML::Key << "radius" << YAML::Value << target.radius;
}

void eco_sys_lab_package::DeserializeDsSphereCollider(const YAML::Node& in, DsSphereCollider& target) {
  if (in["bound_color"])
    target.bound_color = in["bound_color"].as<glm::vec4>();
  if (in["radius"])
    target.radius = in["radius"].as<float>();
  if (in["softness"])
    target.softness = in["softness"].as<float>();
}
