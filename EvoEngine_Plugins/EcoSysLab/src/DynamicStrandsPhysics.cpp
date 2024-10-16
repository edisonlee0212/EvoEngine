#include "DynamicStrandsPhysics.hpp"
#include "Eigen"
#include "Shader.hpp"
using namespace eco_sys_lab_plugin;

void DynamicStrands::Physics(const PhysicsParameters& physics_parameters,
                             const std::vector<std::shared_ptr<IDynamicStrandsOperator>>& target_operators,
                             const std::shared_ptr<DynamicStrandsPreStep>& target_pre_step,
                             const std::vector<std::shared_ptr<IDynamicStrandsConstraint>>& target_constraints,
                             const std::shared_ptr<DynamicStrandsPostStep>& target_post_step) {
  for (const auto& op : target_operators) {
    op->Execute(physics_parameters, *this);
  }
  if (target_pre_step)
    target_pre_step->Execute(physics_parameters, *this);
  for (const auto& c : target_constraints) {
    c->Project(physics_parameters, *this);
  }
  if (target_post_step)
    target_post_step->Execute(physics_parameters, *this);
}

DynamicStrandsPreStep::DynamicStrandsPreStep() {
  if (!pre_step_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/PreStep.comp");

    pre_step_pipeline = std::make_shared<ComputePipeline>();
    pre_step_pipeline->compute_shader = shader;
    pre_step_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = pre_step_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PreStepPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    pre_step_pipeline->Initialize();
  }
}

void DynamicStrandsPreStep::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                                    DynamicStrands& target_dynamic_strands) {
  PreStepPushConstant push_constant;
  push_constant.segment_size = target_dynamic_strands.strand_segments.size();
  push_constant.time_step = physics_parameters.time_step;
  push_constant.inv_time_step = 1.f / push_constant.time_step;
  if (physics_parameters.gpu) {
    const auto current_frame_index = Platform::GetCurrentFrameIndex();
    const uint32_t task_work_group_invocations =
        Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      pre_step_pipeline->Bind(vk_command_buffer);
      pre_step_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

      pre_step_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.segment_size, task_work_group_invocations), 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    });
  } else {
    const auto update_inertia_w = [](DynamicStrands::GpuStrandSegment& segment) {
      // Update w
      glm::mat3 rot = glm::mat3_cast(segment.q);
      glm::mat3 inertia_tensor_diag = glm::mat3(segment.inertia_tensor.x, 0.0, 0.0, 0.0, segment.inertia_tensor.y, 0.0,
                                                0.0, 0.0, segment.inertia_tensor.z);
      glm::mat3 inertia_w = rot * inertia_tensor_diag * transpose(rot);

      glm::mat3 inverse_inertia_tensor_diag =
          glm::mat3(segment.inv_inertia_tensor.x, 0.0, 0.0, 0.0, segment.inv_inertia_tensor.y, 0.0, 0.0, 0.0,
                    segment.inv_inertia_tensor.z);
      glm::mat3 inverse_inertia_w = rot * inverse_inertia_tensor_diag * transpose(rot);

      segment.inertia_w =
          glm::mat4(inertia_w[0][0], inertia_w[0][1], inertia_w[0][2], 0.0, inertia_w[1][0], inertia_w[1][1],
                    inertia_w[1][2], 0.0, inertia_w[2][0], inertia_w[2][1], inertia_w[2][2], 0.0, 0.0, 0.0, 0.0, 0.0);
      segment.inv_inertia_w =
          glm::mat4(inverse_inertia_w[0][0], inverse_inertia_w[0][1], inverse_inertia_w[0][2], 0.0,
                    inverse_inertia_w[1][0], inverse_inertia_w[1][1], inverse_inertia_w[1][2], 0.0,
                    inverse_inertia_w[2][0], inverse_inertia_w[2][1], inverse_inertia_w[2][2], 0.0, 0.0, 0.0, 0.0, 0.0);
    };

    Jobs::RunParallelFor(target_dynamic_strands.strand_segments.size(), [&](const size_t segment_index) {
      auto& segment = target_dynamic_strands.strand_segments[segment_index];
      update_inertia_w(segment);

      // Calculate velocity and apply acceleration.
      glm::vec3 velocity = glm::vec3(0, 0, 0);
      if (segment.inv_mass != 0.0) {
        velocity = push_constant.inv_time_step * (1.5f * segment.x - 2.0f * segment.old_x + 0.5f * segment.last_x);
        velocity += segment.acceleration * push_constant.time_step;
      }
      // Calculate angular velocity and apply torque.
      glm::vec3 angular_velocity = glm::vec3(0, 0, 0);
      if (segment.inv_mass != 0.0) {
        glm::quat rot = segment.q * glm::conjugate(segment.old_q);
        angular_velocity = glm::vec3(rot.x, rot.y, rot.z) * 2.0f * push_constant.inv_time_step;
        angular_velocity += push_constant.time_step * glm::mat3(segment.inv_inertia_w) *
                            (segment.torque - cross(angular_velocity, glm::mat3(segment.inertia_w) * angular_velocity));
      }

      // Shift position values
      segment.last_x = segment.old_x;
      segment.old_x = segment.x;
      // Apply velocity
      segment.x += push_constant.time_step * velocity;

      // Shift rotation values
      segment.last_q = segment.old_q;
      segment.old_q = segment.q;
      // Apply angular velocity
      glm::quat angular_velocity_q = glm::quat(0.0f, angular_velocity.x, angular_velocity.y, angular_velocity.z);

      segment.q += 0.5f * push_constant.time_step * (angular_velocity_q * segment.q);
      segment.q = normalize(segment.q);

      // Update w
      update_inertia_w(segment);
    });
  }
}

DynamicStrandsPostStep::DynamicStrandsPostStep() {
  if (!post_step_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/PostStep.comp");

    post_step_pipeline = std::make_shared<ComputePipeline>();
    post_step_pipeline->compute_shader = shader;
    post_step_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = post_step_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PostStepPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    post_step_pipeline->Initialize();
  }
}

void DynamicStrandsPostStep::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                                     DynamicStrands& target_dynamic_strands) {
  if (physics_parameters.gpu) {
    const auto current_frame_index = Platform::GetCurrentFrameIndex();

    PostStepPushConstant push_constant;
    push_constant.segment_size = target_dynamic_strands.strand_segments.size();
    const uint32_t task_work_group_invocations =
        Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      post_step_pipeline->Bind(vk_command_buffer);
      post_step_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

      post_step_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.segment_size, task_work_group_invocations), 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    });
  } else {
  }
}

DsPositionUpdate::DsPositionUpdate() {
  if (!layout) {
    layout = std::make_shared<DescriptorSetLayout>();
    layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
  }
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();

  commands_buffer.resize(max_frame_in_flight);

  for (int frame_index = 0; frame_index < max_frame_in_flight; frame_index++) {
    commands_buffer[frame_index] = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  }

  if (!external_force_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/PositionUpdate.comp");

    external_force_pipeline = std::make_shared<ComputePipeline>();
    external_force_pipeline->compute_shader = shader;

    external_force_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    external_force_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& push_constant_range = external_force_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PositionUpdatePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    external_force_pipeline->Initialize();
  }

  commands_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : commands_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(layout);
  }
}

void DsPositionUpdate::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                               DynamicStrands& target_dynamic_strands) {
  if (commands.empty())
    return;
  if (physics_parameters.gpu) {
    const auto current_frame_index = Platform::GetCurrentFrameIndex();
    commands_buffer[current_frame_index]->UploadVector(commands);
    commands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(0,
                                                                                 commands_buffer[current_frame_index]);

    PositionUpdatePushConstant push_constant;
    push_constant.commands_size = commands.size();
    const uint32_t task_work_group_invocations =
        Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      external_force_pipeline->Bind(vk_command_buffer);
      external_force_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      external_force_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                 commands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

      external_force_pipeline->PushConstant(vk_command_buffer, 0, push_constant);

      vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.commands_size, task_work_group_invocations), 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    });
  } else {
    Jobs::RunParallelFor(commands.size(), [&](const size_t command_index) {
      const auto& position_update = commands[command_index];
      const uint32_t segment_index = position_update.strand_segment_index;
      auto& segment = target_dynamic_strands.strand_segments[segment_index];
      const glm::vec3 new_position = position_update.new_position;
      const glm::vec3 old_position = segment.x;
      const glm::vec3 delta_position = new_position - old_position;

      segment.x = new_position;
      segment.last_x = segment.last_x + delta_position;
    });
  }
}

DsExternalForce::DsExternalForce() {
  if (!layout) {
    layout = std::make_shared<DescriptorSetLayout>();
    layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);

    layout->Initialize();
  }

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();

  commands_buffer.resize(max_frame_in_flight);

  for (int frame_index = 0; frame_index < max_frame_in_flight; frame_index++) {
    commands_buffer[frame_index] = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  }

  if (!external_force_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/ExternalForce.comp");
    external_force_pipeline = std::make_shared<ComputePipeline>();
    external_force_pipeline->compute_shader = shader;

    external_force_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    external_force_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& push_constant_range = external_force_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(ExternalForcePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    external_force_pipeline->Initialize();
  }
  commands_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : commands_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(layout);
  }
}

void DsExternalForce::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                              DynamicStrands& target_dynamic_strands) {
  if (commands.empty())
    return;
  if (physics_parameters.gpu) {
    const auto current_frame_index = Platform::GetCurrentFrameIndex();
    commands_buffer[current_frame_index]->UploadVector(commands);

    commands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(0,
                                                                                 commands_buffer[current_frame_index]);

    ExternalForcePushConstant push_constant;
    push_constant.commands_size = commands.size();
    const uint32_t task_work_group_invocations =
        Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      external_force_pipeline->Bind(vk_command_buffer);
      external_force_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      external_force_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                 commands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

      external_force_pipeline->PushConstant(vk_command_buffer, 0, push_constant);

      vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.commands_size, task_work_group_invocations), 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    });
  } else {
  }
}

DsStiffRod::DsStiffRod() {
  if (!layout) {
    layout = std::make_shared<DescriptorSetLayout>();
    layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);

    layout->Initialize();
  }

  VkBufferCreateInfo uniform_buffer_create_info{};
  uniform_buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  uniform_buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
  uniform_buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  uniform_buffer_create_info.size = 1;
  VkBufferCreateInfo storage_buffer_create_info{};
  storage_buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  storage_buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  storage_buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  storage_buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();

  rod_properties_buffer.resize(max_frame_in_flight);

  for (int frame_index = 0; frame_index < max_frame_in_flight; frame_index++) {
    rod_properties_buffer[frame_index] =
        std::make_shared<Buffer>(uniform_buffer_create_info, buffer_vma_allocation_create_info);
  }

  per_strand_data_list_buffer = std::make_shared<Buffer>(storage_buffer_create_info, buffer_vma_allocation_create_info);
  rod_constraints_buffer = std::make_shared<Buffer>(storage_buffer_create_info, buffer_vma_allocation_create_info);

  if (!init_constraint_pipeline) {
    static std::shared_ptr<Shader> init_constraint_shader{};
    init_constraint_shader = std::make_shared<Shader>();
    init_constraint_shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                                std::filesystem::path("./EcoSysLabResources") /
                                    "Shaders/Compute/DynamicStrands/Constraints/StiffRodInitConstraints.comp");

    init_constraint_pipeline = std::make_shared<ComputePipeline>();
    init_constraint_pipeline->compute_shader = init_constraint_shader;

    init_constraint_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    init_constraint_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& init_push_constant_range = init_constraint_pipeline->push_constant_ranges.emplace_back();
    init_push_constant_range.size = sizeof(StiffRodInitConstraintConstant);
    init_push_constant_range.offset = 0;
    init_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    init_constraint_pipeline->Initialize();

    static std::shared_ptr<Shader> update_constraint_shader{};
    update_constraint_shader = std::make_shared<Shader>();
    update_constraint_shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                                  std::filesystem::path("./EcoSysLabResources") /
                                      "Shaders/Compute/DynamicStrands/Constraints/StiffRodUpdateConstraints.comp");

    update_constraint_pipeline = std::make_shared<ComputePipeline>();
    update_constraint_pipeline->compute_shader = update_constraint_shader;

    update_constraint_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    update_constraint_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& update_push_constant_range = update_constraint_pipeline->push_constant_ranges.emplace_back();
    update_push_constant_range.size = sizeof(StiffRodUpdateConstraintConstant);
    update_push_constant_range.offset = 0;
    update_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    update_constraint_pipeline->Initialize();

    static std::shared_ptr<Shader> project_constraint_shader{};
    project_constraint_shader = std::make_shared<Shader>();
    project_constraint_shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                                   std::filesystem::path("./EcoSysLabResources") /
                                       "Shaders/Compute/DynamicStrands/Constraints/StiffRodProjectConstraints.comp");

    project_constraint_pipeline = std::make_shared<ComputePipeline>();
    project_constraint_pipeline->compute_shader = project_constraint_shader;

    project_constraint_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    project_constraint_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& project_push_constant_range = project_constraint_pipeline->push_constant_ranges.emplace_back();
    project_push_constant_range.size = sizeof(StiffRodProjectConstraintConstant);
    project_push_constant_range.offset = 0;
    project_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    project_constraint_pipeline->Initialize();
  }

  if (strands_physics_descriptor_sets.empty()) {
    strands_physics_descriptor_sets.resize(max_frame_in_flight);
    for (auto& i : strands_physics_descriptor_sets) {
      i = std::make_shared<DescriptorSet>(layout);
    }
  }
}

void DsStiffRod::InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                                DynamicStrands& target_dynamic_strands) {
  per_strand_data_list.resize(target_dynamic_strands.strands.size());
  rod_constraints.clear();
  rod_constraints.reserve(glm::max(0, static_cast<int>(target_dynamic_strands.strand_segments.size()) -
                                          static_cast<int>(target_dynamic_strands.strands.size())));
  for (uint32_t strand_index = 0; strand_index < target_dynamic_strands.strands.size(); strand_index++) {
    const auto& gpu_strand = target_dynamic_strands.strands[strand_index];
    auto& stiff_rod_per_strand_data = per_strand_data_list[strand_index];
    if (gpu_strand.begin_segment_handle == -1) {
      stiff_rod_per_strand_data.begin_rod_constraint_handle = -1;
      stiff_rod_per_strand_data.end_rod_constraint_handle = -1;
      continue;
    }
    if (gpu_strand.begin_segment_handle == gpu_strand.end_segment_handle) {
      stiff_rod_per_strand_data.begin_rod_constraint_handle = -1;
      stiff_rod_per_strand_data.end_rod_constraint_handle = -1;
      continue;
    }
    stiff_rod_per_strand_data.begin_rod_constraint_handle = static_cast<int>(rod_constraints.size());
    int segment_handle = gpu_strand.begin_segment_handle;
    int next_segment_handle = target_dynamic_strands.strand_segments[segment_handle].next_handle;
    while (next_segment_handle != -1) {
      rod_constraints.emplace_back();
      auto& rod_constraint = rod_constraints.back();
      rod_constraint.next_constraint_handle = rod_constraints.size();
      rod_constraint.segment0_index = segment_handle;
      rod_constraint.segment1_index = next_segment_handle;
      const auto& segment0 = target_dynamic_strands.strand_segments[segment_handle];
      const auto& segment1 = target_dynamic_strands.strand_segments[next_segment_handle];
      const auto& q0 = segment0.q0;
      const auto& q1 = segment1.q0;
      const auto& l0 = segment0.length;
      const auto& l1 = segment1.length;
      const float segment_length = (l0 + l1) * .5f;
      rod_constraint.rest_darboux_vector = ComputeDarbouxVector(q0, q1, segment_length);
      rod_constraint.average_segment_length = segment_length;
      const auto rotation0_transpose = glm::transpose(glm::mat3_cast(q0));
      const auto rotation1_transpose = glm::transpose(glm::mat3_cast(q1));
      const auto& p0 = segment0.x0;
      const auto& p1 = segment1.x0;
      const auto constraint_position0 = p0 + q0 * glm::vec3(0, 0, -1) * segment0.length * .5f;
      const auto constraint_position1 = p1 + q1 * glm::vec3(0, 0, 1) * segment1.length * .5f;
      assert(glm::distance(constraint_position0, constraint_position1) < 0.0001f);
      const auto constraint_position = (constraint_position0 + constraint_position1) * .5f;
      rod_constraint.constraint_info =
          glm::mat4(glm::vec4(rotation0_transpose * (constraint_position - p0), 0.f),
                    glm::vec4(rotation1_transpose * (constraint_position - p1), 0.f),
                    glm::vec4(constraint_position, 0.f), glm::vec4(constraint_position, 0.f));
      const auto radius = (segment0.radius + segment1.radius) * .5f;
      const float youngs_modulus = initialize_parameters.youngs_modulus * 1000000000.f;
      const float shear_modulus = initialize_parameters.torsion_modulus * 1000000000.f;
      const auto second_moment_of_area = glm::pi<float>() * std::pow(radius * .5f, 4.f);
      const auto polar_moment_of_inertia = glm::pi<float>() * std::pow(radius, 4.f) / 2.f;

      const auto stretching_stiffness = youngs_modulus * glm::pi<float>() * radius * radius / segment_length;
      const auto bending_stiffness = youngs_modulus * second_moment_of_area / glm::pow(segment_length, 3.f);
      const auto torsional_stiffness = shear_modulus * polar_moment_of_inertia / segment_length;

      rod_constraint.stiffness_coefficient_k = glm::vec3(stretching_stiffness, bending_stiffness, torsional_stiffness);

      segment_handle = next_segment_handle;
      next_segment_handle = target_dynamic_strands.strand_segments[segment_handle].next_handle;
    }
    rod_constraints.back().next_constraint_handle = -1;
    stiff_rod_per_strand_data.end_rod_constraint_handle = static_cast<int>(rod_constraints.size()) - 1;
  }

  per_strand_data_list_buffer->UploadVector(per_strand_data_list);
  rod_constraints_buffer->UploadVector(rod_constraints);
}

void DsStiffRod::Project(const DynamicStrands::PhysicsParameters& physics_parameters,
                         DynamicStrands& target_dynamic_strands) {
  StiffRodInitConstraintConstant init_push_constant;
  init_push_constant.constraint_size = rod_constraints.size();
  init_push_constant.time_step = physics_parameters.time_step / physics_parameters.sub_step;
  StiffRodUpdateConstraintConstant update_constraint_constant;
  update_constraint_constant.constraint_size = rod_constraints.size();

  StiffRodProjectConstraintConstant project_constraint_constant;
  project_constraint_constant.strand_size = per_strand_data_list.size();

  if (physics_parameters.gpu) {
    const auto current_frame_index = Platform::GetCurrentFrameIndex();
    rod_properties_buffer[current_frame_index]->Upload(rod_properties);

    strands_physics_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
        0, rod_properties_buffer[current_frame_index]);
    strands_physics_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(1, per_strand_data_list_buffer);
    strands_physics_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(2, rod_constraints_buffer);

    const uint32_t task_work_group_invocations =
        Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      Platform::EverythingBarrier(vk_command_buffer);
      for (int sub_step_i = 0; sub_step_i < physics_parameters.sub_step; sub_step_i++) {
        init_constraint_pipeline->Bind(vk_command_buffer);
        init_constraint_pipeline->BindDescriptorSet(
            vk_command_buffer, 0,
            target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        init_constraint_pipeline->BindDescriptorSet(
            vk_command_buffer, 1, strands_physics_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

        init_constraint_pipeline->PushConstant(vk_command_buffer, 0, init_push_constant);
        vkCmdDispatch(vk_command_buffer,
                      Platform::DivUp(init_push_constant.constraint_size, task_work_group_invocations), 1, 1);
        Platform::EverythingBarrier(vk_command_buffer);
        for (int iteration_i = 0; iteration_i < physics_parameters.max_iteration; iteration_i++) {
          update_constraint_pipeline->Bind(vk_command_buffer);
          update_constraint_pipeline->BindDescriptorSet(
              vk_command_buffer, 0,
              target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
          update_constraint_pipeline->BindDescriptorSet(
              vk_command_buffer, 1, strands_physics_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

          update_constraint_pipeline->PushConstant(vk_command_buffer, 0, update_constraint_constant);
          vkCmdDispatch(vk_command_buffer,
                        Platform::DivUp(update_constraint_constant.constraint_size, task_work_group_invocations), 1, 1);
          Platform::EverythingBarrier(vk_command_buffer);

          project_constraint_pipeline->Bind(vk_command_buffer);
          project_constraint_pipeline->BindDescriptorSet(
              vk_command_buffer, 0,
              target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
          project_constraint_pipeline->BindDescriptorSet(
              vk_command_buffer, 1, strands_physics_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

          project_constraint_pipeline->PushConstant(vk_command_buffer, 0, project_constraint_constant);
          vkCmdDispatch(vk_command_buffer,
                        Platform::DivUp(project_constraint_constant.strand_size, task_work_group_invocations), 1, 1);
          Platform::EverythingBarrier(vk_command_buffer);
        }
      }
    });
  } else {
    const auto init_constraints = [&]() {
      const auto init_stretch_bending_twisting_constraint =
          [&](const glm::vec3& stiffness_coefficient_k, glm::vec3& stretch_compliance,
              glm::vec3& bending_and_torsion_compliance, vec6& lambda_sum) {
            const float time_step_quadratic = init_push_constant.time_step * init_push_constant.time_step;

            stretch_compliance = glm::vec3(time_step_quadratic / stiffness_coefficient_k[0],
                                           time_step_quadratic / stiffness_coefficient_k[0],
                                           time_step_quadratic / stiffness_coefficient_k[0]);

            bending_and_torsion_compliance = glm::vec3(time_step_quadratic / stiffness_coefficient_k[1],
                                                       time_step_quadratic / stiffness_coefficient_k[1],
                                                       time_step_quadratic / stiffness_coefficient_k[2]);

            lambda_sum.v[0] = 0.0;
            lambda_sum.v[1] = 0.0;
            lambda_sum.v[2] = 0.0;
            lambda_sum.v[3] = 0.0;
            lambda_sum.v[4] = 0.0;
            lambda_sum.v[5] = 0.0;
          };

      Jobs::RunParallelFor(rod_constraints.size(), [&](const size_t constraint_index) {
        auto& rod_constraint = rod_constraints[constraint_index];
        init_stretch_bending_twisting_constraint(
            rod_constraint.stiffness_coefficient_k, rod_constraint.stretch_compliance,
            rod_constraint.bending_and_torsion_compliance, rod_constraint.lambda_sum);
      });
    };
    const auto update_constraints = [&]() {
      const auto update_stretch_bending_twisting_constraint = [](const glm::vec3& x0, const glm::quat& q0,
                                                                 const glm::vec3& x1, const glm::quat& q1,
                                                                 glm::mat4& constraint_info) {
        // constraintInfo contains
        // 0:	connector in segment 0 (local)
        // 1:	connector in segment 1 (local)
        // 2:	connector in segment 0 (global)
        // 3:	connector in segment 1 (global)

        // compute world space positions of connectors
        const glm::mat3 rot0 = glm::mat3_cast(q0);
        const glm::mat3 rot1 = glm::mat3_cast(q1);

        constraint_info[2] = glm::vec4(rot0 * glm::vec3(constraint_info[0]) + x0, 0.0f);
        constraint_info[3] = glm::vec4(rot1 * glm::vec3(constraint_info[1]) + x1, 0.0f);
      };

      Jobs::RunParallelFor(rod_constraints.size(), [&](const size_t constraint_index) {
        auto& rod_constraint = rod_constraints[constraint_index];
        const auto& strand_segment0 = target_dynamic_strands.strand_segments[rod_constraint.segment0_index];
        const auto& strand_segment1 = target_dynamic_strands.strand_segments[rod_constraint.segment1_index];
        update_stretch_bending_twisting_constraint(strand_segment0.x, strand_segment0.q, strand_segment1.x,
                                                   strand_segment1.q, rod_constraint.constraint_info);
      });
    };
    const auto solve_constraints = [&]() {
      const auto solve_stretch_bending_twisting_constraints =
          [&](const int constraint_handle, const float inv_mass0, const glm::vec3& x0,
              const glm::mat3& inverse_inertia0, const glm::quat& q0, const float inv_mass1, const glm::vec3& x1,
              const glm::mat3& inverse_inertia1, const glm::quat& q1, const glm::vec3& rest_darboux_vector,
              const float average_segment_length, const glm::vec3& stretch_compliance,
              const glm::vec3& bending_and_torsion_compliance, const glm::mat4& constraint_info,
              glm::vec3& x0_correction, glm::quat& q0_correction, glm::vec3& x1_correction, glm::quat& q1_correction,
              vec6& lambda_sum) {
            const auto compute_bending_and_torsion_jacobians = [](const glm::quat& q0, const glm::quat& q1,
                                                                  float segment_length, glm::mat4& j_omega0,
                                                                  glm::mat4& j_omega1) {
              j_omega0 = glm::mat4(-q1[3], -q1[2], q1[1], q1[0], q1[2], -q1[3], -q1[0], q1[1], -q1[1], q1[0], -q1[3],
                                   q1[2], 0.0, 0.0, 0.0, 0.0);
              j_omega1 = glm::mat4(q0[3], q0[2], -q0[1], -q0[0], -q0[2], q0[3], q0[0], -q0[1], q0[1], -q0[0], q0[3],
                                   -q0[2], 0.0, 0.0, 0.0, 0.0);
              j_omega0 *= 2.0 / segment_length;
              j_omega1 *= 2.0 / segment_length;
            };

            const auto compute_matrix_g = [](const glm::quat& q, glm::mat4& g) {
              g = glm::mat4(q[3], q[2], -q[1], 0.0, -q[2], q[3], q[0], 0.0, q[1], -q[0], q[3], 0.0, -q[0], -q[1], -q[2],
                            0.0) *
                  0.5f;
            };

            const auto compute_matrix_k = [](const glm::vec3& connector, const float inv_mass, const glm::vec3& x,
                                             const glm::mat3& inverse_inertia, glm::mat3& k) {
              if (inv_mass != 0.0) {
                glm::vec3 v = connector - x;
                const float a = v[0];
                const float b = v[1];
                const float c = v[2];

                const float j11 = inverse_inertia[0][0];
                const float j12 = inverse_inertia[0][1];
                const float j13 = inverse_inertia[0][2];
                const float j22 = inverse_inertia[1][1];
                const float j23 = inverse_inertia[1][2];
                const float j33 = inverse_inertia[2][2];

                k[0][0] = c * c * j22 - b * c * (j23 + j23) + b * b * j33 + inv_mass;
                k[0][1] = -(c * c * j12) + a * c * j23 + b * c * j13 - a * b * j33;
                k[0][2] = b * c * j12 - a * c * j22 - b * b * j13 + a * b * j23;
                k[1][0] = k[0][1];
                k[1][1] = c * c * j11 - a * c * (j13 + j13) + a * a * j33 + inv_mass;
                k[1][2] = -(b * c * j11) + a * c * j12 + a * b * j13 - a * a * j23;
                k[2][0] = k[0][2];
                k[2][1] = k[1][2];
                k[2][2] = b * b * j11 - a * b * (j12 + j12) + a * a * j22 + inv_mass;
              } else {
                k = glm::mat3(0, 0, 0, 0, 0, 0, 0, 0, 0);
              }
            };

            const auto cross_product_matrix = [](const glm::vec3& v) {
              return glm::mat3(0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0);
            };
            const auto solve_ldlt = [](const mat6& A, vec6& x, const vec6& b) {
              mat6 L;
              L.v[0].v[0] = 1.0;
              L.v[0].v[1] = 0.0;
              L.v[0].v[2] = 0.0;
              L.v[0].v[3] = 0.0;
              L.v[0].v[4] = 0.0;
              L.v[0].v[5] = 0.0;

              L.v[1].v[0] = 0.0;
              L.v[1].v[1] = 1.0;
              L.v[1].v[2] = 0.0;
              L.v[1].v[3] = 0.0;
              L.v[1].v[4] = 0.0;
              L.v[1].v[5] = 0.0;

              L.v[2].v[0] = 0.0;
              L.v[2].v[1] = 0.0;
              L.v[2].v[2] = 1.0;
              L.v[2].v[3] = 0.0;
              L.v[2].v[4] = 0.0;
              L.v[2].v[5] = 0.0;

              L.v[3].v[0] = 0.0;
              L.v[3].v[1] = 0.0;
              L.v[3].v[2] = 0.0;
              L.v[3].v[3] = 1.0;
              L.v[3].v[4] = 0.0;
              L.v[3].v[5] = 0.0;

              L.v[4].v[0] = 0.0;
              L.v[4].v[1] = 0.0;
              L.v[4].v[2] = 0.0;
              L.v[4].v[3] = 0.0;
              L.v[4].v[4] = 1.0;
              L.v[4].v[5] = 0.0;

              L.v[5].v[0] = 0.0;
              L.v[5].v[1] = 0.0;
              L.v[5].v[2] = 0.0;
              L.v[5].v[3] = 0.0;
              L.v[5].v[4] = 0.0;
              L.v[5].v[5] = 1.0;

              vec6 D;
              D.v[0] = 0.0;
              D.v[1] = 0.0;
              D.v[2] = 0.0;
              D.v[3] = 0.0;
              D.v[4] = 0.0;
              D.v[5] = 0.0;

              // Decomposition
              for (int i = 0; i < 6; i++) {
                // Compute diagonal elements of D
                D.v[i] = A.v[i].v[i];
                for (int k = 0; k < i; k++) {
                  D.v[i] -= L.v[i].v[k] * L.v[i].v[k] * D.v[k];
                }
                // Compute elements of L (below the diagonal)
                for (int j = i + 1; j < 6; j++) {
                  L.v[j].v[i] = A.v[j].v[i];
                  for (int k = 0; k < i; k++) {
                    L.v[j].v[i] -= L.v[j].v[k] * L.v[i].v[k] * D.v[k];
                  }
                  L.v[j].v[i] /= D.v[i];
                }
              }
              vec6 y, z;
              // Forward substitution
              for (int i = 0; i < 6; i++) {
                y.v[i] = b.v[i];
                for (int j = 0; j < i; j++) {
                  y.v[i] -= L.v[i].v[j] * y.v[j];
                }
              }

              // Diagonal solve
              for (int i = 0; i < 6; i++) {
                z.v[i] = y.v[i] / D.v[i];
              }

              // Backward substitution
              for (int i = 5; i >= 0; i--) {
                x.v[i] = z.v[i];
                for (int j = i + 1; j < 6; j++) {
                  x.v[i] -= L.v[j].v[i] * x.v[j];
                }
              }
            };

            glm::vec3 omega = ComputeDarbouxVector(q0, q1, average_segment_length);
            glm::mat4 j_omega0, j_omega1;
            compute_bending_and_torsion_jacobians(q0, q1, average_segment_length, j_omega0, j_omega1);
            glm::mat4 g0, g1;
            compute_matrix_g(q0, g0);
            compute_matrix_g(q1, g1);

            glm::mat3 j_omega_g0 = glm::mat3(j_omega0 * g0);
            glm::mat3 j_omega_g1 = glm::mat3(j_omega1 * g1);

            glm::vec3 connector0 = constraint_info[2];
            glm::vec3 connector1 = constraint_info[3];

            glm::vec3 stretch_violation = connector0 - connector1;
            glm::vec3 bending_and_torsion_violation = omega - rest_darboux_vector;

            stretch_violation += stretch_compliance * glm::vec3(lambda_sum.v[0], lambda_sum.v[1], lambda_sum.v[2]);
            bending_and_torsion_violation +=
                bending_and_torsion_compliance * glm::vec3(lambda_sum.v[3], lambda_sum.v[4], lambda_sum.v[5]);

            vec6 rhs;
            rhs.v[0] = -stretch_violation.x;
            rhs.v[1] = -stretch_violation.y;
            rhs.v[2] = -stretch_violation.z;
            rhs.v[3] = -bending_and_torsion_violation.x;
            rhs.v[4] = -bending_and_torsion_violation.y;
            rhs.v[5] = -bending_and_torsion_violation.z;

            glm::mat3 k0, k1;
            compute_matrix_k(connector0, inv_mass0, x0, inverse_inertia0, k0);
            compute_matrix_k(connector1, inv_mass1, x1, inverse_inertia1, k1);
            glm::mat3 k = k0 + k1;

            vec6 delta_lambda;

            glm::vec3 r0 = connector0 - x0;
            glm::vec3 r1 = connector1 - x1;

            glm::mat3 r0_cross_t = cross_product_matrix(-r0);
            glm::mat3 r1_cross_t = cross_product_matrix(-r1);

            glm::mat3 off_diag = glm::mat3(0, 0, 0, 0, 0, 0, 0, 0, 0);
            if (inv_mass0 != 0.0) {
              off_diag = j_omega_g0 * inverse_inertia0 * r0_cross_t * -1.0f;
            }
            if (inv_mass1 != 0.0) {
              off_diag += j_omega_g1 * inverse_inertia1 * r1_cross_t;
            }
            glm::mat3 off_diag_t = transpose(off_diag);

            glm::mat3 m_inv_jt0 = inverse_inertia0 * transpose(j_omega_g0);
            glm::mat3 m_inv_jt1 = inverse_inertia1 * transpose(j_omega_g1);

            glm::mat3 jmjt_omega = glm::mat3(0);
            if (inv_mass0 != 0.0) {
              jmjt_omega = j_omega_g0 * m_inv_jt0;
            }
            if (inv_mass1 != 0.0) {
              jmjt_omega += j_omega_g1 * m_inv_jt1;
            }

            constexpr bool use_eigen = true;
            if (use_eigen) {
              using Vector6r = Eigen::Matrix<float, 6, 1, Eigen::DontAlign>;
              using Matrix6r = Eigen::Matrix<float, 6, 6, Eigen::DontAlign>;
              using Vector6r = Eigen::Matrix<float, 6, 1, Eigen::DontAlign>;
              Matrix6r jmjt(Matrix6r::Zero());

              jmjt(0, 0) = k[0][0];
              jmjt(0, 1) = k[0][1];
              jmjt(0, 2) = k[0][2];
              jmjt(1, 0) = k[1][0];
              jmjt(1, 1) = k[1][1];
              jmjt(1, 2) = k[1][2];
              jmjt(2, 0) = k[2][0];
              jmjt(2, 1) = k[2][1];
              jmjt(2, 2) = k[2][2];

              jmjt(3, 0) = off_diag[0][0];
              jmjt(3, 1) = off_diag[0][1];
              jmjt(3, 2) = off_diag[0][2];
              jmjt(4, 0) = off_diag[1][0];
              jmjt(4, 1) = off_diag[1][1];
              jmjt(4, 2) = off_diag[1][2];
              jmjt(5, 0) = off_diag[2][0];
              jmjt(5, 1) = off_diag[2][1];
              jmjt(5, 2) = off_diag[2][2];

              jmjt(0, 3) = off_diag_t[0][0];
              jmjt(0, 4) = off_diag_t[0][1];
              jmjt(0, 5) = off_diag_t[0][2];
              jmjt(1, 3) = off_diag_t[1][0];
              jmjt(1, 4) = off_diag_t[1][1];
              jmjt(1, 5) = off_diag_t[1][2];
              jmjt(2, 3) = off_diag_t[2][0];
              jmjt(2, 4) = off_diag_t[2][1];
              jmjt(2, 5) = off_diag_t[2][2];

              jmjt(3, 3) = jmjt_omega[0][0];
              jmjt(3, 4) = jmjt_omega[0][1];
              jmjt(3, 5) = jmjt_omega[0][2];
              jmjt(4, 3) = jmjt_omega[1][0];
              jmjt(4, 4) = jmjt_omega[1][1];
              jmjt(4, 5) = jmjt_omega[1][2];
              jmjt(5, 3) = jmjt_omega[2][0];
              jmjt(5, 4) = jmjt_omega[2][1];
              jmjt(5, 5) = jmjt_omega[2][2];

              // JMJT.block<3, 3>(3, 3) = JMJTOmega;

              // add compliance
              jmjt(0, 0) += stretch_compliance[0];
              jmjt(1, 1) += stretch_compliance[1];
              jmjt(2, 2) += stretch_compliance[2];
              jmjt(3, 3) += bending_and_torsion_compliance[0];
              jmjt(4, 4) += bending_and_torsion_compliance[1];
              jmjt(5, 5) += bending_and_torsion_compliance[2];

              // solve linear equation system (Equation 19)
              auto decomposition(jmjt.ldlt());

              Vector6r rhs_e;
              rhs_e(0) = rhs.v[0];
              rhs_e(1) = rhs.v[1];
              rhs_e(2) = rhs.v[2];
              rhs_e(3) = rhs.v[3];
              rhs_e(4) = rhs.v[4];
              rhs_e(5) = rhs.v[5];
              Vector6r delta_lambda_e(decomposition.solve(rhs_e));
              delta_lambda.v[0] = delta_lambda_e(0);
              delta_lambda.v[1] = delta_lambda_e(1);
              delta_lambda.v[2] = delta_lambda_e(2);
              delta_lambda.v[3] = delta_lambda_e(3);
              delta_lambda.v[4] = delta_lambda_e(4);
              delta_lambda.v[5] = delta_lambda_e(5);

            } else {
              mat6 jmjt;
              jmjt.v[0].v[0] = k[0][0];
              jmjt.v[0].v[1] = k[0][1];
              jmjt.v[0].v[2] = k[0][2];
              jmjt.v[1].v[0] = k[1][0];
              jmjt.v[1].v[1] = k[1][1];
              jmjt.v[1].v[2] = k[1][2];
              jmjt.v[2].v[0] = k[2][0];
              jmjt.v[2].v[1] = k[2][1];
              jmjt.v[2].v[2] = k[2][2];

              jmjt.v[3].v[0] = off_diag[0][0];
              jmjt.v[3].v[1] = off_diag[0][1];
              jmjt.v[3].v[2] = off_diag[0][2];
              jmjt.v[4].v[0] = off_diag[1][0];
              jmjt.v[4].v[1] = off_diag[1][1];
              jmjt.v[4].v[2] = off_diag[1][2];
              jmjt.v[5].v[0] = off_diag[2][0];
              jmjt.v[5].v[1] = off_diag[2][1];
              jmjt.v[5].v[2] = off_diag[2][2];

              jmjt.v[0].v[3] = off_diag_t[0][0];
              jmjt.v[0].v[4] = off_diag_t[0][1];
              jmjt.v[0].v[5] = off_diag_t[0][2];
              jmjt.v[1].v[3] = off_diag_t[1][0];
              jmjt.v[1].v[4] = off_diag_t[1][1];
              jmjt.v[1].v[5] = off_diag_t[1][2];
              jmjt.v[2].v[3] = off_diag_t[2][0];
              jmjt.v[2].v[4] = off_diag_t[2][1];
              jmjt.v[2].v[5] = off_diag_t[2][2];

              jmjt.v[3].v[3] = jmjt_omega[0][0];
              jmjt.v[3].v[4] = jmjt_omega[0][1];
              jmjt.v[3].v[5] = jmjt_omega[0][2];
              jmjt.v[4].v[3] = jmjt_omega[1][0];
              jmjt.v[4].v[4] = jmjt_omega[1][1];
              jmjt.v[4].v[5] = jmjt_omega[1][2];
              jmjt.v[5].v[3] = jmjt_omega[2][0];
              jmjt.v[5].v[4] = jmjt_omega[2][1];
              jmjt.v[5].v[5] = jmjt_omega[2][2];

              jmjt.v[0].v[0] += stretch_compliance.x;
              jmjt.v[1].v[1] += stretch_compliance.y;
              jmjt.v[2].v[2] += stretch_compliance.z;
              jmjt.v[3].v[3] += bending_and_torsion_compliance.x;
              jmjt.v[4].v[4] += bending_and_torsion_compliance.y;
              jmjt.v[5].v[5] += bending_and_torsion_compliance.z;

              solve_ldlt(jmjt, delta_lambda, rhs);
            }
            lambda_sum.v[0] += delta_lambda.v[0];
            lambda_sum.v[1] += delta_lambda.v[1];
            lambda_sum.v[2] += delta_lambda.v[2];
            lambda_sum.v[3] += delta_lambda.v[3];
            lambda_sum.v[4] += delta_lambda.v[4];
            lambda_sum.v[5] += delta_lambda.v[5];

            glm::vec3 delta_lambda_stretch = glm::vec3(delta_lambda.v[0], delta_lambda.v[1], delta_lambda.v[2]);
            glm::vec3 delta_lambda_bending_and_torsion =
                glm::vec3(delta_lambda.v[3], delta_lambda.v[4], delta_lambda.v[5]);

            x0_correction = glm::vec3(0, 0, 0);
            q0_correction = glm::quat(0, 0, 0, 0);
            x1_correction = glm::vec3(0, 0, 0);
            q1_correction = glm::quat(0, 0, 0, 0);

            if (inv_mass0 != 0.0) {
              x0_correction += inv_mass0 * delta_lambda_stretch;
              glm::vec3 v = inverse_inertia0 * r0_cross_t * (-1.0f * delta_lambda_stretch) +
                            m_inv_jt0 * delta_lambda_bending_and_torsion;
              q0_correction.x = g0[0][0] * v[0] + g0[0][1] * v[1] + g0[0][2] * v[2];
              q0_correction.y = g0[1][0] * v[0] + g0[1][1] * v[1] + g0[1][2] * v[2];
              q0_correction.z = g0[2][0] * v[0] + g0[2][1] * v[1] + g0[2][2] * v[2];
              q0_correction.w = g0[3][0] * v[0] + g0[3][1] * v[1] + g0[3][2] * v[2];
            }

            if (inv_mass1 != 0.0) {
              x1_correction -= inv_mass1 * delta_lambda_stretch;
              glm::vec3 v =
                  inverse_inertia1 * r1_cross_t * delta_lambda_stretch + m_inv_jt1 * delta_lambda_bending_and_torsion;
              q1_correction.x = g1[0][0] * v[0] + g1[0][1] * v[1] + g1[0][2] * v[2];
              q1_correction.y = g1[1][0] * v[0] + g1[1][1] * v[1] + g1[1][2] * v[2];
              q1_correction.z = g1[2][0] * v[0] + g1[2][1] * v[1] + g1[2][2] * v[2];
              q1_correction.w = g1[3][0] * v[0] + g1[3][1] * v[1] + g1[3][2] * v[2];
            }

            GpuRodConstraint rod_constraint = rod_constraints[constraint_handle];
            rod_constraint.omega = omega;
            rod_constraint.j_omega0 = j_omega0;
            rod_constraint.j_omega1 = j_omega1;
            rod_constraint.g0 = g0;
            rod_constraint.g1 = g1;
            rod_constraint.j_omega_g0 = glm::mat4(j_omega_g0);
            rod_constraint.j_omega_g1 = glm::mat4(j_omega_g1);

            rod_constraint.connector0 = connector0;
            rod_constraint.connector1 = connector1;

            rod_constraint.stretch_violation = stretch_violation;
            rod_constraint.bending_and_torsion_violation = bending_and_torsion_violation;

            rod_constraint.delta_lambda_stretch = delta_lambda_stretch;
            rod_constraint.delta_lambda_bending_and_torsion = delta_lambda_bending_and_torsion;

            rod_constraint.x0_correction = x0_correction;
            rod_constraint.x1_correction = x1_correction;

            rod_constraint.q0_correction = q0_correction;
            rod_constraint.q1_correction = q1_correction;

            rod_constraint.x[0] = delta_lambda.v[0];
            rod_constraint.x[1] = delta_lambda.v[1];
            rod_constraint.x[2] = delta_lambda.v[2];
            rod_constraint.x[3] = delta_lambda.v[3];
            rod_constraint.x[4] = delta_lambda.v[4];
            rod_constraint.x[5] = delta_lambda.v[5];

            rod_constraint.rhs[0] = rhs.v[0];
            rod_constraint.rhs[1] = rhs.v[1];
            rod_constraint.rhs[2] = rhs.v[2];
            rod_constraint.rhs[3] = rhs.v[3];
            rod_constraint.rhs[4] = rhs.v[4];
            rod_constraint.rhs[5] = rhs.v[5];

            rod_constraints[constraint_handle] = rod_constraint;
          };

      const auto project_constraint = [&](const int rod_constraint_handle) {
        GpuRodConstraint& rod_constraint = rod_constraints[rod_constraint_handle];
        glm::vec3 x0_correction, x1_correction;
        glm::quat q0_correction, q1_correction;
        glm::vec3 fake_compliance = glm::vec3(0);
        auto& segment0 = target_dynamic_strands.strand_segments[rod_constraint.segment0_index];
        auto& segment1 = target_dynamic_strands.strand_segments[rod_constraint.segment1_index];
        solve_stretch_bending_twisting_constraints(
            rod_constraint_handle, segment0.inv_mass, segment0.x, segment0.inv_inertia_w, segment0.q, segment1.inv_mass,
            segment1.x, segment1.inv_inertia_w, segment1.q, rod_constraint.rest_darboux_vector,
            rod_constraint.average_segment_length, fake_compliance, fake_compliance, rod_constraint.constraint_info,
            x0_correction, q0_correction, x1_correction, q1_correction, rod_constraint.lambda_sum);

        if (segment0.inv_mass != 0.0) {
          segment0.x = segment0.x + x0_correction;
          segment0.q = normalize(segment0.q + q0_correction);
        }

        if (segment1.inv_mass != 0.0) {
          segment1.x = segment1.x + x1_correction;
          segment1.q = normalize(segment1.q + q0_correction);
        }
      };

      Jobs::RunParallelFor(per_strand_data_list.size(), [&](const size_t strand_index) {
        const auto& per_strand_data = per_strand_data_list[strand_index];
        int rod_constraint_handle = per_strand_data.begin_rod_constraint_handle;
        while (rod_constraint_handle != -1) {
          project_constraint(rod_constraint_handle);
          rod_constraint_handle = rod_constraints[rod_constraint_handle].next_constraint_handle;
        }
      });
    };
    for (int sub_step_i = 0; sub_step_i < physics_parameters.sub_step; sub_step_i++) {
      init_constraints();
      for (int iteration_i = 0; iteration_i < physics_parameters.max_iteration; iteration_i++) {
        update_constraints();
        solve_constraints();
      }
    }
  }
}

void DsStiffRod::DownloadData() {
  rod_constraints_buffer->DownloadVector(rod_constraints, rod_constraints.size());
  per_strand_data_list_buffer->DownloadVector(per_strand_data_list, per_strand_data_list.size());
}

void DsStiffRod::UploadData() {
  rod_constraints_buffer->UploadVector(rod_constraints);
  per_strand_data_list_buffer->UploadVector(per_strand_data_list);
}

glm::vec3 DsStiffRod::ComputeDarbouxVector(const glm::quat& q0, const glm::quat& q1,
                                           const float average_segment_length) {
  const auto relative_rotation = glm::conjugate(q0) * q1;
  return 2.f / average_segment_length * glm::vec3(relative_rotation.x, relative_rotation.y, relative_rotation.z);
}
