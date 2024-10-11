#include "DynamicStrandsPhysics.hpp"
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
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
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
                                    const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  
  PreStepPushConstant push_constant;
  push_constant.segment_size = target_dynamic_strands.strand_segments.size();
  push_constant.time_step = physics_parameters.time_step;
  push_constant.inv_time_step = 1.f / push_constant.time_step; 
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
                                     const DynamicStrands& target_dynamic_strands) {
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
                               const DynamicStrands& target_dynamic_strands) {
  if (commands.empty())
    return;

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  commands_buffer[current_frame_index]->UploadVector(commands);
  commands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(0, commands_buffer[current_frame_index]);

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
                              const DynamicStrands& target_dynamic_strands) {
  if (commands.empty())
    return;

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  commands_buffer[current_frame_index]->UploadVector(commands);

  commands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(0, commands_buffer[current_frame_index]);

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
                                const DynamicStrands& target_dynamic_strands) {
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
      rod_constraint.next_handle = rod_constraints.size();
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

      // compute bending and torsion stiffness of the K matrix diagonal; assumption: the rod axis follows the y-axis of
      // the local frame as with Blender's armatures
#define M_PI_4 0.785398163397448309616  // pi/4
      const auto second_moment_of_area(static_cast<float>(M_PI_4) *
                                       std::pow((segment0.radius + segment1.radius) * .5f, 4.f));
      const auto bending_stiffness(initialize_parameters.youngs_modulus * second_moment_of_area);
      const auto torsion_stiffness(2.f * initialize_parameters.torsion_modulus * second_moment_of_area);
      rod_constraint.stiffness_coefficient_k = glm::vec3(bending_stiffness, torsion_stiffness, bending_stiffness);

      segment_handle = next_segment_handle;
      next_segment_handle = target_dynamic_strands.strand_segments[segment_handle].next_handle;
    }
    rod_constraints.back().next_handle = -1;
    stiff_rod_per_strand_data.end_rod_constraint_handle = static_cast<int>(rod_constraints.size()) - 1;
  }

  per_strand_data_list_buffer->UploadVector(per_strand_data_list);
  rod_constraints_buffer->UploadVector(rod_constraints);
}

void DsStiffRod::Project(const DynamicStrands::PhysicsParameters& physics_parameters,
                         const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  rod_properties_buffer[current_frame_index]->Upload(rod_properties);

  strands_physics_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
      0, rod_properties_buffer[current_frame_index]);
  strands_physics_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(1, per_strand_data_list_buffer);
  strands_physics_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(2, rod_constraints_buffer);

  int num_sub_step = 1;
  int num_iteration = 1;

  StiffRodInitConstraintConstant init_push_constant;
  init_push_constant.constraint_size = rod_constraints.size();
  init_push_constant.time_step = physics_parameters.time_step / num_sub_step;
  init_push_constant.inv_time_step = 1.f / init_push_constant.time_step;

  StiffRodUpdateConstraintConstant update_constraint_constant;
  update_constraint_constant.constraint_size = rod_constraints.size();
  
  StiffRodProjectConstraintConstant project_constraint_constant;
  project_constraint_constant.strand_size = per_strand_data_list.size();

  

  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    Platform::EverythingBarrier(vk_command_buffer);
    for (int sub_step_i = 0; sub_step_i < num_sub_step; sub_step_i++) {
      init_constraint_pipeline->Bind(vk_command_buffer);
      init_constraint_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      init_constraint_pipeline->BindDescriptorSet(
          vk_command_buffer, 1, strands_physics_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

      init_constraint_pipeline->PushConstant(vk_command_buffer, 0, init_push_constant);
      vkCmdDispatch(vk_command_buffer, Platform::DivUp(init_push_constant.constraint_size, task_work_group_invocations),
                    1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
      for (int iteration_i = 0; iteration_i < num_iteration; iteration_i++) {
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
}

void DsStiffRod::DownloadData() {
  rod_constraints_buffer->DownloadVector(rod_constraints, rod_constraints.size());
  per_strand_data_list_buffer->DownloadVector(per_strand_data_list, per_strand_data_list.size());
}

glm::vec3 DsStiffRod::ComputeDarbouxVector(const glm::quat& q0, const glm::quat& q1,
                                           const float average_segment_length) {
  const auto relative_rotation = glm::conjugate(q0) * q1;
  return 2.f / average_segment_length * glm::vec3(relative_rotation.x, relative_rotation.y, relative_rotation.z);
}
