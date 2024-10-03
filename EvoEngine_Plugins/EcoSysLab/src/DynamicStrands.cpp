#include "DynamicStrands.hpp"
#include "Shader.hpp"
using namespace eco_sys_lab_plugin;

void DynamicStrands::Initialize(const InitializeParameters& initialize_parameters, const std::vector<Strand>& strands,
                                const std::vector<StrandSegment>& strand_segments) {
  Clear();
  this->strand_segments = strand_segments;
  this->strands.resize(strands.size());
  Jobs::RunParallelFor(strands.size(), [&](const size_t i) {
    this->strands[i].start_position = strands[i].start_position;
    this->strands[i].start_thickness = strands[i].start_thickness;
    this->strands[i].start_color = strands[i].start_color;
  });

  for (uint32_t i = 0; i < strands.size(); i++) {
    const auto& handles = strands[i].PeekStrandSegmentHandles();
    this->strands[i].strand_segment_handles_offset = static_cast<int32_t>(strand_segment_handles.size());
    this->strands[i].first_strand_segment_handle = handles.front();
    this->strands[i].last_strand_segment_handle = handles.back();
    this->strands[i].strand_segment_handles_size = static_cast<int32_t>(handles.size());
    strand_segment_handles.insert(strand_segment_handles.end(), handles.begin(), handles.end());
  }
  strand_segment_handles.resize(strand_segment_handles.size() + strand_segment_handles.size() % 4);
  const auto max_frame_count = Platform::GetMaxFramesInFlight();
  device_strand_segments_buffer.resize(max_frame_count);
  device_strands_buffer.resize(max_frame_count);
  device_strand_segment_handles_buffer.resize(max_frame_count);

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  for (uint32_t i = 0; i < max_frame_count; i++) {
    device_strand_segments_buffer[i] = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
    device_strands_buffer[i] = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
    device_strand_segment_handles_buffer[i] =
        std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
    Upload(i);
  }
}
/*
void DynamicStrands::Simulate(const SimulateParameters& simulate_parameters) {
  //1. Update velocity.
  Jobs::RunParallelFor(strand_segments.size(), [&](const auto& i) {
    auto& segment = strand_segments[i];
    segment.end_velocity += glm::vec3(0, -9.81f, 0) * simulate_parameters.d_t * segment.inv_end_mass;
  });

  //2. Update position.
  Jobs::RunParallelFor(strand_segments.size(), [&](const auto& i) {
    auto& segment = strand_segments[i];
    segment.end_position += segment.end_velocity * simulate_parameters.d_t;
  });

  //3. Update angular velocity.
  Jobs::RunParallelFor(strand_segments.size(), [&](const auto& i) {
    auto& segment = strand_segments[i];
  });

  // 4. Update rotation.
  Jobs::RunParallelFor(strand_segments.size(), [&](const auto& i) {
    auto& segment = strand_segments[i];
    auto dq = glm::quat(0.f, segment.angular_velocity * simulate_parameters.d_t);
    segment.rotation += dq * segment.rotation * .5f;
    segment.rotation = glm::normalize(segment.rotation);
  });

  for (uint32_t iteration = 0; iteration < simulate_parameters.solver_iterations; iteration++) {
    for (uint32_t i = 0; i < strand_segments.size(); i++) {
      if (i % 2 == 0) {

      }
    }
  }
}*/

void DynamicStrands::Render(const RenderParameters& render_parameters, const std::shared_ptr<Camera>& camera,
                            const GlobalTransform& camera_model, const GlobalTransform& strands_model) const {
  if (!Platform::Constants::support_mesh_shader) {
    EVOENGINE_LOG("Failed to render! Mesh shader unsupported!")
    return;
  }
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  static std::shared_ptr<Shader> task_shader{};
  static std::shared_ptr<Shader> mesh_shader{};
  static std::shared_ptr<Shader> frag_shader{};

  static std::shared_ptr<DescriptorSetLayout> strands_layout{};

  static std::shared_ptr<GraphicsPipeline> render_pipeline{};

  static std::vector<std::shared_ptr<DescriptorSet>> strands_descriptor_sets{};
  struct RenderPushConstant {};
  if (!render_pipeline) {
    // Load shader
    task_shader = std::make_shared<Shader>();
    task_shader->Set(ShaderType::Task, Platform::Constants::shader_global_defines,
                     std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Task/DynamicStrands.task");

    mesh_shader = std::make_shared<Shader>();
    mesh_shader->Set(ShaderType::Mesh, Platform::Constants::shader_global_defines,
                     std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Mesh/DynamicStrands.mesh");

    frag_shader = std::make_shared<Shader>();
    frag_shader->Set(ShaderType::Fragment, Platform::Constants::shader_global_defines,
                     std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/DynamicStrands.frag");
    // Descriptor set layout
    strands_layout = std::make_shared<DescriptorSetLayout>();
    strands_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);

    render_pipeline = std::make_shared<GraphicsPipeline>();
    render_pipeline->task_shader = task_shader;
    render_pipeline->mesh_shader = mesh_shader;

    render_pipeline->fragment_shader = frag_shader;
    render_pipeline->geometry_type = GeometryType::Mesh;
    auto per_frame_layout = Platform::GetDescriptorSetLayout("PER_FRAME_LAYOUT");
    render_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    render_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};

    auto& push_constant_range = render_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    render_pipeline->Initialize();
    const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();
    strands_descriptor_sets.resize(max_frame_in_flight);
    for (auto& i : strands_descriptor_sets) {
      i = std::make_shared<DescriptorSet>(strands_layout);
    }
  }
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
      0, device_strand_segments_buffer[current_frame_index], 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
      1, device_strands_buffer[current_frame_index], 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
      2, device_strand_segment_handles_buffer[current_frame_index], 0);
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    camera->GetRenderTexture()->Render(
        vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
          render_pipeline->Bind(vk_command_buffer);
          render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                             render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
          render_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                             strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

          const uint32_t count =
              (strands.size() + task_work_group_invocations - 1) / task_work_group_invocations;
          vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
        });
  });
}

void DynamicStrands::Upload(const uint32_t current_frame_index) const {
  device_strand_segments_buffer[current_frame_index]->UploadVector(this->strand_segments);
  device_strands_buffer[current_frame_index]->UploadVector(this->strands);
  device_strand_segments_buffer[current_frame_index]->UploadVector(this->strand_segment_handles);
}

void DynamicStrands::Download(const uint32_t current_frame_index) {
  device_strand_segments_buffer[current_frame_index]->DownloadVector(this->strand_segments,
                                                                     this->strand_segments.size());
  device_strands_buffer[current_frame_index]->DownloadVector(this->strands, this->strands.size());
  device_strand_segments_buffer[current_frame_index]->DownloadVector(this->strand_segment_handles,
                                                                     this->strand_segment_handles.size());
}

void DynamicStrands::Clear() {
  strand_segments.clear();
  strands.clear();
  strand_segment_handles.clear();
  device_strand_segments_buffer.clear();
  device_strands_buffer.clear();
  device_strand_segment_handles_buffer.clear();
}
