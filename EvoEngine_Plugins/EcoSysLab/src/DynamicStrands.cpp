#include "DynamicStrands.hpp"
#include "Shader.hpp"
using namespace eco_sys_lab_plugin;

DynamicStrands::DynamicStrands() {
  const auto max_frame_count = Platform::GetMaxFramesInFlight();
  assert(max_frame_count == 2);
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  current_version = 0;
  for (uint32_t i = 0; i < max_frame_count; i++) {
    device_ref_strand_segments_buffer[i] =
        std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
    device_ref_strands_buffer[i] = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
    device_ref_strand_segment_particles_buffer[i] =
        std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

    device_strand_segments_buffer[i] = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
    device_strands_buffer[i] = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
    device_strand_segment_particles_buffer[i] =
        std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

    device_buffer_version[i] = current_version;
  }
  if (!strands_layout) {
    strands_layout = std::make_shared<DescriptorSetLayout>();
    strands_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(5, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);

    strands_layout->Initialize();
  }
}

void DynamicStrands::Step() {
  Step(step_parameters);
}

void DynamicStrands::Step(const StepParameters& target_step_parameters) {
  current_left = !current_left;

  const auto current_frame_index = current_left ? 0 : 1;
  if (device_buffer_version[current_frame_index] != current_version) {
    Upload();
    device_buffer_version[current_frame_index] = current_version;
  }

  if (target_step_parameters.physics) {
    Physics(target_step_parameters.physics_parameters);
  }

  if (target_step_parameters.render) {
    Render(target_step_parameters.render_parameters);
  }
}

void DynamicStrands::UpdateData(const InitializeParameters& initialize_parameters,
                                const std::vector<Strand>& target_strands,
                                const std::vector<StrandSegment>& target_strand_segments) {
  Clear();
  assert(initialize_parameters.root_transform.GetScale() == glm::vec3(1.0f));
  ref_strands.resize(target_strands.size());
  ref_strand_segments.resize(target_strand_segments.size());
  ref_strand_segment_particles.resize(ref_strands.size() + ref_strand_segments.size());
  int particle_index = 0;
  for (uint32_t strand_index = 0; strand_index < target_strands.size(); strand_index++) {
    auto& target_strand = target_strands[strand_index];
    auto& first_particle = ref_strand_segment_particles[particle_index];
    first_particle.position = target_strand.start_position;
    first_particle.thickness = target_strand.start_thickness;
    first_particle.color = target_strand.start_color;
    particle_index++;
    const auto& handles = target_strand.PeekStrandSegmentHandles();
    for (const auto& i : handles) {
      ref_strand_segments[i].start_particle_index = particle_index - 1;
      ref_strand_segments[i].end_particle_index = particle_index;
      auto& particle = ref_strand_segment_particles[particle_index];
      particle.position = target_strand_segments[i].end_position;
      particle.thickness = target_strand_segments[i].end_thickness;
      particle.color = target_strand_segments[i].end_color;
      particle_index++;
    }
    ref_strands[strand_index].first_strand_segment_handle = handles.front();
    ref_strands[strand_index].last_strand_segment_handle = handles.back();
  }
  Jobs::RunParallelFor(target_strand_segments.size(), [&](const size_t i) {
    ref_strand_segments[i].prev_handle = target_strand_segments[i].GetPrevHandle();
    ref_strand_segments[i].next_handle = target_strand_segments[i].GetNextHandle();
    ref_strand_segments[i].strand_handle = target_strand_segments[i].GetStrandHandle();
    ref_strand_segments[i].index = target_strand_segments[i].GetIndex();
    ref_strand_segments[i].rotation =
        initialize_parameters.root_transform.GetRotation() * target_strand_segments[i].rotation;
  });
  current_version++;
}

/*
void DynamicStrands::Simulate(const SimulateParameters& simulate_parameters) {
  //1. Update velocity.
  Jobs::RunParallelFor(ref_strand_segments.size(), [&](const auto& i) {
    auto& segment = ref_strand_segments[i];
    segment.end_velocity += glm::vec3(0, -9.81f, 0) * simulate_parameters.d_t * segment.inv_end_mass;
  });

  //2. Update position.
  Jobs::RunParallelFor(ref_strand_segments.size(), [&](const auto& i) {
    auto& segment = ref_strand_segments[i];
    segment.end_position += segment.end_velocity * simulate_parameters.d_t;
  });

  //3. Update angular velocity.
  Jobs::RunParallelFor(ref_strand_segments.size(), [&](const auto& i) {
    auto& segment = ref_strand_segments[i];
  });

  // 4. Update rotation.
  Jobs::RunParallelFor(ref_strand_segments.size(), [&](const auto& i) {
    auto& segment = ref_strand_segments[i];
    auto dq = glm::quat(0.f, segment.angular_velocity * simulate_parameters.d_t);
    segment.rotation += dq * segment.rotation * .5f;
    segment.rotation = glm::normalize(segment.rotation);
  });

  for (uint32_t iteration = 0; iteration < simulate_parameters.solver_iterations; iteration++) {
    for (uint32_t i = 0; i < ref_strand_segments.size(); i++) {
      if (i % 2 == 0) {

      }
    }
  }
}*/

void DynamicStrands::Upload() const {
  const auto current_frame_index = current_left ? 0 : 1;
  device_ref_strand_segments_buffer[current_frame_index]->UploadVector(ref_strand_segments);
  device_ref_strands_buffer[current_frame_index]->UploadVector(ref_strands);
  device_ref_strand_segment_particles_buffer[current_frame_index]->UploadVector(ref_strand_segment_particles);

  device_strand_segments_buffer[current_frame_index]->UploadVector(ref_strand_segments);
  device_strands_buffer[current_frame_index]->UploadVector(ref_strands);
  device_strand_segment_particles_buffer[current_frame_index]->UploadVector(ref_strand_segment_particles);
}

void DynamicStrands::Clear() {
  ref_strand_segments.clear();
  ref_strands.clear();
  ref_strand_segment_particles.clear();
}

void DynamicStrands::BindStrandsDescriptorSet(const std::shared_ptr<DescriptorSet>& target_descriptor_set) const {
  const auto current_frame_index = current_left ? 0 : 1;
  target_descriptor_set->UpdateBufferDescriptorBinding(0, device_ref_strand_segments_buffer[current_frame_index], 0);
  target_descriptor_set->UpdateBufferDescriptorBinding(1, device_ref_strands_buffer[current_frame_index], 0);
  target_descriptor_set->UpdateBufferDescriptorBinding(
      2, device_ref_strand_segment_particles_buffer[current_frame_index], 0);

  target_descriptor_set->UpdateBufferDescriptorBinding(3, device_strand_segments_buffer[current_frame_index], 0);
  target_descriptor_set->UpdateBufferDescriptorBinding(4, device_strands_buffer[current_frame_index], 0);
  target_descriptor_set->UpdateBufferDescriptorBinding(5, device_strand_segment_particles_buffer[current_frame_index],
                                                       0);
}
