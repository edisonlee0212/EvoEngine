#include "DynamicStrands.hpp"

#include "DynamicStrandsPhysics.hpp"
#include "Shader.hpp"
#include "glm/gtc/matrix_access.hpp"
#include "glm/gtx/quaternion.hpp"
using namespace eco_sys_lab_plugin;

DynamicStrands::DynamicStrands() {
  if (!strands_layout) {
    strands_layout = std::make_shared<DescriptorSetLayout>();
    strands_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);

    strands_layout->Initialize();
  }

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

  device_strands_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segments_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_particles_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();
  strands_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : strands_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(strands_layout);
  }
}

void DynamicStrands::Step(const StepParameters& target_step_parameters) {
  if (segments.empty())
    return;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();

  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(0, device_strands_buffer, 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(1, device_segments_buffer, 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(2, device_particles_buffer, 0);

  if (target_step_parameters.physics) {
    Physics(target_step_parameters.physics_parameters, operators, pre_step, constraints);
  }

  if (target_step_parameters.render) {
    Render(target_step_parameters.render_parameters);
  }
}

void DynamicStrands::Upload() const {
  device_strands_buffer->UploadVector(strands);
  device_segments_buffer->UploadVector(segments);
  device_particles_buffer->UploadVector(particles);

  for (const auto& op : operators) {
    op->UploadData();
  }
  for (const auto& c : constraints) {
    c->UploadData();
  }
}

void DynamicStrands::Download() {
  device_strands_buffer->DownloadVector(strands, strands.size());
  device_segments_buffer->DownloadVector(segments, segments.size());
  device_particles_buffer->DownloadVector(particles, particles.size());
  for (const auto& op : operators) {
    op->DownloadData();
  }
  for (const auto& c : constraints) {
    c->DownloadData();
  }
}

void DynamicStrands::Clear() {
  strands.clear();
  segments.clear();
  particles.clear();
}

glm::vec3 DynamicStrands::ComputeInertiaTensorBox(const float mass, const float width, const float height,
                                                  const float depth) {
  return {
      mass / 12.f * (height * height + depth * depth),
      mass / 12.f * (width * width + depth * depth),
      mass / 12.f * (width * width + height * height),
  };
}

glm::vec3 DynamicStrands::ComputeInertiaTensorRod(const float mass, const float radius, const float length) {
  return {
      mass / 12.f * (radius * radius + length * length),
      mass / 12.f * (radius * radius + length * length),
      mass / 4.f * (radius * radius + radius * radius),
  };
}

void DynamicStrands::InitializeOperators(const InitializeParameters& initialize_parameters) const {
  for (const auto& i : operators)
    i->InitializeData(initialize_parameters, *this);
}

void DynamicStrands::InitializeConstraints(const InitializeParameters& initialize_parameters) const {
  for (const auto& i : constraints)
    i->InitializeData(initialize_parameters, *this);
}
