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

void DynamicStrands::InitializeStrandsGroup(const InitializeParameters& initialize_parameters,
    const StrandModelStrandGroup& strand_group) {
  Clear();
  assert(initialize_parameters.root_transform.GetScale() == glm::vec3(1.0f));
  const auto& target_strands = strand_group.PeekStrands();
  const auto& target_strand_segments = strand_group.PeekStrandSegments();
  strands.resize(target_strands.size());

  Jobs::RunParallelFor(target_strands.size(), [&](const size_t i) {
    auto& strand = strands[i];
    const auto& target_strand = target_strands[i];
    const auto& handles = target_strand.PeekStrandSegmentHandles();
    if (!handles.empty()) {
      strand.begin_segment_handle = handles.front();
      strand.end_segment_handle = handles.back();
    } else {
      strand.begin_segment_handle = -1;
      strand.end_segment_handle = -1;
    }
  });

  segments.resize(target_strand_segments.size());
  Jobs::RunParallelFor(target_strand_segments.size(), [&](const size_t i) {
    auto& segment = segments[i];
    const auto& target_strand_segment = target_strand_segments[i];
    segment.prev_handle = target_strand_segment.GetPrevHandle();
    segment.next_handle = target_strand_segment.GetNextHandle();
    segment.strand_handle = target_strand_segment.GetStrandHandle();
    segment.rest_length = strand_group.GetStrandSegmentLength(static_cast<int>(i));

    segment.color = target_strand_segment.end_color;
    segment.radius = target_strand_segment.end_thickness * .5f;
    segment.damping = initialize_parameters.angular_velocity_damping;
    segment.q0 = segment.q = segment.last_q = segment.old_q =
        initialize_parameters.root_transform.GetRotation() * target_strand_segment.rotation;
    segment.torque = glm::vec3(0.f);

    if (segment.prev_handle == -1 && initialize_parameters.static_root) {
      segment.inertia_tensor = segment.inv_inertia_tensor = glm::vec3(0.0f);
      segment.inv_mass = 0.f;
    } else {
      const float mass =
          segment.radius * segment.radius * glm::pi<float>() * initialize_parameters.wood_density * segment.rest_length;
      segment.inertia_tensor = ComputeInertiaTensorRod(mass, segment.radius, segment.rest_length);
      segment.inv_inertia_tensor = 1.f / segment.inertia_tensor;
      segment.inv_mass = 1.f / mass;
    }

    /*
    const float youngs_modulus = initialize_parameters.youngs_modulus * 1000000000.f;
    const float shear_modulus = initialize_parameters.torsion_modulus * 1000000000.f;
    const auto second_moment_of_area = glm::pi<float>() * std::pow(segment.radius * .5f, 4.f);
    const auto polar_moment_of_inertia = glm::pi<float>() * std::pow(segment.radius, 4.f) / 2.f;

    segment.stretching_stiffness = segment.shearing_stiffness =
        youngs_modulus * glm::pi<float>() * segment.radius * segment.radius / segment.rest_length;
    segment.bending_stiffness = youngs_modulus * second_moment_of_area / glm::pow(segment.rest_length, 3.f);
    segment.twisting_stiffness = shear_modulus * polar_moment_of_inertia / segment.rest_length;*/

    segment.stretching_stiffness = initialize_parameters.stretch_stiffness;
    segment.shearing_stiffness = initialize_parameters.shear_stiffness;
  });

  particles.resize(strands.size() + segments.size());

  int particle_index = 0;
  for (uint32_t strand_index = 0; strand_index < target_strands.size(); strand_index++) {
    auto& target_strand = target_strands[strand_index];
    const auto& handles = target_strand.PeekStrandSegmentHandles();
    if (handles.empty())
      continue;
    auto& strand = strands[strand_index];
    strand.begin_particle_handle = particle_index;
    auto& first_particle = particles[particle_index];
    first_particle.x = first_particle.last_x = first_particle.old_x =
        glm::vec4(initialize_parameters.root_transform.TransformPoint(target_strand.start_position), 0.0);
    first_particle.damping = initialize_parameters.velocity_damping;
    first_particle.x0 = first_particle.x;
    first_particle.acceleration = glm::vec3(0.f);
    first_particle.inv_mass = 0;
    first_particle.prev_handle = -1;
    particle_index++;
    first_particle.next_handle = particle_index;
    for (const auto& i : handles) {
      auto& segment = segments[i];
      segment.particle0_handle = particle_index - 1;
      segment.particle1_handle = particle_index;
      const auto& node_handle = strand_group.PeekStrandSegmentData(i).node_handle;
      particles[segment.particle0_handle].node_handle = node_handle;
      particles[segment.particle1_handle].node_handle = node_handle;
      particles[segment.particle0_handle].strand_handle = strand_index;
      particles[segment.particle1_handle].strand_handle = strand_index;

      const auto segment_length = strand_group.GetStrandSegmentLength(i);
      auto& particle = particles[particle_index];
      particle.prev_handle = particle_index - 1;
      particle.next_handle = particle_index + 1;
      particle.x = particle.last_x = particle.old_x =
          glm::vec4(initialize_parameters.root_transform.TransformPoint(target_strand_segments[i].end_position), 0.0);

      particle.x0 = particle.x;
      particle.acceleration = glm::vec3(0.f);
      particle.damping = initialize_parameters.velocity_damping;
      const float mass =
          segment.radius * segment.radius * glm::pi<float>() * initialize_parameters.wood_density * segment_length;
      particle.inv_mass = 1.f / mass;

      particle_index++;
    }
    particles.back().next_handle = -1;

    strand.begin_segment_handle = handles.front();
    strand.end_segment_handle = handles.back();

    strand.end_particle_handle = particle_index - 1;
  }
  Upload();

  InitializeOperators(initialize_parameters);
  InitializeConstraints(initialize_parameters);
}

void DynamicStrands::Step(const StepParameters& target_step_parameters) const {
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
