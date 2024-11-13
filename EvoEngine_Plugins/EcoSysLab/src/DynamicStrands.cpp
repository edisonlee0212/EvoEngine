#include "DynamicStrands.hpp"

#include "DynamicStrandsPhysics.hpp"
#include "Shader.hpp"
#include "glm/gtc/matrix_access.hpp"
#include "glm/gtx/quaternion.hpp"

#ifdef USE_CGAL
#  include <CGAL/Delaunay_triangulation_3.h>
#  include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#  include <CGAL/Triangulation_vertex_base_with_info_3.h>
#endif

#ifdef USE_CGAL
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_with_info_3<unsigned int, K> Vb;
typedef CGAL::Triangulation_data_structure_3<Vb> Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds> Delaunay_CGAL;
typedef K::Point_3 Point;
#endif

using namespace eco_sys_lab_plugin;

#ifdef USE_CGAL
inline glm::vec3 cgal_to_glm(const CGAL::Point_3<CGAL::Epick>& p) {
  return glm::vec3(p.x(), p.y(), p.z());
}
#endif

DynamicStrands::DynamicStrands() {
  if (!strands_layout) {
    strands_layout = std::make_shared<DescriptorSetLayout>();
    strands_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);

    strands_layout->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);

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
  device_connections_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  device_delaunay_tetrahedrons_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();
  strands_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : strands_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(strands_layout);
  }

  pre_step = std::make_shared<DynamicStrandsPreStep>();
  prediction = std::make_shared<DynamicStrandsPrediction>();
}

bool DynamicStrands::WaitForUpload() const {
  return wait_for_upload;
}

bool DynamicStrands::InitializeParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat("Wood Density", &wood_density, 0.01f, 0.01f, 3.0f))
    changed = true;
  if (ImGui::DragFloat("Shear stiffness", &shear_stiffness, 0.01f, 0.01f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Stretch stiffness", &stretch_stiffness, 0.01f, 0.01f, 1.0f))
    changed = true;

  if (ImGui::DragFloat("Bending stiffness", &bending_stiffness, 0.01f, 0.01f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Twisting stiffness", &twisting_stiffness, 0.01f, 0.01f, 1.0f))
    changed = true;

  if (ImGui::DragFloat("Velocity damping", &velocity_damping, 0.01f, 0.01f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Angular velocity damping", &angular_velocity_damping, 0.01f, 0.01f, 1.0f))
    changed = true;

  if (ImGui::DragFloat("Neighbor range", &neighbor_range, 0.01f, 0.01f, 1.0f))
    changed = true;

  if (ImGui::DragFloat3("Max neighbor strain", &neighbor_strain, 0.001f, 0.001f, 1.0f))
    changed = true;

  if (ImGui::DragFloat3("Max bend twist strain", &max_bend_twist_strain.x, 0.001f, 0.001f, 1.0f))
    changed = true;
  return changed;
}

void DynamicStrands::Initialize(const InitializeParameters& initialize_parameters,
                                const StrandModelSkeleton& strand_model_skeleton,
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

    const float mass =
        segment.radius * segment.radius * glm::pi<float>() * initialize_parameters.wood_density * segment.rest_length;
    segment.inertia_tensor = ComputeInertiaTensorRod(mass, segment.radius, segment.rest_length);
    segment.inv_inertia_tensor = 1.f / segment.inertia_tensor;
    segment.original_inv_mass = 1.f / mass;

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

  particles.resize(segments.size() * 2);
  Jobs::RunParallelFor(segments.size(), [&](const size_t segment_handle) {
    auto& segment = segments[segment_handle];
    const auto& strand_segment = strand_group.PeekStrandSegment(static_cast<int>(segment_handle));
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(static_cast<int>(segment_handle));
    auto& particle0 = particles[segment_handle * 2];
    auto& particle1 = particles[segment_handle * 2 + 1];
    segment.particle0_handle = static_cast<int>(segment_handle) * 2;
    segment.particle1_handle = static_cast<int>(segment_handle) * 2 + 1;
    particle0.damping = particle1.damping = initialize_parameters.velocity_damping;
    particle0.x0 = particle0.x = particle0.last_x = particle0.old_x =
        glm::vec4(initialize_parameters.root_transform.TransformPoint(
                      strand_group.GetStrandSegmentStart(static_cast<int>(segment_handle))),
                  0.0);

    particle1.x0 = particle1.x = particle1.last_x = particle1.old_x =
        glm::vec4(initialize_parameters.root_transform.TransformPoint(strand_segment.end_position), 0.0);

    particle0.acceleration = particle1.acceleration = glm::vec3(0.0);
    particle0.strand_handle = particle1.strand_handle = segment.strand_handle;
    particle0.node_handle = particle1.node_handle = strand_segment_data.node_handle;
    particle0.segment_handle = particle1.segment_handle = static_cast<int>(segment_handle);
  });

  for (uint32_t strand_index = 0; strand_index < target_strands.size(); strand_index++) {
    auto& target_strand = target_strands[strand_index];
    const auto& handles = target_strand.PeekStrandSegmentHandles();
    if (handles.size() < 2)
      continue;

    auto& strand = strands[strand_index];
    strand.begin_segment_handle = handles.front();
    strand.end_segment_handle = handles.back();
    const int handle_index_offset = static_cast<int>(connections.size());
    strand.begin_connection_handle = handle_index_offset;
    strand.end_connection_handle = handle_index_offset + static_cast<int>(handles.size()) - 2;

    connections.resize(connections.size() + handles.size() - 1);
    for (int handle_index = 0; handle_index < static_cast<int>(handles.size()) - 1; handle_index++) {
      const auto connection_handle = handle_index + handle_index_offset;
      auto& connection = connections[connection_handle];
      connection.segment0_handle = handles[handle_index];
      connection.segment1_handle = handles[handle_index + 1];
      const auto& segment0 = segments[connection.segment0_handle];
      const auto& segment1 = segments[connection.segment1_handle];
      connection.segment0_particle_handle = segment0.particle1_handle;
      connection.segment1_particle_handle = segment1.particle0_handle;

      if (handle_index > 0) {
        connection.prev_handle = connection_handle - 1;
      } else {
        connection.prev_handle = -1;
      }
      if (handle_index < static_cast<int>(handles.size()) - 2) {
        connection.next_handle = connection_handle + 1;
      } else {
        connection.next_handle = -1;
      }
      particles[connection.segment0_particle_handle].connection_handle = connection_handle;
      particles[connection.segment1_particle_handle].connection_handle = connection_handle;
      connection.bending_stiffness = initialize_parameters.bending_stiffness;
      connection.twisting_stiffness = initialize_parameters.twisting_stiffness;
      const auto& q0 = segment0.q0;
      const auto& q1 = segment1.q0;

      connection.rest_darboux_vector = glm::conjugate(q0) * q1;
      connection.bend_twist_strain_valid.w = 1.0;
      connection.max_bend_twist_strain = initialize_parameters.max_bend_twist_strain;
    }
  }
  ComputeDelaunay(particles, delaunay_tetrahedrons);

  Upload();
  for (const auto& i : constraints)
    i->InitializeData(initialize_parameters, strand_model_skeleton, *this);
}

bool DynamicStrands::PhysicsParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat("Time step", &time_step, 0.001f, 0.001f, 1.0f))
    changed = true;
  if (ImGui::DragInt("Sub step", &sub_step, 1, 1, 100)) {
    changed = true;
  }

  if (ImGui::DragInt("Constraint Iteration", &constraint_iteration, 1, 1, 500))
    changed = true;
  return changed;
}
void DynamicStrands::UpdateBindings() const {
  if (segments.empty())
    return;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();

  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(0, device_strands_buffer, 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(1, device_segments_buffer, 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(2, device_particles_buffer, 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(3, device_connections_buffer, 0);

  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(4, device_delaunay_tetrahedrons_buffer, 0);
}

void DynamicStrands::Upload() {
  wait_for_upload = true;
  Platform::AddTemporaryBufferSyncAction([&]() {
    device_strands_buffer->UploadVector(strands);
    device_segments_buffer->UploadVector(segments);
    device_particles_buffer->UploadVector(particles);
    device_connections_buffer->UploadVector(connections);

    device_delaunay_tetrahedrons_buffer->UploadVector(delaunay_tetrahedrons);

    for (const auto& c : constraints) {
      c->UploadData();
    }
    wait_for_upload = false;
  });
}

void DynamicStrands::Download() {
  Platform::AddTemporaryBufferSyncAction([&]() {
    device_strands_buffer->DownloadVector(strands, strands.size());
    device_segments_buffer->DownloadVector(segments, segments.size());
    device_particles_buffer->DownloadVector(particles, particles.size());
    device_connections_buffer->DownloadVector(connections, connections.size());

    device_delaunay_tetrahedrons_buffer->DownloadVector(delaunay_tetrahedrons, delaunay_tetrahedrons.size());
    for (const auto& c : constraints) {
      c->DownloadData();
    }
  });
}

void DynamicStrands::Clear() {
  strands.clear();
  segments.clear();
  particles.clear();
  connections.clear();
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

void DynamicStrands::ComputeDelaunay(const std::vector<GpuParticle>& particles,
                                     std::vector<GpuDelaunayTetrahedron>& tetrahedrons) {
  const auto point_plane_distance = [&](const glm::vec3& target_point, const glm::vec3& target_a,
                                        const glm::vec3& target_b, const glm::vec3& target_c) {
    // Compute the normal of the triangle
    const glm::vec3 ab = target_b - target_a;
    const glm::vec3 ac = target_c - target_a;
    const glm::vec3 normal = glm::normalize(glm::cross(ab, ac));

    // Compute signed distance from point to triangle's plane
    const float distance = glm::dot(normal, target_point - target_a);

    return distance;
  };

  /// @brief Given two arrays of size 4, each containing one element that is not in
  /// the other, compute the respective indices of these elements in the arrays
  /// @param a index array of size 4
  /// @param b index array of size 4
  /// @return pair of indices:
  /// first indicates the position in a that does not occur in b,
  /// second indicates the position in b that does not occur in a.
  const auto compare_indices = [&](const int a[4], const int b[4]) {
    std::vector a_in_both(4, false);
    std::vector b_in_both(4, false);

    for (size_t i = 0; i < 4; i++) {
      for (size_t j = 0; j < 4; j++) {
        if (a[i] == b[j]) {
          a_in_both[i] = true;
          b_in_both[j] = true;
        }
      }
    }

    int a_not_in_b = -1, b_not_in_a = -1;

    for (int i = 0; i < 4; i++) {
      if (!a_in_both[i]) {
        a_not_in_b = i;
      }
      if (!b_in_both[i]) {
        b_not_in_a = i;
      }
    }

    if (a_not_in_b >= 4 || b_not_in_a >= 4) {
      return std::make_pair(a_not_in_b, b_not_in_a);
      // throw std::exception("Did not find mismatched indices!");
    }

    return std::make_pair(a_not_in_b, b_not_in_a);
  };

  const auto is_valid = [&](const int target_indices[4], const std::vector<GpuParticle>& target_particles) {
    for (size_t i = 0; i < 4; i++) {
      if (static_cast<unsigned>(target_indices[i]) >= target_particles.size()) {
        EVOENGINE_ERROR("tetrahedron vertex index out of range, will be discarded: " << target_indices[i]);
        return false;
      }
    }

    // check if all indices are distinct
    for (size_t i = 0; i < 4; i++) {
      for (size_t j = i + 1; j < 4; j++) {
        if (target_indices[i] == target_indices[j]) {
          return false;
        }
      }
    }

    return true;
  };

// TODO: maybe a different library will work here
#ifdef USE_CGAL
  std::vector<std::pair<Point, unsigned> > points;

  for (size_t i = 0; i < particles.size(); i++) {
    Point p_cgal(particles[i].x0[0], particles[i].x0[1], particles[i].x0[2]);
    points.push_back(std::make_pair(p_cgal, i));
  }

  Delaunay_CGAL dt;
  dt.insert(points.begin(), points.end());

  // TODO: parallel for
  for (auto cell_it = dt.all_cells_begin(); cell_it != dt.all_cells_end(); cell_it++) {
    auto& cell = *cell_it;

    auto& tetrahedron = dt.tetrahedron(cell_it);

    GpuDelaunayTetrahedron gpu_tet;

    for (size_t i = 0; i < 4; i++) {
      gpu_tet.indices[i] = cell.vertex(i)->info();
      gpu_tet.neighbors[i] = -1;
    }

    if (!is_valid(gpu_tet.indices, particles)) {
      continue;  // discard this tetrahedron
    }

    // set up debugging members
    gpu_tet.color = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
    for (size_t i = 0; i < 4; i++) {
      gpu_tet.render_neighbor[i] = -1;
    }
    gpu_tet.task_looked_at = 0;
    gpu_tet.mesh_looked_at = 0;
    gpu_tet.inside = 0;
    gpu_tet.triangles_accepted = 0;

    // check orientation of the tetrahedron
    float d = point_plane_distance(particles[gpu_tet.indices[3]].x0, particles[gpu_tet.indices[0]].x0,
                                   particles[gpu_tet.indices[1]].x0, particles[gpu_tet.indices[2]].x0);

    if (d > 0)  // point no. 3 is in front if the triangle 0 1 2, we need to correct this so the triangles face outwards
    {
      std::swap(gpu_tet.indices[1], gpu_tet.indices[2]);
    }

    // fill in neighbor indices
    // TODO: need to figure out how to check if a neighbor is valid
    for (size_t i = 0; i < 4; i++) {
      auto& neighbor = *cell.neighbor(i);
      int neighbor_indices[4];

      for (size_t j = 0; j < 4; j++) {
        neighbor_indices[j] = neighbor.vertex(j)->info();
      }

      if (!is_valid(neighbor_indices, particles)) {
        continue;
      }

      const auto mismatch_indices = compare_indices(gpu_tet.indices, neighbor_indices);
      // TODO: according to CGAL documentation, this is guaranteed anyway, so we do not need to match both sides
      // store it such that the neighboring tetrahedron always consists of different indices
      // e. g. for the triangle 1 2 4 we store the corresponding neighboring index at position 3
      gpu_tet.neighbors[mismatch_indices.first] = neighbor_indices[mismatch_indices.second];
    }

    tetrahedrons.push_back(gpu_tet);
  }

#else
  EVOENGINE_ERROR("CGAL is required for delaunay!");
#endif
}
