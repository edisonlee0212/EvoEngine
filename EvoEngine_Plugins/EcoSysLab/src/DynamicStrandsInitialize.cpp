#include "DsConstraints.hpp"
#include "DynamicStrands.hpp"
#include "FoliageDescriptor.hpp"
#include "UVMapUtils.hpp"
#include "glm/gtc/matrix_access.hpp"
#include "glm/gtx/quaternion.hpp"

#include "Shader.hpp"

using namespace eco_sys_lab_plugin;

void DynamicStrands::Initialize(const InitializeParameters& initialize_parameters,
                                const StrandModelSkeleton& strand_model_skeleton,
                                const StrandModelStrandGroup& strand_model_strand_group,
                                const DtsStrandGroup& strand_group) {
  Clear();
  frame_index = 0;
  simulated_time = 0.f;
  assert(initialize_parameters.root_transform.GetScale() == glm::vec3(1.0f));
  const auto& target_strands = strand_group.PeekStrands();
  const auto& target_strand_segments = strand_group.PeekStrandSegments();
  const auto& target_strand_segment_data_list = strand_group.PeekStrandSegmentDataList();
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
  Jobs::RunParallelFor(target_strand_segments.size(), [&](const size_t segment_handle) {
    auto& segment = segments[segment_handle];
    const auto& target_strand_segment = target_strand_segments[segment_handle];
    const auto& target_strand_segment_data = target_strand_segment_data_list[segment_handle];
    segment.prev_handle = target_strand_segment.GetPrevHandle();
    segment.next_handle = target_strand_segment.GetNextHandle();
    segment.strand_handle = target_strand_segment.GetStrandHandle();
    segment.rest_length = glm::max(1e-6f, strand_group.GetStrandSegmentLength(static_cast<int>(segment_handle)));
    segment.color = target_strand_segment.end_color;

    segment.radius = glm::max(1e-6f, target_strand_segment.end_thickness * .5f);
    segment.q0 = segment.q = segment.last_q =
        initialize_parameters.root_transform.GetRotation() * target_strand_segment.rotation;
    segment.torque = glm::vec3(0.f);
    // 0.6046 = area radio of the circle within its bounding equilateral triangle.
    const float ratio = target_strand_segment_data.initial_distance_to_boundary * segment.radius * 2.f /
                        initialize_parameters.max_distance_to_boundary;

    segment.original_mass =
        glm::max(1e-6f, segment.radius * segment.radius * glm::pi<float>() *
                            initialize_parameters.wood_density.GetValue(ratio) * segment.rest_length);
    segment.extra_mass = 0.f;
    segment.property1 = segment.property2 = segment.property3 = 0.f;
    segment.inertia_tensor = ComputeInertiaTensorRod(segment.original_mass, segment.radius, segment.rest_length);
    segment.inv_inertia_tensor = 1.f / segment.inertia_tensor;
    const float area = glm::pi<float>() * segment.radius * segment.radius;
    segment.max_stretching_modulus = glm::max(1e-9f, initialize_parameters.max_youngs_modulus.GetValue(ratio)) * 1e9f;
    segment.max_shearing_modulus = glm::max(1e-9f, initialize_parameters.max_shear_modulus.GetValue(ratio)) * 1e9f;
    segment.moisture_content = glm::max(1e-9f, initialize_parameters.moisture_content.GetValue(ratio));
    segment.boundary_distance = target_strand_segment_data.initial_distance_to_boundary * segment.radius * 2.f;
    segment.profile_position = target_strand_segment_data.profile_position;
    segment.profile_polar_coordinate = target_strand_segment_data.profile_polar_coordinate;

    segment.stretching_alpha = 1.f / (segment.max_stretching_modulus * area / segment.rest_length);
    segment.shearing_alpha = 1.f / (segment.max_shearing_modulus * area / segment.rest_length);
    const float max_shear_strain = glm::max(0.001f, initialize_parameters.max_shear_strain.GetValue(ratio));
    const float max_stretch_strain = glm::max(0.001f, initialize_parameters.max_stretch_strain.GetValue(ratio));
    segment.shear_stretch_strain_limit = segment.max_shear_stretch_strain =
        glm::vec2(max_shear_strain, max_stretch_strain);

    const auto& strand_segment = strand_group.PeekStrandSegment(static_cast<int>(segment_handle));
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(static_cast<int>(segment_handle));
    auto& particle0 = segment.particle0;
    auto& particle1 = segment.particle1;
    segment.group_index = 0;
    particle0.x0 = particle0.x = particle0.last_x = glm::vec3(initialize_parameters.root_transform.TransformPoint(
        strand_group.GetStrandSegmentStart(static_cast<int>(segment_handle))));

    particle1.x0 = particle1.x = particle1.last_x =
        glm::vec3(initialize_parameters.root_transform.TransformPoint(strand_segment.end_position));

    particle0.acceleration = particle1.acceleration = glm::vec3(0.0);
    particle0.node_handle = particle1.node_handle = strand_segment_data.node_handle;
  });

  DtsStrandGroup uniformly_subdivided_strand_group;
  strand_model_strand_group.UniformlySubdivide<DtsStrandGroupData, DtsStrandData, DtsStrandSegmentData>(
      uniformly_subdivided_strand_group, initialize_parameters.uniform_subdivision,
      [&](const StrandHandle src_handle, DtsStrandData& strand_data) {

      },
      [&](const float start_root_distance, const float end_root_distance, const StrandSegmentHandle src_handle,
          const uint32_t original_segment_index, const float segment_t, DtsStrandSegmentData& segment_data,
          const uint32_t sub_segment_index) {
        const auto& src_segment_data = strand_model_strand_group.PeekStrandSegmentData(src_handle);
        segment_data.node_handle = src_segment_data.node_handle;
        segment_data.original_segment_handle = src_handle;
        segment_data.original_segment_index = original_segment_index;
        segment_data.segment_index = sub_segment_index;
        segment_data.original_segment_t = segment_t;
        segment_data.start_root_distance = start_root_distance;
        segment_data.end_root_distance = end_root_distance;
        const auto& strand_segment = strand_model_strand_group.PeekStrandSegment(src_handle);
        const auto& strand = strand_model_strand_group.PeekStrand(strand_segment.GetStrandHandle());
        const auto& strand_segment_handles = strand.PeekStrandSegmentHandles();

        glm::vec2 p0, p1, p3;
        const glm::vec2 p2 = src_segment_data.profile_position;
        float d0, d1, d3;
        const float d2 = src_segment_data.initial_distance_to_boundary;
        if (src_handle == strand_segment_handles.front()) {
          d1 = d2;
          d0 = d1 * 2.0f - d2;

          p1 = p2;
          p0 = p1 * 2.0f - p2;

        } else if (strand_segment.GetPrevHandle() == strand_segment_handles.front()) {
          const auto& prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetPrevHandle());
          d0 = d2;
          d1 = prev_segment_data.initial_distance_to_boundary;

          p0 = p2;
          p1 = prev_segment_data.profile_position;

        } else {
          const auto& prev_segment = strand_model_strand_group.PeekStrandSegment(strand_segment.GetPrevHandle());
          const auto& prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetPrevHandle());
          const auto& prev_prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(prev_segment.GetPrevHandle());
          d0 = prev_prev_segment_data.initial_distance_to_boundary;
          d1 = prev_segment_data.initial_distance_to_boundary;

          p0 = prev_prev_segment_data.profile_position;
          p1 = prev_segment_data.profile_position;
        }
        if (src_handle == strand_segment_handles.back()) {
          d3 = d2 * 2.0f - d1;

          p3 = p2 * 2.0f - p1;

        } else {
          const auto& next_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetNextHandle());
          d3 = next_segment_data.initial_distance_to_boundary;

          p3 = next_segment_data.profile_position;
        }
        segment_data.initial_distance_to_boundary = Strands::CubicInterpolation(d0, d1, d2, d3, segment_t);
        segment_data.profile_position = Strands::CubicInterpolation(p0, p1, p2, p3, segment_t);

        const auto calculate_polar_coordinates = [](const glm::vec2& profile_position) {
          const auto r = glm::length(profile_position);
          if (r <= glm::epsilon<float>()) {
            return glm::vec2(0.0f);
          }
          if (profile_position.y >= 0)
            return glm::vec2(r, glm::acos(profile_position.x / r));
          return glm::vec2(r, -glm::acos(profile_position.x / r));
        };

        segment_data.profile_polar_coordinate = calculate_polar_coordinates(segment_data.profile_position);
      },
      (initialize_parameters.min_segment_length + initialize_parameters.max_segment_length) * .5f * .01f);

  uniform_particles.resize(uniformly_subdivided_strand_group.PeekStrandSegments().size() + target_strands.size());
  std::vector<int> uniform_particle_offsets(target_strands.size());
  if (!uniform_particle_offsets.empty())
    uniform_particle_offsets[0] = 0;
  for (uint32_t strand_index = 1; strand_index < target_strands.size(); strand_index++) {
    uniform_particle_offsets[strand_index] =
        uniform_particle_offsets[strand_index - 1] +
        uniformly_subdivided_strand_group.PeekStrand(strand_index - 1).PeekStrandSegmentHandles().size() + 1;
  }
  Jobs::RunParallelFor(target_strands.size(), [&](const size_t strand_index) {
    auto& random_subdivided_strand = target_strands[strand_index];
    auto& uniformly_subdivided_strand = uniformly_subdivided_strand_group.PeekStrand(strand_index);
    const auto uniform_particle_offset = uniform_particle_offsets[strand_index];

    auto& first_uniform_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(
        uniformly_subdivided_strand.PeekStrandSegmentHandles()[0]);
    auto& first_uniform_particle = uniform_particles[uniform_particle_offset];
    int random_segment_walker_index = 0;
    first_uniform_particle.segment_handle =
        random_subdivided_strand.PeekStrandSegmentHandles()[random_segment_walker_index];
    first_uniform_particle.node_index = first_uniform_segment_data.node_handle;
    first_uniform_particle.t = 0.0f;
    first_uniform_particle.segment_index = 0;
    first_uniform_particle.prev_particle_handle = -1;
    first_uniform_particle.next_particle_handle = -1;
    first_uniform_particle.next_node_index = -1;
    first_uniform_particle.strand_index = strand_index;
    first_uniform_particle.is_single_strand_particle = 1;
    first_uniform_particle.local_extrusion_distance = 0.0f;
    first_uniform_particle.override_color = glm::vec4(0.f);
    // First 2 particles within same strand will always have same profile position/polar coordinate.
    first_uniform_particle.profile_position = first_uniform_segment_data.profile_position;
    first_uniform_particle.profile_polar_coordinate = first_uniform_segment_data.profile_polar_coordinate;

    int last_index_with_new_node = 0;
    float previous_root_distance = 0.0f;
    for (int uniform_segment_index = 0;
         uniform_segment_index < uniformly_subdivided_strand.PeekStrandSegmentHandles().size();
         uniform_segment_index++) {
      const auto& uniform_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(
          uniformly_subdivided_strand.PeekStrandSegmentHandles()[uniform_segment_index]);
      auto& uniform_particle = uniform_particles[uniform_particle_offset + 1 + uniform_segment_index];
      uniform_particle.node_index = uniform_segment_data.node_handle;
      uniform_particle.segment_index = uniform_segment_index + 1;
      uniform_particle.prev_particle_handle = uniform_particle_offset + uniform_segment_index;
      uniform_particles[uniform_particle_offset + uniform_segment_index].next_particle_handle =
          uniform_particle_offset + 1 + uniform_segment_index;
      uniform_particle.next_particle_handle = -1;  // will stay for the last particle of the strand
      uniform_particle.next_node_index = -1;       // will stay for the last particle of the strand
      uniform_particle.strand_index = strand_index;
      uniform_particle.is_single_strand_particle = 1;
      uniform_particle.local_extrusion_distance = 0.0f;
      uniform_particle.override_color = glm::vec4(0.f);
      uniform_particle.profile_position = uniform_segment_data.profile_position;
      uniform_particle.profile_polar_coordinate = uniform_segment_data.profile_polar_coordinate;

      if (uniform_particles[uniform_particle_offset + 1 + last_index_with_new_node].node_index !=
          uniform_particle.node_index) {
        // write node index to all previous ones
        for (int i = last_index_with_new_node; i < uniform_segment_index + 1; i++) {
          uniform_particles[uniform_particle_offset + i].next_node_index = uniform_particle.node_index;
        }
        last_index_with_new_node = uniform_segment_index;
      }

      bool found = false;
      while (random_segment_walker_index < random_subdivided_strand.PeekStrandSegmentHandles().size()) {
        uniform_particle.segment_handle =
            random_subdivided_strand.PeekStrandSegmentHandles()[random_segment_walker_index];
        const auto& random_segment_data = strand_group.PeekStrandSegmentData(uniform_particle.segment_handle);
        if (random_segment_data.end_root_distance >= uniform_segment_data.end_root_distance) {
          // Get the start original_segment_t for random_segment.
          if (glm::abs(random_segment_data.end_root_distance - previous_root_distance) < glm::epsilon<float>()) {
            uniform_particle.t = 1.f;
          } else {
            uniform_particle.t = (uniform_segment_data.end_root_distance - previous_root_distance) /
                                 (random_segment_data.end_root_distance - previous_root_distance);
          }
          uniform_particle.distance_to_boundary = uniform_segment_data.initial_distance_to_boundary;
          found = true;
          break;
        }
        random_segment_walker_index++;
        previous_root_distance = random_segment_data.end_root_distance;
      }
      if (!found) {
        EVOENGINE_ERROR("Fault!");
      }
    }
  });

  Jobs::RunParallelFor(uniform_particles.size(), [&](const size_t uniform_particle_index) {
    auto& uniform_particle = uniform_particles[uniform_particle_index];
    const auto& segment = segments[uniform_particle.segment_handle];
    const auto& particle0 = segment.particle0;
    const auto& particle1 = segment.particle1;

    uniform_particle.position = glm::mix(particle0.x0, particle1.x0, uniform_particle.t);
    uniform_particle.normal = glm::vec3(0.0f);
    uniform_particle.deg = 0.0f;
  });

  segment_data_list.resize(segments.size());
  std::vector<glm::vec3> projected_max_bounds(Jobs::GetWorkerSize());
  std::vector<glm::vec3> projected_min_bounds(Jobs::GetWorkerSize());
  std::vector<glm::vec3> max_bounds(Jobs::GetWorkerSize());
  std::vector<glm::vec3> min_bounds(Jobs::GetWorkerSize());
  for (auto& i : projected_max_bounds)
    i = glm::vec3(-FLT_MAX);
  for (auto& i : projected_min_bounds)
    i = glm::vec3(FLT_MAX);

  for (auto& i : max_bounds)
    i = glm::vec3(-FLT_MAX);
  for (auto& i : min_bounds)
    i = glm::vec3(FLT_MAX);

  float average_segment_length =
      (initialize_parameters.min_segment_length + initialize_parameters.max_segment_length) * 0.5f;

  const auto calculate_regularized_segment_p0 = [&](const int segment_handle) {
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(segment_handle);
    return glm::vec3(strand_segment_data.profile_position.x, strand_segment_data.profile_position.y,
                     strand_segment_data.start_root_distance / average_segment_length);
  };
  const auto calculate_regularized_segment_center = [&](const int segment_handle) {
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(segment_handle);
    return glm::vec3(strand_segment_data.profile_position.x, strand_segment_data.profile_position.y,
                     (strand_segment_data.start_root_distance + strand_segment_data.end_root_distance) * .5f /
                         average_segment_length);
  };
  const auto calculate_regularized_segment_p1 = [&](const int segment_handle) {
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(segment_handle);
    return glm::vec3(strand_segment_data.profile_position.x, strand_segment_data.profile_position.y,
                     strand_segment_data.end_root_distance / average_segment_length);
  };

  Jobs::RunParallelFor(segment_data_list.size(), [&](const auto segment_handle, const auto worker_i) {
    projected_max_bounds[worker_i] =
        glm::max(projected_max_bounds[worker_i], calculate_regularized_segment_p0(static_cast<int>(segment_handle)));
    projected_min_bounds[worker_i] =
        glm::min(projected_min_bounds[worker_i], calculate_regularized_segment_p0(static_cast<int>(segment_handle)));
    projected_max_bounds[worker_i] =
        glm::max(projected_max_bounds[worker_i], calculate_regularized_segment_p1(static_cast<int>(segment_handle)));
    projected_min_bounds[worker_i] =
        glm::min(projected_min_bounds[worker_i], calculate_regularized_segment_p1(static_cast<int>(segment_handle)));

    const auto pos = segments[segment_handle].GetCenterX0();

    max_bounds[worker_i] = glm::max(max_bounds[worker_i], pos);
    min_bounds[worker_i] = glm::min(min_bounds[worker_i], pos);
    max_bounds[worker_i] = glm::max(max_bounds[worker_i], pos);
    min_bounds[worker_i] = glm::min(min_bounds[worker_i], pos);

    for (int j = 0; j < BUNDLE_MAX_CONNECTION; j++) {
      segment_data_list[segment_handle].pair_handles[j] = -1;
    }
  });

  for (uint32_t strand_index = 0; strand_index < target_strands.size(); strand_index++) {
    auto& target_strand = target_strands[strand_index];
    const auto& segment_handles = target_strand.PeekStrandSegmentHandles();
    if (segment_handles.size() < 2)
      continue;

    auto& strand = strands[strand_index];
    strand.begin_segment_handle = segment_handles.front();
    strand.end_segment_handle = segment_handles.back();
    const int handle_index_offset = static_cast<int>(segment_pairs.size());
    strand.begin_segment_pair_handle = handle_index_offset;
    strand.end_segment_pair_handle = handle_index_offset + static_cast<int>(segment_handles.size()) - 2;
    segment_pairs.resize(segment_pairs.size() + segment_handles.size() - 1);
    for (int segment_handle_index = 0; segment_handle_index < static_cast<int>(segment_handles.size());
         segment_handle_index++) {
      const auto segment0_handle = segment_handles[segment_handle_index];
      auto& segment0 = segments[segment0_handle];
      if (segment_handle_index == static_cast<int>(segment_handles.size()) - 1)
        break;
      const auto segment_pair_handle = segment_handle_index + handle_index_offset;
      auto& segment_pair = segment_pairs[segment_pair_handle];

      segment_pair.segment0_handle = segment0_handle;
      segment_pair.segment1_handle = segment_handles[segment_handle_index + 1];

      const auto& segment1 = segments[segment_pair.segment1_handle];

      auto& segment0_data = segment_data_list[segment0_handle];
      auto& segment1_data = segment_data_list[segment_pair.segment1_handle];

      segment0_data.pair_handles[1] = segment_pair_handle;
      segment1_data.pair_handles[0] = segment_pair_handle;

      // particles[segment0.particle1_handle].connection_handle = segment_pair_handle;
      // particles[segment1.particle0_handle].connection_handle = segment_pair_handle;
    }
  }

  for (uint32_t strand_index = 0; strand_index < target_strands.size(); strand_index++) {
    auto& gpu_strand = strands[strand_index];
    gpu_strand.front_propagate_begin_segment_handle = -1;
    gpu_strand.back_propagate_begin_segment_handle = -1;
    gpu_strand.front_propagate_begin_segment_pair_handle = -1;
    gpu_strand.back_propagate_begin_segment_pair_handle = -1;
    gpu_strand.alternative_front_propagate_begin_segment_pair_handle = -1;
    gpu_strand.alternative_back_propagate_begin_segment_pair_handle = -1;
    gpu_strand.alternative_front_propagate_begin_segment_handle = -1;
    gpu_strand.alternative_back_propagate_begin_segment_handle = -1;
    if (gpu_strand.begin_segment_handle == -1) {
      continue;
    }
    gpu_strand.front_propagate_begin_segment_handle = gpu_strand.begin_segment_handle;
    if (gpu_strand.begin_segment_handle == gpu_strand.end_segment_handle) {
      gpu_strand.alternative_front_propagate_begin_segment_handle = gpu_strand.begin_segment_handle;
      continue;
    }
    gpu_strand.alternative_front_propagate_begin_segment_handle = segments[gpu_strand.begin_segment_handle].next_handle;

    gpu_strand.front_propagate_begin_segment_pair_handle = gpu_strand.begin_segment_pair_handle;
    if (gpu_strand.begin_segment_pair_handle == gpu_strand.end_segment_pair_handle) {
      gpu_strand.alternative_front_propagate_begin_segment_pair_handle = gpu_strand.begin_segment_pair_handle;
      continue;
    }
    gpu_strand.alternative_front_propagate_begin_segment_pair_handle = gpu_strand.begin_segment_pair_handle + 1;

    const int connection_size = gpu_strand.end_segment_pair_handle - gpu_strand.begin_segment_pair_handle + 1;
    gpu_strand.back_propagate_begin_segment_pair_handle =
        connection_size % 2 == 0 ? gpu_strand.end_segment_pair_handle : gpu_strand.end_segment_pair_handle - 1;

    gpu_strand.alternative_back_propagate_begin_segment_pair_handle =
        connection_size % 2 == 0 ? gpu_strand.end_segment_pair_handle - 1 : gpu_strand.end_segment_pair_handle;

    gpu_strand.back_propagate_begin_segment_handle =
        connection_size % 2 == 1 ? gpu_strand.end_segment_handle : segments[gpu_strand.end_segment_handle].prev_handle;

    gpu_strand.alternative_back_propagate_begin_segment_handle =
        connection_size % 2 == 1 ? segments[gpu_strand.end_segment_handle].prev_handle : gpu_strand.end_segment_handle;
  }
  connection_segment_pair_size = segment_pairs.size();

  auto projected_max_bound = glm::vec3(-FLT_MAX);
  auto projected_min_bound = glm::vec3(FLT_MAX);
  for (auto& i : projected_max_bounds)
    projected_max_bound = glm::max(i, projected_max_bound);
  for (auto& i : projected_min_bounds)
    projected_min_bound = glm::min(i, projected_min_bound);

  auto max_bound = glm::vec3(-FLT_MAX);
  auto min_bound = glm::vec3(FLT_MAX);
  for (auto& i : max_bounds)
    max_bound = glm::max(i, max_bound);
  for (auto& i : min_bounds)
    min_bound = glm::min(i, min_bound);

  struct SegmentInfo {
    glm::vec3 p0;
    glm::vec3 p1;
    glm::vec3 center_position;
    int node_handle;
    int strand_handle;
    int segment_handle;
  };

  VoxelGrid<std::vector<SegmentInfo>> projected_voxel_grid;
  constexpr auto projected_cell_size = 1.f;
  projected_voxel_grid.Initialize(projected_cell_size, projected_min_bound - glm::vec3(projected_cell_size) * 2.f,
                                  projected_max_bound + glm::vec3(projected_cell_size) * 2.f, {});

  for (int segment_handle = 0; segment_handle < segments.size(); segment_handle++) {
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(segment_handle);
    SegmentInfo s_d;
    s_d.p0 = calculate_regularized_segment_p0(segment_handle);
    s_d.p1 = calculate_regularized_segment_p1(segment_handle);
    s_d.center_position = calculate_regularized_segment_center(segment_handle);
    s_d.node_handle = strand_segment_data.node_handle;
    s_d.strand_handle = strand_group.PeekStrandSegment(segment_handle).GetStrandHandle();
    s_d.segment_handle = segment_handle;
    projected_voxel_grid.Ref(s_d.center_position).emplace_back(s_d);
  }
  std::multimap<float, std::map<std::pair<int, int>, std::pair<float, float>>> candidates;
  for (int segment_handle = 0; segment_handle < segments.size(); segment_handle++) {
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(segment_handle);
    const auto p0 = calculate_regularized_segment_p0(segment_handle);
    const auto p1 = calculate_regularized_segment_p1(segment_handle);

    const auto extended_p0 = p0 - glm::vec3(0, 0, initialize_parameters.neighbor_vertical_range);
    const auto extended_p1 = p1 + glm::vec3(0, 0, initialize_parameters.neighbor_vertical_range);

    const auto center = calculate_regularized_segment_center(segment_handle);
    const auto strand_handle = strand_group.PeekStrandSegment(segment_handle).GetStrandHandle();
    projected_voxel_grid.ForEach(
        center,
        glm::max(initialize_parameters.neighbor_vertical_range, initialize_parameters.neighbor_horizontal_range),
        [&](const std::vector<SegmentInfo>& list) {
          for (const auto& info : list) {
            if (info.segment_handle == segment_handle)
              continue;
            if (info.strand_handle == strand_handle) {
              continue;
            }
            //  Function to check if a point is inside a cylinder
            const auto cylinder_check = [](const glm::vec3& p0, const glm::vec3& p1, const float radius,
                                           const glm::vec3& point, float& horizontal_distance,
                                           float& vertical_distance) {
              // Calculate the direction vector of the cylinder's axis
              const glm::vec3 d_v = p1 - p0;
              const float height = glm::length(d_v);
              const glm::vec3 direction = glm::normalize(d_v);

              // Vector from p0 to point
              const glm::vec3 p0_p = point - p0;

              // Projection scalar
              const float t = glm::dot(p0_p, direction);
              // Check if projection is within the cylinder's height
              if (t < 0.0f || t > height) {
                return false;  // Outside the cylinder height
              }

              // Closest point on the cylinder's axis
              const glm::vec3 closest_point = p0 + t * direction;

              // Distance from point to the axis
              horizontal_distance = glm::length(point - closest_point);
              // Check if the distance is within the radius
              return horizontal_distance <= radius;
            };
            float horizontal_distance1, horizontal_distance2;
            float vertical_distance1, vertical_distance2;
            const auto check1 =
                cylinder_check(extended_p0, extended_p1, initialize_parameters.neighbor_horizontal_range, info.p0,
                               horizontal_distance1, vertical_distance1);
            const auto check2 =
                cylinder_check(extended_p0, extended_p1, initialize_parameters.neighbor_horizontal_range, info.p1,
                               horizontal_distance2, vertical_distance2);
            if (!check1 && !check2)
              continue;

            bool node_check = false;
            if (info.node_handle == strand_segment_data.node_handle)
              node_check = true;
            if (!node_check) {
              if (auto& node = strand_model_skeleton.PeekNode(strand_segment_data.node_handle);
                  info.node_handle == node.GetParentHandle()) {
                node_check = true;
              } else {
                for (const auto& child_handle : node.PeekChildHandles()) {
                  if (info.node_handle == child_handle) {
                    node_check = true;
                    break;
                  }
                }
              }
            }
            if (!node_check)
              continue;
            const auto horizontal_distance = glm::min(horizontal_distance1, horizontal_distance2);
            const auto pair = segment_handle <= info.segment_handle
                                  ? std::make_pair(segment_handle, info.segment_handle)
                                  : std::make_pair(info.segment_handle, segment_handle);

            const auto distance_pair = std::make_pair(horizontal_distance, 0.f);
            if (const auto search = candidates.find(horizontal_distance); search != candidates.end()) {
              search->second.emplace(pair, distance_pair);
            } else {
              candidates.insert({horizontal_distance, {}});
              candidates.find(horizontal_distance)->second.insert({pair, distance_pair});
            }
          }
        });
  }

  std::vector<uint32_t> counters(segments.size(), 2);
  for (const auto& candidate_set : candidates) {
    for (const auto& candidate : candidate_set.second) {
      auto& first = counters[candidate.first.first];
      auto& second = counters[candidate.first.second];
      if (first >= BUNDLE_MAX_CONNECTION || second >= BUNDLE_MAX_CONNECTION)
        continue;
      const auto pair_handle = static_cast<int>(segment_pairs.size());
      segment_pairs.emplace_back();
      auto& new_pair = segment_pairs.back();
      new_pair.segment0_handle = candidate.first.first;
      new_pair.segment1_handle = candidate.first.second;
      segment_data_list[candidate.first.first].pair_handles[first] = pair_handle;
      segment_data_list[candidate.first.second].pair_handles[second] = pair_handle;
      first++;
      second++;
    }
  }

  Jobs::RunParallelFor(segment_pairs.size(), [&](const auto pair_index) {
    auto& segment_pair = segment_pairs[pair_index];
    auto& segment0 = segments[segment_pair.segment0_handle];
    auto& segment1 = segments[segment_pair.segment1_handle];
    const bool direct_connection = segment_data_list[segment_pair.segment0_handle].pair_handles[1] == pair_index;
    auto& segment0_particle0 = segment0.particle0;
    auto& segment0_particle1 = segment0.particle1;
    auto& segment1_particle0 = segment1.particle0;
    auto& segment1_particle1 = segment1.particle1;
    const auto segment0_center_position = (segment0_particle0.x0 + segment0_particle1.x0) * .5f;
    const auto segment1_center_position = (segment1_particle0.x0 + segment1_particle1.x0) * .5f;
    segment_pair.segment0_offset =
        glm::vec4(glm::inverse(segment1.q0) * (segment0_center_position - segment1_center_position), 0.0f);
    segment_pair.segment1_offset =
        glm::vec4(glm::inverse(segment0.q0) * (segment1_center_position - segment0_center_position), 0.0f);
    segment_pair.rest_darboux_vector = glm::conjugate(segment0.q0) * segment1.q0;
    segment_pair.bend_twist_bundle_integrity = 1.0f;
    segment_pair.connectivity_integrity = direct_connection ? 1.0f : 0.0f;
    const float ratio0 = segment0.boundary_distance / initialize_parameters.max_distance_to_boundary;
    const float ratio1 = segment1.boundary_distance / initialize_parameters.max_distance_to_boundary;
    const float ratio = (ratio0 + ratio1) * .5f;
    segment_pair.max_bending_modulus =
        glm::max(1e-9f, initialize_parameters.max_bending_modulus.GetValue(ratio)) * 1e9f;
    segment_pair.max_torsion_modulus =
        glm::max(1e-9f, initialize_parameters.max_torsion_modulus.GetValue(ratio)) * 1e9f;
    const float average_segment_radius = (segment0.radius + segment1.radius) * .5f;
    const float average_segment_length = (segment0.rest_length + segment1.rest_length) * .5f;
    const auto second_moment_of_area = glm::pi<float>() * std::pow(average_segment_radius, 4.f) * 0.25f;
    const auto polar_moment_of_inertia = glm::pi<float>() * std::pow(average_segment_radius, 4.f) * 0.5f;
    segment_pair.bending_alpha =
        1.f / (segment_pair.max_bending_modulus * second_moment_of_area / glm::pow(average_segment_length, 3.f));
    segment_pair.torsion_alpha =
        1.f / (segment_pair.max_torsion_modulus * polar_moment_of_inertia / average_segment_length);
    const auto& q0 = segment0.q0;
    const auto& q1 = segment1.q0;
    segment_pair.rest_darboux_vector = glm::conjugate(q0) * q1;
    const float max_bend_strain = glm::max(0.001f, initialize_parameters.max_bend_strain.GetValue(ratio));
    const float max_twist_strain = glm::max(0.001f, initialize_parameters.max_twist_strain.GetValue(ratio));
    const float max_bundle_strain = glm::max(0.001f, initialize_parameters.max_bundle_strain.GetValue(ratio));
    segment_pair.max_bending_twist_bundle_strain = segment_pair.bending_twist_bundle_limit =
        glm::vec3(max_bend_strain, max_twist_strain, max_bundle_strain);
  });
  // set up nodes
  auto& skeleton_nodes = strand_model_skeleton.PeekRawNodes();
  nodes.resize(skeleton_nodes.size());

  for (size_t i = 0; i < skeleton_nodes.size(); i++) {
    nodes[i].prev_handle = skeleton_nodes[i].GetParentHandle();
  }

  if (!initialize_parameters.triangulate_per_bundle) {
    ComputeDelaunay(delaunay_tetrahedrons, initialize_parameters.use_cgal);
  } else {
    ComputeDelaunayPerBundle(delaunay_tetrahedrons, initialize_parameters.use_cgal);
  }
  for (const auto& i : constraints)
    i->InitializeData(initialize_parameters, strand_model_skeleton, strand_group, *this);

  hashed_grid_elements.resize(segments.size());
  hashed_grid_cell_starts.resize(HASH_GRID_CELL_SIZE);

  // Create foliage here.
  auto initialize_parameters_copy = initialize_parameters;
  auto fd = initialize_parameters_copy.foliage_descriptor.Get<FoliageDescriptor>();
  if (!fd)
    fd = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
  const auto& node_list = strand_model_skeleton.PeekSortedNodeList();
  const auto tree_dim = strand_model_skeleton.max - strand_model_skeleton.min;

  VoxelGrid<std::vector<SegmentInfo>> voxel_grid;
  const auto current_leaf_size = fd->leaf_size * glm::length(tree_dim) * 0.1f;
  const auto cell_size = 2.f * (current_leaf_size.y + fd->position_variance * glm::length(tree_dim) * 0.1f) +
                         initialize_parameters.max_segment_length;
  voxel_grid.Initialize(cell_size, min_bound - glm::vec3(cell_size) * 2.f, max_bound + glm::vec3(cell_size) * 2.f, {});
  std::unordered_set<int> enabled_node_handles;
  for (int segment_handle = 0; segment_handle < segments.size(); segment_handle++) {
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(segment_handle);
    SegmentInfo s_d;
    const auto& segment = segments[segment_handle];
    s_d.p0 = segment.particle0.x0;
    s_d.p1 = segment.particle1.x0;
    s_d.center_position = (s_d.p0 + s_d.p1) * 0.5f;
    s_d.node_handle = strand_segment_data.node_handle;
    enabled_node_handles.emplace(s_d.node_handle);
    s_d.strand_handle = strand_group.PeekStrandSegment(segment_handle).GetStrandHandle();
    s_d.segment_handle = segment_handle;
    voxel_grid.Ref(s_d.center_position).emplace_back(s_d);
  }

  struct LeafInfo {
    Transform matrix;
    int node_handle;
  };
  std::vector<LeafInfo> leaf_infos;
  for (const auto& internode_handle : node_list) {
    if (enabled_node_handles.find(internode_handle) == enabled_node_handles.end())
      continue;
    const auto& strand_model_node = strand_model_skeleton.PeekNode(internode_handle);
    std::vector<glm::mat4> leaf_matrices;
    fd->GenerateFoliageMatrices(leaf_matrices, strand_model_node.info, glm::length(tree_dim));
    for (const auto& matrix : leaf_matrices) {
      auto& leaf_info = leaf_infos.emplace_back();
      leaf_info.node_handle = internode_handle;
      leaf_info.matrix.value = initialize_parameters.root_transform.value * matrix;
    }
  }
  foliage.resize(leaf_infos.size());
  Jobs::RunParallelFor(foliage.size(), [&](const auto foliage_index) {
    const auto& leaf_info = leaf_infos[foliage_index];
    auto& leaf = foliage[foliage_index];
    bool found = false;
    glm::vec3 center = leaf_info.matrix.GetPosition();
    int target_segment_handle = 0;
    float distance = FLT_MAX;

    float min_radius = 0.f;
    float max_radius = cell_size;
    while (!found) {
      voxel_grid.ForEach(center, min_radius, max_radius, [&](const std::vector<SegmentInfo>& list) {
        for (const auto& i : list) {
          if (i.node_handle == leaf_info.node_handle) {
            if (const auto new_distance = glm::distance(center, i.center_position); new_distance < distance) {
              found = true;
              distance = new_distance;
              target_segment_handle = i.segment_handle;
            }
          }
        }
      });
      min_radius = max_radius;
      max_radius += cell_size;
    }
    leaf.segment_handle = target_segment_handle;
    leaf.attachment_integrity = 1.f;
    leaf.q0 = leaf.q = leaf.last_q = leaf_info.matrix.GetRotation();
    leaf.x0 = leaf.x = leaf.last_x = leaf_info.matrix.GetPosition();
    leaf.rotation_integrity = 1.f;
    leaf.scale = leaf_info.matrix.GetScale();
    leaf.original_mass = 0.0001f;
    leaf.extra_mass = 0.0f;
    leaf.inv_mass = 1.f / leaf.original_mass;  // 0.1g
    leaf.property1 = leaf.property2 = leaf.property3 = 0.f;
    leaf.inertia_tensor = ComputeInertiaTensorBox(1.f, leaf.scale.x, leaf.scale.y, leaf.scale.z);
    leaf.inv_inertia_tensor = 1.f / leaf.inertia_tensor;

    leaf.position_alpha = glm::max(1e-6f, initialize_parameters.leaf_position_alpha.GetValue());
    leaf.rotation_alpha = glm::max(1e-6f, initialize_parameters.leaf_rotation_alpha.GetValue());
    leaf.position_strain_limit = glm::max(1e-6f, initialize_parameters.max_leaf_position_strain.GetValue());
    leaf.rotation_strain_limit = glm::max(1e-6f, initialize_parameters.max_leaf_rotation_strain.GetValue());

    const auto& segment = segments[target_segment_handle];
    leaf.position_offset = glm::vec4(glm::inverse(segment.q0) * (leaf.x0 - segment.GetCenterX0()), 0.0f);
  });
  Upload();
  // Feel free to modify the push constants.
  struct BarkFlagInitializationPushConstant {
    uint32_t delaunay_tetrahedron_size;
  };

  static std::shared_ptr<ComputePipeline> bark_flag_initialization_pipeline;
  if (!bark_flag_initialization_pipeline) {
    std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Initialization/BarkFlag.comp");
    bark_flag_initialization_pipeline = std::make_shared<ComputePipeline>();
    bark_flag_initialization_pipeline->compute_shader = shader;
    bark_flag_initialization_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    auto& push_constant_range = bark_flag_initialization_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(BarkFlagInitializationPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bark_flag_initialization_pipeline->Initialize();
  }
  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;

  // Update push constant here. You should only access data within dynamic strands.
  BarkFlagInitializationPushConstant push_constant;
  push_constant.delaunay_tetrahedron_size = delaunay_tetrahedrons.size();

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto group_size = Platform::DivUp(delaunay_tetrahedrons.size(), work_group_invocations);
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    bark_flag_initialization_pipeline->Bind(vk_command_buffer);
    bark_flag_initialization_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    bark_flag_initialization_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    vkCmdDispatch(vk_command_buffer, group_size, 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}