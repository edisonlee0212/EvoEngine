#include "BasicFoliageDescriptor.hpp"
#include "DsConstraints.hpp"
#include "DynamicStrandUtils.hpp"
#include "DynamicStrands.hpp"
#include "UVMapUtils.hpp"
#include "glm/gtc/matrix_access.hpp"
#include "glm/gtx/quaternion.hpp"

#include "Shader.hpp"

using namespace eco_sys_lab_plugin;

void DynamicStrands::InitializeMesh(const DynamicStrandsInitializeParameters& initialize_parameters) {
  // same as RenderPushConstant, might change this later
  struct BarkFlagInitializationPushConstant {
    uint32_t tetrahedrons_size = 0;
    float alpha = 0.0f;
    float bifurcation_alpha = 0.0f;
    float max_dist_squared = 0.0f;
  };

  struct UniformParticleInitializationPushConstant {
    uint32_t uniform_particle_size = 0;
  };

  // Process tetrahedrons in two shader passes
  static std::shared_ptr<ComputePipeline> interior_initialization_pipeline;
  if (!interior_initialization_pipeline) {
    std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Initialization/Interior.comp");
    interior_initialization_pipeline = std::make_shared<ComputePipeline>();
    interior_initialization_pipeline->compute_shader = shader;
    interior_initialization_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    auto& push_constant_range = interior_initialization_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(BarkFlagInitializationPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    interior_initialization_pipeline->Initialize();
  }

  static std::shared_ptr<ComputePipeline> bark_flag_initialization_pipeline;
  if (!bark_flag_initialization_pipeline) {
    std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
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

  // Process uniform particles
  static std::shared_ptr<ComputePipeline> uniform_particle_initialization_pipeline;
  if (!uniform_particle_initialization_pipeline) {
    std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Initialization/Normal.comp");
    uniform_particle_initialization_pipeline = std::make_shared<ComputePipeline>();
    uniform_particle_initialization_pipeline->compute_shader = shader;
    uniform_particle_initialization_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    auto& push_constant_range = uniform_particle_initialization_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(UniformParticleInitializationPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    uniform_particle_initialization_pipeline->Initialize();
  }

  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;

  // Update push constant here. You should only access data within dynamic strands.
  BarkFlagInitializationPushConstant push_constant;
  push_constant.tetrahedrons_size = delaunay_tetrahedrons.size();
  push_constant.alpha = initialize_parameters.alpha;
  push_constant.bifurcation_alpha = initialize_parameters.bifurcation_alpha;
  push_constant.max_dist_squared = initialize_parameters.max_dist_squared;

  UniformParticleInitializationPushConstant uniform_particle_push_constant;
  uniform_particle_push_constant.uniform_particle_size = uniform_particles.size();

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto delaunay_tetrahedrons_group_size = Platform::DivUp(delaunay_tetrahedrons.size(), work_group_invocations);
  const auto uniform_particles_group_size = Platform::DivUp(uniform_particles.size(), work_group_invocations);

  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    interior_initialization_pipeline->Bind(vk_command_buffer);
    interior_initialization_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    interior_initialization_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    vkCmdDispatch(vk_command_buffer, delaunay_tetrahedrons_group_size, 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });

  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    bark_flag_initialization_pipeline->Bind(vk_command_buffer);
    bark_flag_initialization_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    bark_flag_initialization_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    vkCmdDispatch(vk_command_buffer, delaunay_tetrahedrons_group_size, 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });

  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    uniform_particle_initialization_pipeline->Bind(vk_command_buffer);
    uniform_particle_initialization_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    uniform_particle_initialization_pipeline->PushConstant(vk_command_buffer, 0, uniform_particle_push_constant);
    vkCmdDispatch(vk_command_buffer, uniform_particles_group_size, 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void DynamicStrands::InitializeData(std::mt19937& random_engine,
                                    const DynamicStrandsInitializeParameters& initialize_parameters,
                                    const StrandModelSkeleton& strand_model_skeleton,
                                    const StrandModelStrandGroup& strand_model_strand_group,
                                    DtsStrandGroup& randomly_subdivided_strand_group,
                                    DtsStrandGroup& uniformly_subdivided_strand_group) {
  Clear();

  constraints.emplace_back(std::make_shared<DsStiffRod>());
  constraints.emplace_back(std::make_shared<DsBundle>());
  constraints.emplace_back(std::make_shared<DsLeafAttachment>());

  strand_model_strand_group.Subdivide<DtsStrandGroupData, DtsStrandData, DtsStrandSegmentData>(
      randomly_subdivided_strand_group,
      [&]() {
        return Random::Uniform(random_engine, initialize_parameters.min_segment_length,
                               initialize_parameters.max_segment_length);
      },
      [](StrandHandle src_handle, DtsStrandData& strand_data) {
      },
      [&](const float start_root_distance, const float end_root_distance, const StrandSegmentHandle src_handle,
          const uint32_t original_segment_index, const float segment_t, DtsStrandSegmentData& segment_data,
          const uint32_t sub_segment_index) {
        const auto& src_segment_data = strand_model_strand_group.PeekStrandSegmentData(src_handle);
        segment_data.node_handle = src_segment_data.node_handle;
        segment_data.original_segment_t = segment_t;
        segment_data.original_segment_handle = src_handle;
        segment_data.original_segment_index = original_segment_index;
        segment_data.segment_index = sub_segment_index;
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

  randomly_subdivided_strand_group.RandomAssignColor();

  frame_index = 0;
  simulated_time = 0.f;
  const auto& target_strands = randomly_subdivided_strand_group.PeekStrands();
  const auto& target_strand_segments = randomly_subdivided_strand_group.PeekStrandSegments();
  const auto& target_strand_segment_data_list = randomly_subdivided_strand_group.PeekStrandSegmentDataList();
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
    segment.node_handle = target_strand_segment_data.node_handle;
    segment.rest_length =
        glm::max(1e-6f, randomly_subdivided_strand_group.GetStrandSegmentLength(static_cast<int>(segment_handle)));
    segment.color = target_strand_segment.end_color;

    segment.radius = glm::max(1e-6f, target_strand_segment.end_thickness * .5f);
    segment.q0 = segment.q = segment.last_q =
        initialize_parameters.root_transform.GetRotation() * target_strand_segment.rotation;
    segment.torque = glm::vec3(0.f);
    // 0.6046 = area radio of the circle within its bounding equilateral triangle.
    const float distance_to_boundary = target_strand_segment_data.initial_distance_to_boundary * segment.radius * 2.f;
    const float root_distance =
        (target_strand_segment_data.start_root_distance + target_strand_segment_data.end_root_distance) * .5f;
    BiologicalPropertiesGraph::Input biological_properties_input;
    biological_properties_input.root_distance = root_distance;
    biological_properties_input.polar_distance = segment.profile_polar_coordinate.x;
    biological_properties_input.polar_angle = segment.profile_polar_coordinate.y;
    biological_properties_input.profile_boundary_distance = distance_to_boundary;

    BiologicalPropertiesGraph::Output biological_properties =
        initialize_parameters.biological_properties_graph.GetValues(biological_properties_input);
    const float trunk_strength_factor =
        initialize_parameters.trunk_additional_strength
            ? ActivationFunction::Sigmoid(biological_properties.trunk_additional_strength_factor, 0.f,
                                          biological_properties.trunk_offset,
                                          1.f / biological_properties.trunk_transition, root_distance)
            : 0.f;
    ModulusGraph::Input modulus_input;
    modulus_input.root_distance = root_distance;
    modulus_input.polar_distance = segment.profile_polar_coordinate.x;
    modulus_input.polar_angle = segment.profile_polar_coordinate.y;
    modulus_input.profile_boundary_distance = distance_to_boundary;

    ModulusGraph::Output::DensityType density = initialize_parameters.modulus_graph.GetDensity(modulus_input);
    segment.original_mass = glm::max(
        1e-6f, segment.radius * segment.radius * glm::pi<float>() *
                   ActivationFunction::Sigmoid(density.x, density.y, initialize_parameters.sapwood_offset,
                                               1.f / initialize_parameters.wood_transition, distance_to_boundary) *
                   segment.rest_length);
    segment.extra_mass = 0.f;
    segment.snow_amount = 0.f;

    segment.C = 0.2f;
    segment.HC = 1.0f;
    segment.HL = 1.0f;
    segment.RW = 0.0f;
    segment.RB = 0.0f;
    segment.C_pre = 0.2f;
    segment.HC_pre = 1.0f;
    segment.HL_pre = 1.0f;
    segment.RW_pre = 0.0f;
    segment.RB_pre = 0.0f;
    segment.K = 0.2f;
    segment.diffusion_c = 0.f;
    segment.diffusion_w = 0.f;
    segment.diffusion_b = 0.f;

    segment.pairs_count = 0;
    segment.moisture = 1.f;

    segment.moisture_pre = 1.f;
    segment.diffusion_m = 0.f;
    segment.cube_pattern = 0;
    segment.internal_pattern = 0;

    segment.inertia_tensor = ComputeInertiaTensorRod(segment.original_mass, segment.radius, segment.rest_length);
    segment.inv_inertia_tensor = 1.f / segment.inertia_tensor;
    const float area = glm::pi<float>() * segment.radius * segment.radius;

    ModulusGraph::Output::ShearStretchModulusType max_stretch_shear_modulus =
        initialize_parameters.modulus_graph.GetShearStretchModulus(modulus_input);
    segment.max_young_modulus =
        glm::max(1e-9f,
                 ActivationFunction::Sigmoid(max_stretch_shear_modulus.x, max_stretch_shear_modulus.y,
                                             initialize_parameters.sapwood_offset,
                                             1.f / initialize_parameters.wood_transition, distance_to_boundary)) *
        1e9f;
    segment.strength =
        glm::max(1e-9f, 1.0f - initialize_parameters.damage_graph.GetValue(
                                   glm::vec3(target_strand_segment_data.profile_position * segment.radius * 2.f,
                                             target_strand_segment_data.end_root_distance) /
                                   initialize_parameters.damage_scale_factor));
    segment.boundary_distance = distance_to_boundary;
    segment.profile_position = target_strand_segment_data.profile_position;

    segment.profile_polar_coordinate = target_strand_segment_data.profile_polar_coordinate;

    segment.shear_stretch_alpha = 1.f / (segment.max_young_modulus * area / segment.rest_length);

    StrengthGraph::Input strength_input;
    strength_input.root_distance = root_distance;
    strength_input.polar_distance = segment.profile_polar_coordinate.x;
    strength_input.polar_angle = segment.profile_polar_coordinate.y;
    strength_input.profile_boundary_distance = distance_to_boundary;

    StrengthGraph::Output::ShearStretchStrengthType shear_stretch_strength =
        initialize_parameters.strength_graph.GetShearStretchStrength(strength_input);
    const float max_shear_stretch_strain = glm::max(
        0.001f, trunk_strength_factor + ActivationFunction::Sigmoid(shear_stretch_strength.x, shear_stretch_strength.y,
                                                                    initialize_parameters.sapwood_offset,
                                                                    1.f / initialize_parameters.wood_transition,
                                                                    distance_to_boundary));
    segment.shear_stretch_strain_limit = segment.max_shear_stretch_strain = max_shear_stretch_strain;

    const auto& strand_segment = randomly_subdivided_strand_group.PeekStrandSegment(static_cast<int>(segment_handle));
    const auto& strand_segment_data =
        randomly_subdivided_strand_group.PeekStrandSegmentData(static_cast<int>(segment_handle));
    auto& particle0 = segment.particle0;
    auto& particle1 = segment.particle1;
    segment.group_index = 0;
    particle0.x0 = particle0.x = particle0.last_x = glm::vec3(initialize_parameters.root_transform.TransformPoint(
        randomly_subdivided_strand_group.GetStrandSegmentStart(static_cast<int>(segment_handle))));

    particle1.x0 = particle1.x = particle1.last_x =
        glm::vec3(initialize_parameters.root_transform.TransformPoint(strand_segment.end_position));

    particle0.acceleration = particle1.acceleration = glm::vec3(0.0);
    particle0.node_handle = particle1.node_handle = strand_segment_data.node_handle;
  });

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
    first_uniform_particle.is_on_surface = 0;
    first_uniform_particle.is_bark = 1;
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
      uniform_particle.is_on_surface = 0;
      uniform_particle.is_bark = 1;
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
        const auto& random_segment_data =
            randomly_subdivided_strand_group.PeekStrandSegmentData(uniform_particle.segment_handle);
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

    // give the option to use either cubic hermite spline or linear interpolation
    if (initialize_parameters.use_cubic_hermite_spline) {
      const glm::vec3& p0 = segments[uniform_particle.segment_handle].particle0.x;
      const glm::vec3& p1 = segments[uniform_particle.segment_handle].particle1.x;

      // use cubic hermite spline interpolation to update uniform particles
      // particle0 tangent
      int prev_segment_handle = segments[uniform_particle.segment_handle].prev_handle;
      glm::vec3 M0;
      if (prev_segment_handle == -1) {
        // start point tangent
        M0 = 0.5f * (p1 - p0);
      } else {
        glm::vec3 p_prev = segments[prev_segment_handle].particle0.x;
        M0 = 0.5f * (p1 - p_prev);
      }

      // particle1 tangent
      int next_segment_handle = segments[uniform_particle.segment_handle].next_handle;
      glm::vec3 M1;
      if (next_segment_handle == -1) {
        // end point tangent
        M1 = 0.5f * (p1 - p0);
      } else {
        glm::vec3 p_next = segments[next_segment_handle].particle1.x;
        M1 = 0.5f * (p_next - p0);
      }

      uniform_particles[uniform_particle_index].position =
          DynamicStrandUtils::CubicHermiteSpline(p0, p1, M0, M1, uniform_particle.t);
      uniform_particles[uniform_particle_index].tangent =
          DynamicStrandUtils::CubicHermiteSplineTangent(p0, p1, M0, M1, uniform_particle.t);
    } else {
      uniform_particle.position = glm::mix(particle0.x, particle1.x, uniform_particle.t);
    }

    uniform_particle.initial_position = uniform_particle.position;
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
    const auto& strand_segment_data = randomly_subdivided_strand_group.PeekStrandSegmentData(segment_handle);
    return glm::vec3(strand_segment_data.profile_position.x, strand_segment_data.profile_position.y,
                     strand_segment_data.start_root_distance / average_segment_length);
  };
  const auto calculate_regularized_segment_center = [&](const int segment_handle) {
    const auto& strand_segment_data = randomly_subdivided_strand_group.PeekStrandSegmentData(segment_handle);
    return glm::vec3(strand_segment_data.profile_position.x, strand_segment_data.profile_position.y,
                     (strand_segment_data.start_root_distance + strand_segment_data.end_root_distance) * .5f /
                         average_segment_length);
  };
  const auto calculate_regularized_segment_p1 = [&](const int segment_handle) {
    const auto& strand_segment_data = randomly_subdivided_strand_group.PeekStrandSegmentData(segment_handle);
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
      if (segment_handle_index == static_cast<int>(segment_handles.size()) - 1)
        break;
      const auto segment_pair_handle = segment_handle_index + handle_index_offset;
      auto& segment_pair = segment_pairs[segment_pair_handle];

      segment_pair.segment0_handle = segment0_handle;
      segment_pair.segment1_handle = segment_handles[segment_handle_index + 1];

      auto& segment0_data = segment_data_list[segment0_handle];
      auto& segment1_data = segment_data_list[segment_pair.segment1_handle];

      segment0_data.pair_handles[1] = segment_pair_handle;
      segment1_data.pair_handles[0] = segment_pair_handle;
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
  struct SegmentInfo {
    glm::vec3 p0;
    glm::vec3 p1;
    glm::vec3 center_position;
    int node_handle;
    int strand_handle;
    int segment_handle;
  };

  auto projected_max_bound = glm::vec3(-FLT_MAX);
  auto projected_min_bound = glm::vec3(FLT_MAX);
  for (auto& i : projected_max_bounds)
    projected_max_bound = glm::max(i, projected_max_bound);
  for (auto& i : projected_min_bounds)
    projected_min_bound = glm::min(i, projected_min_bound);

  VoxelGrid<std::vector<SegmentInfo>> projected_voxel_grid;
  constexpr auto projected_cell_size = 1.f;
  projected_voxel_grid.Initialize(projected_cell_size, projected_min_bound - glm::vec3(projected_cell_size) * 2.f,
                                  projected_max_bound + glm::vec3(projected_cell_size) * 2.f, {});

  for (int segment_handle = 0; segment_handle < segments.size(); segment_handle++) {
    const auto& strand_segment_data = randomly_subdivided_strand_group.PeekStrandSegmentData(segment_handle);
    SegmentInfo s_d;
    s_d.p0 = calculate_regularized_segment_p0(segment_handle);
    s_d.p1 = calculate_regularized_segment_p1(segment_handle);
    s_d.center_position = calculate_regularized_segment_center(segment_handle);
    s_d.node_handle = strand_segment_data.node_handle;
    s_d.strand_handle = randomly_subdivided_strand_group.PeekStrandSegment(segment_handle).GetStrandHandle();
    s_d.segment_handle = segment_handle;
    projected_voxel_grid.Ref(s_d.center_position).emplace_back(s_d);
  }
  std::multimap<float, std::map<std::pair<int, int>, std::pair<float, float>>> candidates;
  for (int segment_handle = 0; segment_handle < segments.size(); segment_handle++) {
    const auto& strand_segment_data = randomly_subdivided_strand_group.PeekStrandSegmentData(segment_handle);
    const auto p0 = calculate_regularized_segment_p0(segment_handle);
    const auto p1 = calculate_regularized_segment_p1(segment_handle);

    const auto extended_p0 = p0 - glm::vec3(0, 0, initialize_parameters.neighbor_vertical_range);
    const auto extended_p1 = p1 + glm::vec3(0, 0, initialize_parameters.neighbor_vertical_range);

    const auto center = calculate_regularized_segment_center(segment_handle);
    const auto strand_handle = randomly_subdivided_strand_group.PeekStrandSegment(segment_handle).GetStrandHandle();
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
  uint32_t max_counter = 0;
  for (const auto& counter : counters) {
    max_counter = glm::max(counter, max_counter);
  }
  EVOENGINE_LOG("Max counter: " + std::to_string(max_counter));

  Jobs::RunParallelFor(segment_pairs.size(), [&](const auto pair_index) {
    auto& segment_pair = segment_pairs[pair_index];
    auto& segment0 = segments[segment_pair.segment0_handle];
    auto& segment1 = segments[segment_pair.segment1_handle];

    const auto& target_strand_segment0_data = target_strand_segment_data_list[segment_pair.segment0_handle];
    const auto& target_strand_segment1_data = target_strand_segment_data_list[segment_pair.segment1_handle];

    const float root_distance =
        (target_strand_segment0_data.start_root_distance + target_strand_segment0_data.end_root_distance +
         target_strand_segment1_data.start_root_distance + target_strand_segment1_data.end_root_distance) *
        .25f;
    BiologicalPropertiesGraph::Input biological_properties_input;
    biological_properties_input.root_distance = root_distance;
    biological_properties_input.polar_distance = (target_strand_segment0_data.profile_polar_coordinate.x +
                                                  target_strand_segment1_data.profile_polar_coordinate.x) *
                                                 .5f;
    biological_properties_input.polar_angle = (target_strand_segment0_data.profile_polar_coordinate.y +
                                               target_strand_segment1_data.profile_polar_coordinate.y) *
                                              .5f;
    biological_properties_input.profile_boundary_distance = (target_strand_segment0_data.initial_distance_to_boundary +
                                                             target_strand_segment1_data.initial_distance_to_boundary) *
                                                            .5f;
    BiologicalPropertiesGraph::Output biological_properties =
        initialize_parameters.biological_properties_graph.GetValues(biological_properties_input);
    const float trunk_strength_factor =
        initialize_parameters.trunk_additional_strength
            ? ActivationFunction::Sigmoid(biological_properties.trunk_additional_strength_factor, 0.f,
                                          biological_properties.trunk_offset,
                                          1.f / biological_properties.trunk_transition, root_distance)
            : 0.f;
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
    const float distance_to_boundary = (segment0.boundary_distance + segment1.boundary_distance) * .5f;

    ModulusGraph::Input modulus_graph_input;
    modulus_graph_input.root_distance = root_distance;
    modulus_graph_input.polar_distance = (target_strand_segment0_data.profile_polar_coordinate.x +
                                          target_strand_segment1_data.profile_polar_coordinate.x) *
                                         .5f;
    modulus_graph_input.polar_angle = (target_strand_segment0_data.profile_polar_coordinate.y +
                                       target_strand_segment1_data.profile_polar_coordinate.y) *
                                      .5f;
    modulus_graph_input.profile_boundary_distance = (target_strand_segment0_data.initial_distance_to_boundary +
                                                     target_strand_segment1_data.initial_distance_to_boundary) *
                                                    .5f;

    glm::vec2 max_bending_modulus = initialize_parameters.modulus_graph.GetBendingModulus(modulus_graph_input);
    segment_pair.max_bending_modulus =
        glm::max(1e-9f, ActivationFunction::Sigmoid(
                            max_bending_modulus.x, max_bending_modulus.y, initialize_parameters.sapwood_offset,
                            1.f / initialize_parameters.wood_transition, distance_to_boundary)) *
        1e9f;

    glm::vec2 max_twisting_modulus = initialize_parameters.modulus_graph.GetTwistingModulus(modulus_graph_input);
    segment_pair.max_torsion_modulus =
        glm::max(1e-9f, ActivationFunction::Sigmoid(
                            max_twisting_modulus.x, max_twisting_modulus.y, initialize_parameters.sapwood_offset,
                            1.f / initialize_parameters.wood_transition, distance_to_boundary)) *
        1e9f;
    const float segment_radius = (segment0.radius + segment1.radius) * .5f;
    const float segment_length = (segment0.rest_length + segment1.rest_length) * .5f;
    const auto second_moment_of_area = glm::pi<float>() * std::pow(segment_radius, 4.f) * 0.25f;
    const auto polar_moment_of_inertia = glm::pi<float>() * std::pow(segment_radius, 4.f) * 0.5f;
    segment_pair.bending_alpha =
        1.f / (segment_pair.max_bending_modulus * second_moment_of_area / glm::pow(segment_length, 3.f));
    segment_pair.torsion_alpha = 1.f / (segment_pair.max_torsion_modulus * polar_moment_of_inertia / segment_length);
    const auto& q0 = segment0.q0;
    const auto& q1 = segment1.q0;
    segment_pair.rest_darboux_vector = glm::conjugate(q0) * q1;

    StrengthGraph::Input strength_graph_input;  // TODO: update parameters
    strength_graph_input.root_distance = root_distance;
    strength_graph_input.polar_distance = (target_strand_segment0_data.profile_polar_coordinate.x +
                                           target_strand_segment1_data.profile_polar_coordinate.x) *
                                          .5f;
    strength_graph_input.polar_angle = (target_strand_segment0_data.profile_polar_coordinate.y +
                                        target_strand_segment1_data.profile_polar_coordinate.y) *
                                       .5f;
    strength_graph_input.profile_boundary_distance = (target_strand_segment0_data.initial_distance_to_boundary +
                                                      target_strand_segment1_data.initial_distance_to_boundary) *
                                                     .5f;
    // TODO: evaluate all at once

    glm::vec2 bending_strength = initialize_parameters.strength_graph.GetBendingStrength(strength_graph_input);
    const float max_bending_strain = glm::max(
        0.001f, trunk_strength_factor + ActivationFunction::Sigmoid(bending_strength.x, bending_strength.y,
                                                                    initialize_parameters.sapwood_offset,
                                                                    1.f / initialize_parameters.wood_transition,
                                                                    distance_to_boundary));
    glm::vec2 twisting_strength = initialize_parameters.strength_graph.GetTwistingStrength(strength_graph_input);
    const float max_twisting_strain = glm::max(
        0.001f, trunk_strength_factor + ActivationFunction::Sigmoid(twisting_strength.x, twisting_strength.y,
                                                                    initialize_parameters.sapwood_offset,
                                                                    1.f / initialize_parameters.wood_transition,
                                                                    distance_to_boundary));
    glm::vec2 bundle_strength = initialize_parameters.strength_graph.GetBundleStrength(strength_graph_input);

    const float max_bundle_strain = glm::max(
        0.001f, trunk_strength_factor + ActivationFunction::Sigmoid(
                                            bundle_strength.x, bundle_strength.y, initialize_parameters.sapwood_offset,
                                            1.f / initialize_parameters.wood_transition, distance_to_boundary));
    glm::vec2 connectivity_strength =
        initialize_parameters.strength_graph.GetConnectivityStrength(strength_graph_input);
    const float max_connectivity_strain = glm::max(
        0.001f, trunk_strength_factor + ActivationFunction::Sigmoid(connectivity_strength.x, connectivity_strength.y,
                                                                    initialize_parameters.sapwood_offset,
                                                                    1.f / initialize_parameters.wood_transition,
                                                                    distance_to_boundary));
    segment_pair.max_bending_twist_bundle_strain = segment_pair.bending_twist_bundle_strain_limit =
        glm::vec3(max_bending_strain, max_twisting_strain, max_bundle_strain);
    segment_pair.max_connectivity_strain = segment_pair.connectivity_strain_limit = max_connectivity_strain;

    segment_pair.compression_lock = segment_pair.positional_lock = segment_pair.rotational_lock =
        segment_pair.tensile_lock = 0;
  });
  // set up nodes
  auto& skeleton_nodes = strand_model_skeleton.PeekRawNodes();
  nodes.resize(skeleton_nodes.size());

  for (size_t i = 0; i < skeleton_nodes.size(); i++) {
    nodes[i].prev_handle = skeleton_nodes[i].GetParentHandle();
  }

  if (!initialize_parameters.triangulate_per_bundle) {
    ComputeDelaunay(delaunay_tetrahedrons, initialize_parameters.use_cgal, initialize_parameters.min_bundle_size);
  } else {
    ComputeDelaunayPerBundle(delaunay_tetrahedrons, initialize_parameters.use_cgal);
  }
  for (const auto& i : constraints)
    i->InitializeData(initialize_parameters, strand_model_skeleton, randomly_subdivided_strand_group, *this);

  hashed_grid_elements.resize(segments.size());
  hashed_grid_cell_starts.resize(HASH_GRID_CELL_SIZE);

  // Create foliage here.
  auto initialize_parameters_copy = initialize_parameters;
  auto fd = initialize_parameters_copy.foliage_descriptor.Get<BasicFoliageDescriptor>();
  if (!fd) {
    fd = AssetManager::CreateTemporaryAsset<BasicFoliageDescriptor>();
  }
  const auto& node_list = strand_model_skeleton.PeekSortedNodeList();
  const auto tree_dim = strand_model_skeleton.max - strand_model_skeleton.min;

  VoxelGrid<std::vector<SegmentInfo>> voxel_grid;
  const auto current_leaf_size = fd->leaf_size * glm::length(tree_dim) * 0.1f;
  const auto cell_size = 2.f * (current_leaf_size.y + fd->stem_length.GetValue() * glm::length(tree_dim) * 0.1f) +
                         initialize_parameters.max_segment_length;

  auto max_bound = glm::vec3(-FLT_MAX);
  auto min_bound = glm::vec3(FLT_MAX);
  for (auto& i : max_bounds)
    max_bound = glm::max(i, max_bound);
  for (auto& i : min_bounds)
    min_bound = glm::min(i, min_bound);

  voxel_grid.Initialize(cell_size, min_bound - glm::vec3(cell_size) * 2.f, max_bound + glm::vec3(cell_size) * 2.f, {});
  std::unordered_set<int> enabled_node_handles;
  for (int segment_handle = 0; segment_handle < segments.size(); segment_handle++) {
    const auto& strand_segment_data = randomly_subdivided_strand_group.PeekStrandSegmentData(segment_handle);
    SegmentInfo s_d;
    const auto& segment = segments[segment_handle];
    s_d.p0 = segment.particle0.x0;
    s_d.p1 = segment.particle1.x0;
    s_d.center_position = (s_d.p0 + s_d.p1) * 0.5f;
    s_d.node_handle = strand_segment_data.node_handle;
    enabled_node_handles.emplace(s_d.node_handle);
    s_d.strand_handle = randomly_subdivided_strand_group.PeekStrandSegment(segment_handle).GetStrandHandle();
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
}
