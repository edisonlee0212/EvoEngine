#include "DsAlphaShapeMeshing.hpp"
#include "ComputePipeline.hpp"
#include "DsAlphaShapeUtils.hpp"
#include "DynamicStrands.hpp"
#include "Platform/Platform.hpp"
#include "Shader.hpp"

using namespace eco_sys_lab_plugin;

bool DsAlphaShapeVisualizationParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Uniform Particle", &render_uniform_particles))
    changed = true;
  if (render_uniform_particles) {
    if (ImGui::Combo("Uniform particle mode", {"Default", "Segment color", "Single Particles"},
                     uniform_particle_render_mode))
      changed = true;
    switch (uniform_particle_render_mode) {
      case 0: {
        if (ImGui::ColorEdit4("Uniform particle color", &uniform_particle_main.x))
          changed = true;
        break;
      }
    }
    if (ImGui::DragFloat("Uniform Particle multiplier", &uniform_particle_radius_multiplier, 0.1f, 0.1f, 1000.f))
      changed = true;
  }

  return changed;
}

DsAlphaShapeMeshing::DsAlphaShapeMeshing() : DsMeshing() {
  mesh_wireframe_rendering_instance_handle = Handle();
  small_segments_rendering_instance_handle = Handle();
}

DsAlphaShapeMeshing::~DsAlphaShapeMeshing() {
}

DsAlphaShapeMeshing::RenderSettings DsAlphaShapeMeshing::render_settings = {};

void DsAlphaShapeMeshing::InitData(const DynamicStrandsInitializeParameters& initialize_parameters,
                                   const StrandModelSkeleton& strand_model_skeleton,
                                   const StrandModelStrandGroup& strand_model_strand_group,
                                   DtsStrandGroup& randomly_subdivided_strand_group,
                                   DtsStrandGroup& uniformly_subdivided_strand_group) {
  // First compute uniform particles
  const auto& target_strands = randomly_subdivided_strand_group.PeekStrands();

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
    const auto& segment = dynamic_strands->segments[uniform_particle.segment_handle];
    const auto& particle0 = segment.particle0;
    const auto& particle1 = segment.particle1;

    // give the option to use either cubic hermite spline or linear interpolation
    if (initialize_parameters.use_cubic_hermite_spline) {
      const glm::vec3& p0 = dynamic_strands->segments[uniform_particle.segment_handle].particle0.x;
      const glm::vec3& p1 = dynamic_strands->segments[uniform_particle.segment_handle].particle1.x;

      // use cubic hermite spline interpolation to update uniform particles
      // particle0 tangent
      int prev_segment_handle = dynamic_strands->segments[uniform_particle.segment_handle].prev_handle;
      glm::vec3 M0;
      if (prev_segment_handle == -1) {
        // start point tangent
        M0 = 0.5f * (p1 - p0);
      } else {
        glm::vec3 p_prev = dynamic_strands->segments[prev_segment_handle].particle0.x;
        M0 = 0.5f * (p1 - p_prev);
      }

      // particle1 tangent
      int next_segment_handle = dynamic_strands->segments[uniform_particle.segment_handle].next_handle;
      glm::vec3 M1;
      if (next_segment_handle == -1) {
        // end point tangent
        M1 = 0.5f * (p1 - p0);
      } else {
        glm::vec3 p_next = dynamic_strands->segments[next_segment_handle].particle1.x;
        M1 = 0.5f * (p_next - p0);
      }

      uniform_particles[uniform_particle_index].position =
          DsAlphaShapeUtils::CubicHermiteSpline(p0, p1, M0, M1, uniform_particle.t);
      uniform_particles[uniform_particle_index].tangent =
          DsAlphaShapeUtils::CubicHermiteSplineTangent(p0, p1, M0, M1, uniform_particle.t);
    } else {
      uniform_particle.position = glm::mix(particle0.x, particle1.x, uniform_particle.t);
    }

    uniform_particle.initial_position = uniform_particle.position;
    uniform_particle.normal = glm::vec3(0.0f);
    uniform_particle.deg = 0.0f;
  });

  // Now compute Delaunay tetrahedrons
  if (!initialize_parameters.triangulate_per_bundle) {
    ComputeDelaunay(delaunay_tetrahedrons, initialize_parameters.use_cgal, initialize_parameters.min_bundle_size);
  } else {
    ComputeDelaunayPerBundle(delaunay_tetrahedrons, initialize_parameters.use_cgal);
  }
}

void DsAlphaShapeMeshing::InitBuffer(VkBufferCreateInfo& buffer_create_info,
                                     VmaAllocationCreateInfo& buffer_vma_allocation_create_info) {
  device_uniform_particles_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_delaunay_tetrahedrons_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
}

void DsAlphaShapeMeshing::InitializationGraphicsPipeline(
    const DynamicStrandsInitializeParameters& initialize_parameters) {
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
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Initialization/AlphaShapeMeshing/Interior.comp");
    interior_initialization_pipeline = std::make_shared<ComputePipeline>();
    interior_initialization_pipeline->compute_shader = shader;
    interior_initialization_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
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
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Initialization/AlphaShapeMeshing/BarkFlag.comp");
    bark_flag_initialization_pipeline = std::make_shared<ComputePipeline>();
    bark_flag_initialization_pipeline->compute_shader = shader;
    bark_flag_initialization_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
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
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Initialization/AlphaShapeMeshing/Normal.comp");
    uniform_particle_initialization_pipeline = std::make_shared<ComputePipeline>();
    uniform_particle_initialization_pipeline->compute_shader = shader;
    uniform_particle_initialization_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
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
        vk_command_buffer, 0, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    interior_initialization_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    vkCmdDispatch(vk_command_buffer, delaunay_tetrahedrons_group_size, 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });

  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    bark_flag_initialization_pipeline->Bind(vk_command_buffer);
    bark_flag_initialization_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    bark_flag_initialization_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    vkCmdDispatch(vk_command_buffer, delaunay_tetrahedrons_group_size, 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });

  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    uniform_particle_initialization_pipeline->Bind(vk_command_buffer);
    uniform_particle_initialization_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    uniform_particle_initialization_pipeline->PushConstant(vk_command_buffer, 0, uniform_particle_push_constant);
    vkCmdDispatch(vk_command_buffer, uniform_particles_group_size, 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

struct TetrahedronFilteringPushConstant {
  uint32_t tetrahedrons_size = 0;
  float alpha = 0.0f;
  float bifurcation_alpha = 0.0f;
  float max_dist_squared = 0.0f;
  int render_complex = 0;
  float degen_triangle_threshold = 0.0f;
  float break_threshold = 0.02f;
  int persistent_damage;
};

struct UniformParticlePredictionPushConstant {
  uint32_t uniform_particle_size = 0;
  float snow_factor = 50.f;
  float snow_deduction = 0.1f;
  int use_cubic_hermite_spline = 0;
};

void eco_sys_lab_plugin::DsAlphaShapeMeshing::BuildRenderComputePipelines() {
  static std::shared_ptr<Shader> shader{};
  shader = std::make_shared<Shader>();
  shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                     std::filesystem::path("./EcoSysLabResources") /
                         "Shaders/Compute/DynamicStrands/Prediction/AlphaShapeMeshing/UniformParticle.comp");

  branches_uniform_particle_update_pipeline = std::make_shared<ComputePipeline>();
  branches_uniform_particle_update_pipeline->compute_shader = shader;
  branches_uniform_particle_update_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

  auto& push_constant_range = branches_uniform_particle_update_pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(UniformParticlePredictionPushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  branches_uniform_particle_update_pipeline->Initialize();

  // Tetrahedrons
  branches_tetrahedron_filtering_pipeline = std::make_shared<ComputePipeline>();
  branches_tetrahedron_filtering_pipeline->compute_shader = Shader::CreateTemporary(
      ShaderType::Compute, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Compute/DynamicStrands/Rendering/AlphaShapeMeshing/TetrahedronFiltering.comp");
  branches_tetrahedron_filtering_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

  auto& tetrahedron_filtering_push_constant_range =
      branches_tetrahedron_filtering_pipeline->push_constant_ranges.emplace_back();
  tetrahedron_filtering_push_constant_range.size = sizeof(TetrahedronFilteringPushConstant);
  tetrahedron_filtering_push_constant_range.offset = 0;
  tetrahedron_filtering_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  branches_tetrahedron_filtering_pipeline->Initialize();

  // Triangles
  branches_triangle_filtering_pipeline = std::make_shared<ComputePipeline>();
  branches_triangle_filtering_pipeline->compute_shader =
      Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Compute/DynamicStrands/Rendering/AlphaShapeMeshing/TriangleFiltering.comp");
  branches_triangle_filtering_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

  auto& triangle_filtering_push_constant_range =
      branches_triangle_filtering_pipeline->push_constant_ranges.emplace_back();
  triangle_filtering_push_constant_range.size = sizeof(TetrahedronFilteringPushConstant);
  triangle_filtering_push_constant_range.offset = 0;
  triangle_filtering_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  branches_triangle_filtering_pipeline->Initialize();
}

void eco_sys_lab_plugin::DsAlphaShapeMeshing::RenderCompute() const {
  if (dynamic_strands->segments.empty())
    return;
  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const float snow_factor = 100.f;
  const float snow_deduction = 0.1f;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    // Uniform Particles
    UniformParticlePredictionPushConstant uniform_particle_push_constant;
    uniform_particle_push_constant.uniform_particle_size = uniform_particles.size();
    uniform_particle_push_constant.snow_deduction = snow_deduction;
    uniform_particle_push_constant.snow_factor = snow_factor;
    uniform_particle_push_constant.use_cubic_hermite_spline =
        render_settings.branches_render_parameters.use_cubic_hermite_spline ? 1 : 0;
    branches_uniform_particle_update_pipeline->Bind(vk_command_buffer);
    branches_uniform_particle_update_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    branches_uniform_particle_update_pipeline->PushConstant(vk_command_buffer, 0, uniform_particle_push_constant);
    vkCmdDispatch(vk_command_buffer,
                  Platform::DivUp(uniform_particle_push_constant.uniform_particle_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    // Tetrahedrons
    TetrahedronFilteringPushConstant filtering_push_constant;
    filtering_push_constant.tetrahedrons_size = delaunay_tetrahedrons.size();
    filtering_push_constant.alpha = render_settings.branches_render_parameters.alpha;
    filtering_push_constant.bifurcation_alpha = render_settings.branches_render_parameters.bifurcation_alpha;
    filtering_push_constant.max_dist_squared = render_settings.branches_render_parameters.max_dist_squared;
    filtering_push_constant.render_complex = render_settings.branches_render_parameters.render_complex ? 1 : 0;
    filtering_push_constant.degen_triangle_threshold =
        pow(10.0f, -render_settings.branches_render_parameters.degen_triangle_threshold_logairthmic);
    filtering_push_constant.break_threshold = render_settings.branches_render_parameters.break_threshold;
    filtering_push_constant.persistent_damage = render_settings.branches_render_parameters.persistent_damage ? 1 : 0;

    branches_tetrahedron_filtering_pipeline->Bind(vk_command_buffer);
    branches_tetrahedron_filtering_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    branches_tetrahedron_filtering_pipeline->PushConstant(vk_command_buffer, 0, filtering_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(filtering_push_constant.tetrahedrons_size, work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    // Triangles
    branches_triangle_filtering_pipeline->Bind(vk_command_buffer);
    branches_triangle_filtering_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    branches_triangle_filtering_pipeline->PushConstant(vk_command_buffer, 0, filtering_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(filtering_push_constant.tetrahedrons_size, work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void DsAlphaShapeMeshing::BuildRenderingPipelines() {
  BuildBranchesRenderingPipelines();
  BuildSmallSegmentsRenderingPipelines();
}

void eco_sys_lab_plugin::DsAlphaShapeMeshing::Download() {
  if (!uniform_particles.empty())
    device_uniform_particles_buffer->DownloadVector(uniform_particles, uniform_particles.size());
  if (!delaunay_tetrahedrons.empty())
    device_delaunay_tetrahedrons_buffer->DownloadVector(delaunay_tetrahedrons, delaunay_tetrahedrons.size());
}

void eco_sys_lab_plugin::DsAlphaShapeMeshing::Upload() {
  device_uniform_particles_buffer->UploadVector(uniform_particles);
  device_uniform_particles_buffer->SetDebugName("Uniform Particles Buffer");
  device_delaunay_tetrahedrons_buffer->UploadVector(delaunay_tetrahedrons);
  device_delaunay_tetrahedrons_buffer->SetDebugName("Delaunay Tetrahedrons Buffer");
}

void eco_sys_lab_plugin::DsAlphaShapeMeshing::Clear() {
  uniform_particles.clear();
  delaunay_tetrahedrons.clear();
}

void eco_sys_lab_plugin::DsAlphaShapeMeshing::UpdateBindings() const {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  // TODO: tie to alpha shape meshing only
  // TODO: use a different descriptor set for meshing
  dynamic_strands->strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
      8, device_uniform_particles_buffer, 0);
  dynamic_strands->strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
      9, device_delaunay_tetrahedrons_buffer, 0);
}

bool eco_sys_lab_plugin::DsAlphaShapeMeshing::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  return false;
}
void DsAlphaShapeMeshing::Stats(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Text((std::string("Uniform particles count: ") + std::to_string(uniform_particles.size())).c_str());
  ImGui::Text((std::string("Meshlet count: ") + std::to_string(delaunay_tetrahedrons.size())).c_str());
}

void DsAlphaShapeMeshing::OnInspectRenderSettings(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Checkbox("Render branches", &render_settings.branches_render_parameters.enabled);
  if (render_settings.branches_render_parameters.enabled) {
    if (ImGui::TreeNodeEx("Branch render settings")) {
      if (ImGui::Button("Rebuild branches pipelines")) {
        BuildBranchesRenderingPipelines();
      }
      render_settings.branches_render_parameters.OnInspect(editor_layer);
      ImGui::TreePop();
    }
  }
  ImGui::Checkbox("Render Visualization", &render_settings.visualization_rendering);
  if (render_settings.visualization_rendering) {
    ImGui::Checkbox("Render strands", &render_settings.small_segments_visualization_render_parameters.enabled);
    if (render_settings.small_segments_visualization_render_parameters.enabled) {
      if (ImGui::TreeNodeEx("Strands render settings")) {
        render_settings.small_segments_visualization_render_parameters.OnInspect(editor_layer);
        ImGui::TreePop();
      }
    }
  } else {
    ImGui::Checkbox("Render splinters", &render_settings.small_segments_render_parameters.enabled);
    if (render_settings.small_segments_render_parameters.enabled) {
      if (ImGui::TreeNodeEx("Splinter render settings")) {
        render_settings.small_segments_render_parameters.OnInspect(editor_layer);
        ImGui::TreePop();
      }
    }
  }
}

void DsAlphaShapeMeshing::Visualize(const std::shared_ptr<Camera>& target_camera,
                                    const DynamicStrandsInitializeParameters& initialize_parameters,
                                    const DynamicStrandsVisualizationParameters& visualization_parameters) {
  // Mesh shader support should be checked before calling this function
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  const auto current_frame_index = Platform::GetCurrentFrameIndex();

  static std::shared_ptr<GraphicsPipeline> uniform_particle_render_pipeline{};
  struct UniformParticleRenderPushConstant {
    glm::vec4 min_color = glm::vec4(0.2f);
    glm::vec4 max_color = glm::vec4(1.f);

    uint32_t camera_index = 0;
    uint32_t strand_uniform_particle_size = 0;
    uint32_t render_mode = 2;
    float multiplier = 10.0f;
  };

  if (!uniform_particle_render_pipeline) {
    static std::shared_ptr<Shader> task_shader{};
    static std::shared_ptr<Shader> mesh_shader{};
    static std::shared_ptr<Shader> frag_shader{};
    // Load shader
    task_shader = std::make_shared<Shader>();
    task_shader->TryCompile(
        ShaderType::Task, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") /
            "Shaders/Graphics/Task/DynamicStrands/Visualization/AlphaShapeMeshing/UniformParticles.task");
    mesh_shader = std::make_shared<Shader>();
    mesh_shader->TryCompile(
        ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") /
            "Shaders/Graphics/Mesh/DynamicStrands/Visualization/AlphaShapeMeshing/UniformParticles.mesh");

    frag_shader = std::make_shared<Shader>();
    frag_shader->TryCompile(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/DynamicStrands/Visualization.frag");
    // Descriptor set layout
    uniform_particle_render_pipeline = std::make_shared<GraphicsPipeline>();
    uniform_particle_render_pipeline->task_shader = task_shader;
    uniform_particle_render_pipeline->mesh_shader = mesh_shader;

    uniform_particle_render_pipeline->fragment_shader = frag_shader;
    uniform_particle_render_pipeline->geometry_type = GeometryType::Mesh;

    uniform_particle_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
    uniform_particle_render_pipeline->descriptor_set_layouts.emplace_back(dynamic_strands->strands_layout);
    uniform_particle_render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    uniform_particle_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    uniform_particle_render_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};

    auto& push_constant_range = uniform_particle_render_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(UniformParticleRenderPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    uniform_particle_render_pipeline->Initialize();
  }

  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  UniformParticleRenderPushConstant uniform_particle_push_constant;
  uniform_particle_push_constant.render_mode =
      render_settings.meshing_visualization_parameters.uniform_particle_render_mode;
  uniform_particle_push_constant.min_color = render_settings.meshing_visualization_parameters.uniform_particle_main;
  uniform_particle_push_constant.camera_index =
      render_layer->GetCurrentRenderInstanceStorage()->GetCameraIndex(target_camera->GetHandle());
  uniform_particle_push_constant.multiplier =
      render_settings.meshing_visualization_parameters.uniform_particle_radius_multiplier;
  uniform_particle_push_constant.strand_uniform_particle_size = uniform_particles.size();

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
#pragma region Viewport and scissor
    VkViewport viewport;
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = target_camera->GetSize().x;
    viewport.height = target_camera->GetSize().y;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = target_camera->GetSize().x;
    scissor.extent.height = target_camera->GetSize().y;
#pragma endregion
    if (render_settings.meshing_visualization_parameters.render_uniform_particles) {
      uniform_particle_render_pipeline->states.ResetAllStates(1);
      uniform_particle_render_pipeline->states.view_port = viewport;
      uniform_particle_render_pipeline->states.scissor = scissor;
      uniform_particle_render_pipeline->states.polygon_mode = VK_POLYGON_MODE_FILL;
      uniform_particle_render_pipeline->states.color_blend_attachment_states[0].blendEnable = true;

      uniform_particle_render_pipeline->states.ApplyAllStates(vk_command_buffer);
      target_camera->GetRenderTexture()->Render(
          vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
            uniform_particle_render_pipeline->Bind(vk_command_buffer);
            uniform_particle_render_pipeline->BindDescriptorSet(
                vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
            uniform_particle_render_pipeline->BindDescriptorSet(
                vk_command_buffer, 1,
                dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
            uniform_particle_render_pipeline->PushConstant(vk_command_buffer, 0, uniform_particle_push_constant);
            const uint32_t count = Platform::DivUp(uniform_particles.size(),
                                                   task_work_group_invocations);  // TODO: move to meshing
            vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
          });
    }
  });
}

#pragma region specific functions for alpha shape meshing
void DsAlphaShapeMeshing::ComputeDelaunayPerBundle(std::vector<GpuDelaunayTetrahedron>& tetrahedrons, bool use_cgal) {
  int max_dist_from_root = 0;

  for (int i = 0; i < uniform_particles.size(); i++) {
    max_dist_from_root = std::max(max_dist_from_root, uniform_particles[i].segment_index);
  }

  std::vector<std::map<int, std::vector<size_t>>> bundle_maps(max_dist_from_root + 1);
  std::vector<size_t> offsets(max_dist_from_root + 1, 0);
  std::vector<std::vector<size_t>> particle_adjacent_tets(uniform_particles.size(), std::vector<size_t>{});

  for (int i = 0; i < uniform_particles.size(); i++) {
    auto& particle = uniform_particles[i];
    auto& node_handle = particle.node_index;

    if (bundle_maps[particle.segment_index].find(node_handle) == bundle_maps[particle.segment_index].end()) {
      bundle_maps[particle.segment_index][node_handle] = std::vector<size_t>();
    }

    bundle_maps[particle.segment_index][node_handle].push_back(i);
  }

  // TODO: "squish" each bundle such that no internal degenerate tetrahedrons occur
#ifdef USE_CGAL
  // triangulate each bundle:
  for (int d = 0; d < bundle_maps.size(); d++) {
    offsets[d] = tetrahedrons.size();
    auto& map = bundle_maps[d];
    for (auto& kv_pair : map) {
      auto& bundle = kv_pair.second;
      std::vector<std::pair<Point_CGAL, unsigned>> points;

      if (bundle.size() < 3) {
        continue;
      }

      for (size_t i : bundle) {
        auto& particle = uniform_particles[i];

        if (particle.next_particle_handle == -1) {
          continue;
          // TODO: probably even means we can skip this bundle entirely
        }

        auto& next_particle = uniform_particles[particle.next_particle_handle];

        float squish_weight = 0.0f;

        glm::vec3 squished_position =
            squish_weight * particle.position + (1.0f - squish_weight) * next_particle.position;

        Point_CGAL p0_cgal(particle.position[0], particle.position[1], particle.position[2]);
        Point_CGAL p1_cgal(squished_position[0], squished_position[1], squished_position[2]);

        points.emplace_back(p0_cgal, i);
        points.emplace_back(p1_cgal, particle.next_particle_handle);
      }

      CGALDelaunay(points, tetrahedrons);
    }
  }
#else
  for (int d = 0; d < bundle_maps.size(); d++) {
    offsets[d] = tetrahedrons.size();
    auto& map = bundle_maps[d];
    for (auto& kv_pair : map) {
      auto& bundle = kv_pair.second;
      std::vector<glm::vec3> points;
      std::vector<size_t> indices;

      if (bundle.size() < 3) {
        continue;
      }

      int end_of_strand_count = 0;
      for (size_t i : bundle) {
        auto& particle = uniform_particles[i];

        glm::vec3 p0(particle.position[0], particle.position[1], particle.position[2]);
        points.emplace_back(p0);
        indices.emplace_back(i);

        if (particle.next_particle_handle == -1) {
          end_of_strand_count++;
        } else {
          auto& next_particle = uniform_particles[particle.next_particle_handle];

          float squish_weight = 0.0f;

          glm::vec3 squished_position =
              squish_weight * particle.position + (1.0f - squish_weight) * next_particle.position;

          glm::vec3 p1(squished_position[0], squished_position[1], squished_position[2]);

          points.emplace_back(p1);
          indices.emplace_back(particle.next_particle_handle);
        }
      }

      if (end_of_strand_count == bundle.size()) {
        continue;
      }

      TetDelaunay(points, indices, tetrahedrons);
    }
  }
#endif

  std::mutex mtx;

  Jobs::RunParallelFor(tetrahedrons.size(), [&](const size_t tet_index) {
    auto& tet = tetrahedrons[tet_index];

    for (size_t i = 0; i < 4; i++) {
      if (tet.indices[i] == -1) {
        continue;
      }

      if (tet.indices[i] >= particle_adjacent_tets.size()) {
        EVOENGINE_ERROR("particle index out of range, skipping!");
        continue;
      }

      mtx.lock();
      particle_adjacent_tets[tet.indices[i]].emplace_back(tet_index);
      mtx.unlock();
    }
  });

  // now glue them back together
  for (int particle_index = 0; particle_index < uniform_particles.size(); particle_index++) {
    auto& adjacent_tets = particle_adjacent_tets[particle_index];

    // can't be too many, brute force should work here;
    for (size_t i = 0; i < adjacent_tets.size(); i++) {
      for (size_t j = i + 1; j < adjacent_tets.size(); j++) {
        auto& tet0 = tetrahedrons[adjacent_tets[i]];
        auto& tet1 = tetrahedrons[adjacent_tets[j]];

        // check if the two share a face
        size_t occurs_in_both = 0;
        size_t b_in_both = 0;

        for (size_t i = 0; i < 4; i++) {
          for (size_t j = 0; j < 4; j++) {
            if (tet0.indices[i] == tet1.indices[j] && tet0.indices[i] != -1) {
              occurs_in_both++;
            }
          }
        }

        if (occurs_in_both != 3) {
          continue;
        }
        const auto mismatch_indices = DsAlphaShapeUtils::CompareIndices(tet0.indices, tet1.indices);

        tet0.neighbor_tet_ids[mismatch_indices.first] = adjacent_tets[j];
        tet1.neighbor_tet_ids[mismatch_indices.second] = adjacent_tets[i];
      }
    }
  }
}

#ifdef USE_CGAL
void DsAlphaShapeMeshing::CGALDelaunay(const std::vector<std::pair<Point_CGAL, unsigned>>& points,
                                       std::vector<GpuDelaunayTetrahedron>& tetrahedrons) {
  Delaunay_CGAL dt;
  dt.insert(points.begin(), points.end());

  // TODO: parallel for
  for (auto cell_it = dt.all_cells_begin(); cell_it != dt.all_cells_end(); cell_it++) {
    auto& cell = *cell_it;
    auto& tetrahedron = dt.tetrahedron(cell_it);
    int indices[4];
    for (size_t i = 0; i < 4; i++) {
      indices[i] = cell.vertex(i)->info();
    }
    if (!DsAlphaShapeUtils::IsValid(indices, uniform_particles.size())) {
      continue;  // discard this tetrahedron
    }

    // only take tetrahedra that sit between two neighboring planes
    if (!DsAlphaShapeUtils::IsBetweenPlanes(indices, uniform_particles)) {
      continue;
    }

    GpuDelaunayTetrahedron gpu_tet;
    for (size_t i = 0; i < 4; i++) {
      gpu_tet.indices[i] = indices[i];
      gpu_tet.neighbor_tet_ids[i] = -1;
      gpu_tet.is_bark[i] = -1;
    }
    // set up debugging members
    gpu_tet.color = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
    for (int& i : gpu_tet.render_neighbor) {
      i = -1;
    }
    gpu_tet.task_looked_at = 0;
    gpu_tet.mesh_looked_at = 0;
    gpu_tet.inside = 0;
    gpu_tet.triangles_accepted = 0;

    // check orientation of the tetrahedron
    const float d = DsAlphaShapeUtils::PointPlaneDistance(
        uniform_particles[gpu_tet.indices[3]].position, uniform_particles[gpu_tet.indices[0]].position,
        uniform_particles[gpu_tet.indices[1]].position, uniform_particles[gpu_tet.indices[2]].position);

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

      if (!DsAlphaShapeUtils::IsValid(neighbor_indices, uniform_particles.size())) {
        continue;
      }

      if (!DsAlphaShapeUtils::IsBetweenPlanes(neighbor_indices, uniform_particles)) {
        continue;
      }

      const auto mismatch_indices = DsAlphaShapeUtils::CompareIndices(gpu_tet.indices, neighbor_indices);
      // TODO: according to CGAL documentation, this is guaranteed anyway, so we do not need to match both sides
      // store it such that the neighboring tetrahedron always consists of different indices
      // e. g. for the triangle 1 2 4 we store the corresponding neighboring index at position 3
      // gpu_tet.neighbor_tet_ids[mismatch_indices.first] = neighbor.index();
      EVOENGINE_ERROR("CGAL does not provide neighbor indices");
    }

    tetrahedrons.emplace_back(gpu_tet);
  }
}
#endif

void DsAlphaShapeMeshing::TetDelaunay(const std::vector<glm::vec3>& points, const std::vector<size_t>& particle_indices,
                                      std::vector<GpuDelaunayTetrahedron>& tetrahedrons) {
  const auto tets = Delaunay3D::GenerateTetrahedrons(points);

  std::vector<int> valid_index_map(tets.size(), -1);
  int valid_neighbors = 0;
  for (size_t orig_tet_index = 0; orig_tet_index < tets.size(); orig_tet_index++) {
    auto& tet = tets[orig_tet_index];
    int indices[4];
    bool invalid = false;
    for (size_t i = 0; i < 4; i++) {
      if (tet.v[i] >= particle_indices.size() || tet.v[i] < 0) {
        invalid = true;
        EVOENGINE_LOG("Tetrahedron is invalid");
        break;
      }
      indices[i] = particle_indices[tet.v[i]];
    }
    if (invalid) {
      continue;  // discard this tetrahedron
    }

    // only take tetrahedra that sit between two neighboring planes
    if (!DsAlphaShapeUtils::IsBetweenPlanes(indices, uniform_particles)) {
      continue;
    }

    GpuDelaunayTetrahedron gpu_tet;
    for (size_t i = 0; i < 4; i++) {
      gpu_tet.indices[i] = indices[i];
      gpu_tet.neighbor_tet_ids[i] = -1;
      gpu_tet.is_bark[i] = -1;
    }
    // set up debugging members
    gpu_tet.color = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
    for (int& i : gpu_tet.render_neighbor) {
      i = -1;
    }
    gpu_tet.task_looked_at = 0;
    gpu_tet.mesh_looked_at = 0;
    gpu_tet.inside = 0;
    gpu_tet.triangles_accepted = 0;

    // check orientation of the tetrahedron
    const float d = DsAlphaShapeUtils::PointPlaneDistance(
        uniform_particles[gpu_tet.indices[3]].position, uniform_particles[gpu_tet.indices[0]].position,
        uniform_particles[gpu_tet.indices[1]].position, uniform_particles[gpu_tet.indices[2]].position);

    if (d > 0)  // point no. 3 is in front if the triangle 0 1 2, we need to correct this so the triangles face outwards
    {
      std::swap(gpu_tet.indices[1], gpu_tet.indices[2]);
    }

    // fill in neighbor indices
    for (size_t i = 0; i < 4; i++) {
      // check if neighbor is valid
      if (tet.neighbor_tet_indices[i] >= tets.size() || tet.neighbor_tet_indices[i] < 0) {
        continue;
      }

      auto& neighbor = tets[tet.neighbor_tet_indices[i]];
      int neighbor_indices[4];

      bool invalid = false;
      for (size_t j = 0; j < 4; j++) {
        if (neighbor.v[j] >= particle_indices.size()) {
          invalid = true;
          break;
        }

        neighbor_indices[j] = particle_indices[neighbor.v[j]];
      }

      if (invalid) {
        continue;
      }

      if (!DsAlphaShapeUtils::IsValid(neighbor_indices, uniform_particles.size())) {
        continue;
      }

      if (!DsAlphaShapeUtils::IsBetweenPlanes(neighbor_indices, uniform_particles)) {
        continue;
      }

      // TODO: probably not needed
      const auto mismatch_indices = DsAlphaShapeUtils::CompareIndices(gpu_tet.indices, neighbor_indices);

      gpu_tet.neighbor_tet_ids[mismatch_indices.first] = tet.neighbor_tet_indices[i];
      valid_neighbors++;
    }

    valid_index_map[orig_tet_index] = tetrahedrons.size();
    tetrahedrons.emplace_back(gpu_tet);
  }

  // correct neighbor indices
  Jobs::RunParallelFor(tetrahedrons.size(), [&](const size_t tet_index) {
    auto& tet = tetrahedrons[tet_index];
    for (size_t i = 0; i < 4; i++) {
      if (tet.neighbor_tet_ids[i] == -1) {
        continue;
      }
      tet.neighbor_tet_ids[i] = valid_index_map[tet.neighbor_tet_ids[i]];
    }
  });

  EVOENGINE_LOG("Found " << valid_neighbors << " valid neighbors");
}

void DsAlphaShapeMeshing::ComputeDelaunay(std::vector<GpuDelaunayTetrahedron>& tetrahedrons, bool use_cgal,
                                          size_t min_bundle_size) {
  auto bundle_maps = DsAlphaShapeUtils::ComputeBundleMaps(uniform_particles);

  // TODO: maybe we should scrap CGAL
#ifdef USE_CGAL
  if (use_cgal) {
    std::vector<std::pair<Point_CGAL, unsigned>> points;
    for (auto& map : bundle_maps) {
      for (auto& kv_pair : map) {
        auto& bundle = kv_pair.second;

        if (bundle.size() < min_bundle_size) {
          continue;
        }

        for (size_t i : bundle) {
          auto& particle = uniform_particles[i];
          glm::vec3& particle_pos = particle.position;
          Point_CGAL p_cgal(particle_pos[0], particle_pos[1], particle_pos[2]);
          points.emplace_back(p_cgal, i);
        }
      }
    }

    CGALDelaunay(points, tetrahedrons);
  }
#endif
  if (!use_cgal) {
    std::vector<glm::vec3> points;
    std::vector<size_t> indices;

    for (auto& map : bundle_maps) {
      for (auto& kv_pair : map) {
        auto& bundle = kv_pair.second;

        if (bundle.size() < min_bundle_size) {
          continue;
        }

        for (size_t i : bundle) {
          auto& particle = uniform_particles[i];
          glm::vec3& particle_pos = particle.position;
          points.emplace_back(particle_pos);
          indices.emplace_back(i);
        }
      }
    }

    TetDelaunay(points, indices, tetrahedrons);
  }
}
#pragma endregion

#pragma region registration
void DsAlphaShapeMeshing::RegisterRenderInstances(Handle& rendering_instance_handle, std::shared_ptr<Scene> scene,
                                                  Entity& owner) {
  if (render_settings.branches_render_parameters.solid) {
    RegisterBranchesRenderInstance(rendering_instance_handle, scene, owner);
  }
  if (render_settings.branches_render_parameters.wireframe) {
    RegisterBranchesWireframeRenderInstance(rendering_instance_handle, scene, owner);
  }
  if (render_settings.visualization_rendering) {
    RegisterSmallSegmentsVisualizationRenderInstance(rendering_instance_handle, scene, owner);
  } else {
    RegisterSmallSegmentsRenderInstance(rendering_instance_handle, scene, owner);
  }
}

void DsAlphaShapeMeshing::RegisterBranchesRenderInstance(Handle& rendering_instance_handle,
                                                         std::shared_ptr<Scene> scene, Entity& owner) {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  const auto inner_wood_material = dynamic_strands->materials.inner_wood_material_ref.Get<Material>();
  const auto snow_material = dynamic_strands->materials.snow_material_ref.Get<Material>();
  if (const auto bark_material = dynamic_strands->materials.bark_material_ref.Get<Material>();
      bark_material && inner_wood_material && snow_material) {
    if (!dynamic_strands->segments.empty()) {
      if (branches_point_light_render_pipeline && branches_point_light_render_pipeline->Initialized()) {
        render_layer->RenderToPointLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderBranchesToPointLightShadowMap(render_settings.branches_render_parameters, vk_command_buffer,
                                                     view);
        });
      }
      if (branches_spot_light_render_pipeline && branches_spot_light_render_pipeline->Initialized()) {
        render_layer->RenderToSpotLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderBranchesToSpotLightShadowMap(render_settings.branches_render_parameters, vk_command_buffer,
                                                    view);
        });
      }
      if (branches_directional_light_render_pipeline && branches_directional_light_render_pipeline->Initialized()) {
        render_layer->RenderToDirectionalLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderBranchesToDirectionalLightShadowMap(render_settings.branches_render_parameters,
                                                           vk_command_buffer, view);
        });
      }
      if (branches_render_pipeline && branches_render_pipeline->Initialized()) {
        const auto current_render_storage = Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
        const auto renderer_handle = rendering_instance_handle;
        int bark_material_index = -1;
        current_render_storage->RegisterRenderInstance(scene, owner, renderer_handle, bark_material,
                                                       &bark_material_index);
        const auto inner_material_index = current_render_storage->RegisterMaterial(inner_wood_material);
        const auto snow_material_index = current_render_storage->RegisterMaterial(snow_material);
        render_layer->DeferredRenderingAllCameras(
            [=](const VkCommandBuffer vk_command_buffer,
                const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                const RenderLayer::DeferredRenderingView& view) {
              return RenderBranchesToCameraDeferred(renderer_handle, bark_material_index, inner_material_index,
                                                    snow_material_index, render_settings.branches_render_parameters,
                                                    vk_command_buffer, geometry_pass_color_attachment_infos, view,
                                                    VK_POLYGON_MODE_FILL);
            });
      }
    }
  }
}

void eco_sys_lab_plugin::DsAlphaShapeMeshing::RegisterBranchesWireframeRenderInstance(Handle& rendering_instance_handle,
                                                                                      std::shared_ptr<Scene> scene,
                                                                                      Entity& owner) {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  const auto wireframe_material = dynamic_strands->materials.wireframe_material_ref.Get<Material>();
  if (const auto bark_material = dynamic_strands->materials.bark_material_ref.Get<Material>();
      bark_material && wireframe_material) {
    if (!dynamic_strands->segments.empty()) {
      if (branches_render_pipeline && branches_render_pipeline->Initialized()) {
        const auto current_render_storage = Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
        const auto renderer_handle = mesh_wireframe_rendering_instance_handle;
        // TODO: fix double registration
        current_render_storage->RegisterRenderInstance(scene, owner, renderer_handle, wireframe_material);
        const auto wireframe_material_index = current_render_storage->RegisterMaterial(wireframe_material);
        render_layer->DeferredRenderingAllCameras(
            [=](const VkCommandBuffer vk_command_buffer,
                const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                const RenderLayer::DeferredRenderingView& view) {
              return RenderBranchesToCameraDeferred(renderer_handle, wireframe_material_index, wireframe_material_index,
                                                    wireframe_material_index,
                                                    render_settings.branches_render_parameters, vk_command_buffer,
                                                    geometry_pass_color_attachment_infos, view, VK_POLYGON_MODE_LINE);
            });
      }
    }
  }
}

void DsAlphaShapeMeshing::RegisterSmallSegmentsRenderInstance(Handle& rendering_instance_handle,
                                                              std::shared_ptr<Scene> scene, Entity& owner) {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  const auto bark_material = dynamic_strands->materials.bark_material_ref.Get<Material>();
  if (const auto splinter_material = dynamic_strands->materials.splinter_material_ref.Get<Material>();
      bark_material && splinter_material) {
    if (!dynamic_strands->segments.empty()) {
      if (small_segments_point_light_render_pipeline && small_segments_point_light_render_pipeline->Initialized()) {
        render_layer->RenderToPointLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSmallSegmentsToPointLightShadowMap(render_settings.small_segments_render_parameters,
                                                          vk_command_buffer, view);
        });
      }
      if (small_segments_spot_light_render_pipeline && small_segments_spot_light_render_pipeline->Initialized()) {
        render_layer->RenderToSpotLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSmallSegmentsToSpotLightShadowMap(render_settings.small_segments_render_parameters,
                                                         vk_command_buffer, view);
        });
      }
      if (small_segments_directional_light_render_pipeline &&
          small_segments_directional_light_render_pipeline->Initialized()) {
        render_layer->RenderToDirectionalLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSmallSegmentsToDirectionalLightShadowMap(render_settings.small_segments_render_parameters,
                                                                vk_command_buffer, view);
        });
      }

      if (small_segments_render_pipeline && small_segments_render_pipeline->Initialized()) {
        const auto current_render_storage = Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
        const auto renderer_handle = small_segments_rendering_instance_handle;
        current_render_storage->RegisterRenderInstance(scene, owner, renderer_handle, bark_material);
        const auto splinter_material_index = current_render_storage->RegisterMaterial(splinter_material);
        render_layer->DeferredRenderingAllCameras(
            [=](const VkCommandBuffer vk_command_buffer,
                const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                const RenderLayer::DeferredRenderingView& view) {
              return RenderSmallSegmentsToCameraDeferred(renderer_handle, splinter_material_index,
                                                         render_settings.small_segments_render_parameters,
                                                         vk_command_buffer, geometry_pass_color_attachment_infos, view);
            });
      }
    }
  }
}

void DsAlphaShapeMeshing::RegisterSmallSegmentsVisualizationRenderInstance(Handle& rendering_instance_handle,
                                                                           std::shared_ptr<Scene> scene,
                                                                           Entity& owner) {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  if (const auto material = dynamic_strands->materials.splinter_material_ref.Get<Material>()) {
    if (!dynamic_strands->segments.empty()) {
      if (small_segments_point_light_render_pipeline && small_segments_point_light_render_pipeline->Initialized()) {
        render_layer->RenderToPointLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSmallSegmentsToPointLightShadowMap(render_settings.small_segments_render_parameters,
                                                          vk_command_buffer, view);
        });
      }
      if (small_segments_spot_light_render_pipeline && small_segments_spot_light_render_pipeline->Initialized()) {
        render_layer->RenderToSpotLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSmallSegmentsToSpotLightShadowMap(render_settings.small_segments_render_parameters,
                                                         vk_command_buffer, view);
        });
      }
      if (small_segments_directional_light_render_pipeline &&
          small_segments_directional_light_render_pipeline->Initialized()) {
        render_layer->RenderToDirectionalLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSmallSegmentsToDirectionalLightShadowMap(render_settings.small_segments_render_parameters,
                                                                vk_command_buffer, view);
        });
      }
      if (small_segments_visualization_render_pipeline && small_segments_visualization_render_pipeline->Initialized()) {
        const auto current_render_storage = Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
        const auto renderer_handle = small_segments_rendering_instance_handle;
        current_render_storage->RegisterRenderInstance(scene, owner, renderer_handle, material);

        DynamicStrandsInitializeParameters initialize_parameters;  // I suppose this can be empty?!

        render_layer->DeferredRenderingAllCameras(
            [=](const VkCommandBuffer vk_command_buffer,
                const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                const RenderLayer::DeferredRenderingView& view) {
              return RenderSmallSegmentsVisualizationToCameraDeferred(
                  renderer_handle, initialize_parameters,
                  render_settings.small_segments_visualization_render_parameters, vk_command_buffer,
                  geometry_pass_color_attachment_infos, view);
            });
      }
    }
  }
}
#pragma endregion