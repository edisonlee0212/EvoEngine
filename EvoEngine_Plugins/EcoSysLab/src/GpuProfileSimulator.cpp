#include "GpuProfileSimulator.hpp"

#include "Shader.hpp"

using namespace eco_sys_lab_plugin;

void GpuProfileSimulator::Init() {
  if (initialized_)
    return;

  // Create shared descriptor set layout: 3 SSBOs (particles, profiles, grid).
  if (!profile_packing_layout_) {
    profile_packing_layout_ = std::make_shared<DescriptorSetLayout>();
    profile_packing_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT,
                                                   0);
    profile_packing_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT,
                                                   0);
    profile_packing_layout_->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT,
                                                   0);
    profile_packing_layout_->Initialize();
  }

  // Helper: create a compute pipeline from a shader file.
  // Returns true on success; on failure the pipeline pointer remains null so callers can detect it.
  auto create_pipeline = [](const std::filesystem::path& shader_path, std::shared_ptr<ComputePipeline>& pipeline,
                            const uint32_t push_constant_size) -> bool {
    if (pipeline)
      return true;  // Already initialised on a previous call.
    auto shader = std::make_shared<Shader>();
    if (!shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(), shader_path)) {
      EVOENGINE_ERROR("GpuProfileSimulator: failed to compile " + shader_path.string());
      return false;
    }
    // Build into a temporary first so `pipeline` is only assigned on full success.
    auto new_pipeline = std::make_shared<ComputePipeline>();
    new_pipeline->compute_shader = shader;
    new_pipeline->descriptor_set_layouts.emplace_back(profile_packing_layout_);
    auto& range = new_pipeline->push_constant_ranges.emplace_back();
    range.size = push_constant_size;
    range.offset = 0;
    range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    new_pipeline->Initialize();
    pipeline = new_pipeline;
    return true;
  };

  const auto base = std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/ProfilePacking";
  bool all_ok = true;
  all_ok &= create_pipeline(base / "ApplyForces.comp",    apply_forces_pipeline_,    sizeof(ApplyForcesPushConstant));
  all_ok &= create_pipeline(base / "ClearGrid.comp",      clear_grid_pipeline_,      sizeof(ClearGridPushConstant));
  all_ok &= create_pipeline(base / "BuildGrid.comp",      build_grid_pipeline_,      sizeof(BuildGridPushConstant));
  all_ok &= create_pipeline(base / "SolveCollisions.comp",solve_collisions_pipeline_,sizeof(SolveCollisionsPushConstant));
  all_ok &= create_pipeline(base / "VerletUpdate.comp",   verlet_update_pipeline_,   sizeof(VerletUpdatePushConstant));
  if (!all_ok) {
    EVOENGINE_ERROR("GpuProfileSimulator: one or more shaders failed — GPU profile packing disabled.");
    return;  // initialized_ stays false; Simulate/Upload will skip gracefully.
  }

  // Create GPU buffers with initial placeholder size.
  VkBufferCreateInfo buf_info{};
  buf_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buf_info.usage =
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buf_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buf_info.size = 1;
  VmaAllocationCreateInfo alloc_info{};
  alloc_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

  particles_buffer_ = std::make_shared<Buffer>(buf_info, alloc_info);
  profiles_buffer_ = std::make_shared<Buffer>(buf_info, alloc_info);
  grid_buffer_ = std::make_shared<Buffer>(buf_info, alloc_info);
  skeleton_nodes_buffer_ = std::make_shared<Buffer>(buf_info, alloc_info);

  const auto max_frames = Platform::GetMaxFramesInFlight();
  descriptor_sets_.resize(max_frames);
  for (auto& ds : descriptor_sets_) {
    ds = std::make_shared<DescriptorSet>(profile_packing_layout_);
  }

  initialized_ = true;
}

template <typename SkeletonT>
void GpuProfileSimulator::UploadFromSkeleton(SkeletonT& skeleton,
                                             const std::vector<SkeletonNodeHandle>& active_nodes,
                                             const bool freeze_interior) {
  if (!initialized_)
    Init();
  if (!initialized_)
    return;  // Init failed (e.g. shader compile error) — bail out safely.

  cpu_particles_.clear();
  cpu_profiles_.clear();
  profile_to_node_.clear();
  total_grid_cells_ = 0;

  const auto& sorted_list = skeleton.PeekSortedNodeList();
  const std::set<SkeletonNodeHandle> active_set(active_nodes.begin(), active_nodes.end());

  constexpr int max_grid_side = 64;

  for (const auto node_handle : sorted_list) {
    auto& node = skeleton.RefNode(node_handle);
    if (!node.data.HasStrandData())
      continue;

    auto& profile = node.data.strand_data->profile;
    const auto& particles = profile.PeekParticles();
    if (particles.empty())
      continue;

    GpuProfileInfo info{};
    info.particle_offset = static_cast<uint32_t>(cpu_particles_.size());
    info.particle_count = static_cast<uint32_t>(particles.size());
    info.is_active = active_set.count(node_handle) ? 1 : 0;

    // Compute particle extents for grid bounds.
    glm::vec2 pmin(FLT_MAX);
    glm::vec2 pmax(-FLT_MAX);
    for (const auto& p : particles) {
      const auto pos = p.GetPosition();
      pmin = glm::min(pmin, pos);
      pmax = glm::max(pmax, pos);
    }

    info.grid_cell_size = 2.0f;  // matches CPU grid cell size (particle diameter)
    info.grid_min_bound = pmin - glm::vec2(2.0f);
    const glm::vec2 grid_max = pmax + glm::vec2(2.0f);
    info.grid_resolution_x =
        glm::clamp(static_cast<int>(glm::ceil((grid_max.x - info.grid_min_bound.x) / info.grid_cell_size)), 1,
                    max_grid_side);
    info.grid_resolution_y =
        glm::clamp(static_cast<int>(glm::ceil((grid_max.y - info.grid_min_bound.y) / info.grid_cell_size)), 1,
                    max_grid_side);

    info.grid_offset = total_grid_cells_;
    total_grid_cells_ += static_cast<uint32_t>(info.grid_resolution_x * info.grid_resolution_y);

    info.particle_softness = profile.particle_physics_settings.particle_softness;
    info.damping = profile.particle_physics_settings.damping;
    info.max_speed = profile.particle_physics_settings.max_speed;

    cpu_profiles_.push_back(info);
    profile_to_node_.push_back(node_handle);

    // Flatten particles into the global buffer.
    for (const auto& p : particles) {
      GpuProfileParticle gp{};
      gp.position = p.GetPosition();
      gp.last_position = gp.position;
      gp.acceleration = glm::vec2(0.0f);
      gp.delta_position = glm::vec2(0.0f);
      gp.enable = !p.IsEnabled() ? 0 : ((freeze_interior && info.is_active && !p.IsBoundary()) ? 2 : static_cast<uint32_t>(p.status));
      gp.strand_handle = p.strand_handle;
      gp.strand_segment_handle = p.strand_segment_handle;
      gp.node_handle = node_handle;
      gp.birth_step = static_cast<uint32_t>(p.data.birth_step);
      cpu_particles_.push_back(gp);
    }
  }

  total_particles_ = static_cast<uint32_t>(cpu_particles_.size());
  total_profiles_ = static_cast<uint32_t>(cpu_profiles_.size());

  if (total_particles_ == 0)
    return;

  // Upload to GPU.
  particles_buffer_->UploadVector(cpu_particles_);
  profiles_buffer_->UploadVector(cpu_profiles_);

  // Allocate and upload empty grid cells.
  if (total_grid_cells_ > 0) {
    std::vector<GpuGridCell> empty_grid(total_grid_cells_);
    grid_buffer_->UploadVector(empty_grid);
  }

  // Update ALL per-frame descriptor sets so the binding stays valid regardless of
  // which frame index is current when the next CopyFromBuffer or descriptor write happens.
  for (auto& ds : descriptor_sets_) {
    ds->UpdateBufferDescriptorBinding(0, particles_buffer_);
    ds->UpdateBufferDescriptorBinding(1, profiles_buffer_);
    ds->UpdateBufferDescriptorBinding(2, grid_buffer_);
  }
}

void GpuProfileSimulator::Simulate(const uint32_t iterations, const StrandModelParameters& params) {
  if (total_particles_ == 0 || !initialized_)
    return;
  // Guard: pipelines might be null if shaders failed to compile.
  if (!apply_forces_pipeline_ || !clear_grid_pipeline_ || !build_grid_pipeline_ ||
      !solve_collisions_pipeline_ || !verlet_update_pipeline_)
    return;

  const uint32_t wg = Platform::Constants::compute_work_group_invocations;

  // Use descriptor set 0 — all sets were updated by UploadFromSkeleton to point to the
  // same current buffers, so any slot is equivalent.
  const VkDescriptorSet ds = descriptor_sets_[0]->GetVkDescriptorSet();

  // Run all iterations inside a single ImmediateSubmit so the GPU finishes
  // before DownloadToSkeleton reads the results back.
  Platform::ImmediateSubmit([&](const VkCommandBuffer cmd) {
    for (uint32_t iter = 0; iter < iterations; iter++) {
      // Pass 1: Apply center-attraction forces.
      {
        ApplyForcesPushConstant pc{};
        pc.total_particles = total_particles_;
        pc.total_profiles = total_profiles_;
        pc.center_attraction_strength = params.center_attraction_strength;
        apply_forces_pipeline_->Bind(cmd);
        apply_forces_pipeline_->BindDescriptorSet(cmd, 0, ds);
        apply_forces_pipeline_->PushConstant(cmd, 0, pc);
        vkCmdDispatch(cmd, Platform::DivUp(total_particles_, wg), 1, 1);
        Platform::EverythingBarrier(cmd);
      }

      // Pass 2: Clear grid cells.
      {
        ClearGridPushConstant pc{};
        pc.total_grid_cells = total_grid_cells_;
        clear_grid_pipeline_->Bind(cmd);
        clear_grid_pipeline_->BindDescriptorSet(cmd, 0, ds);
        clear_grid_pipeline_->PushConstant(cmd, 0, pc);
        vkCmdDispatch(cmd, Platform::DivUp(total_grid_cells_, wg), 1, 1);
        Platform::EverythingBarrier(cmd);
      }

      // Pass 3: Build spatial hash grid.
      {
        BuildGridPushConstant pc{};
        pc.total_particles = total_particles_;
        pc.total_profiles = total_profiles_;
        build_grid_pipeline_->Bind(cmd);
        build_grid_pipeline_->BindDescriptorSet(cmd, 0, ds);
        build_grid_pipeline_->PushConstant(cmd, 0, pc);
        vkCmdDispatch(cmd, Platform::DivUp(total_particles_, wg), 1, 1);
        Platform::EverythingBarrier(cmd);
      }

      // Pass 4: Solve collisions.
      {
        SolveCollisionsPushConstant pc{};
        pc.total_particles = total_particles_;
        pc.total_profiles = total_profiles_;
        solve_collisions_pipeline_->Bind(cmd);
        solve_collisions_pipeline_->BindDescriptorSet(cmd, 0, ds);
        solve_collisions_pipeline_->PushConstant(cmd, 0, pc);
        vkCmdDispatch(cmd, Platform::DivUp(total_particles_, wg), 1, 1);
        Platform::EverythingBarrier(cmd);
      }

      // Pass 5: Verlet integration.
      {
        VerletUpdatePushConstant pc{};
        pc.total_particles = total_particles_;
        pc.total_profiles = total_profiles_;
        pc.dt = 0.001f;  // matches CPU delta_time_
        pc.damping = params.profile_physics_settings.damping;
        pc.max_speed = params.profile_physics_settings.max_speed;
        verlet_update_pipeline_->Bind(cmd);
        verlet_update_pipeline_->BindDescriptorSet(cmd, 0, ds);
        verlet_update_pipeline_->PushConstant(cmd, 0, pc);
        vkCmdDispatch(cmd, Platform::DivUp(total_particles_, wg), 1, 1);
        Platform::EverythingBarrier(cmd);
      }
    }
  });
}

template <typename SkeletonT>
void GpuProfileSimulator::DownloadToSkeleton(SkeletonT& skeleton) {
  if (total_particles_ == 0 || !initialized_)
    return;

  // Download particle buffer from GPU.
  std::vector<GpuProfileParticle> result;
  particles_buffer_->DownloadVector(result, total_particles_);

  // Write positions back into CPU profiles.
  for (uint32_t pi = 0; pi < total_profiles_; pi++) {
    const auto& info = cpu_profiles_[pi];
    const auto node_handle = profile_to_node_[pi];
    auto& node = skeleton.RefNode(node_handle);
    if (!node.data.HasStrandData())
      continue;

    auto& profile_particles = node.data.strand_data->profile.RefParticles();
    const uint32_t count = glm::min(info.particle_count, static_cast<uint32_t>(profile_particles.size()));
    for (uint32_t j = 0; j < count; j++) {
      const auto& gpu_p = result[info.particle_offset + j];
      profile_particles[j].SetPosition(gpu_p.position);
    }
  }
}

void GpuProfileSimulator::UploadSkeleton(const DevelopmentalStrandModelSkeleton& skeleton,
                                         const StrandModelParameters& params) {
  if (!initialized_)
    Init();
  if (!initialized_)
    return;

  cpu_skeleton_nodes_.clear();

  const auto& sorted_list = skeleton.PeekSortedNodeList();
  if (sorted_list.empty())
    return;

  // Build reverse mapping: node_handle → profile_index.
  std::unordered_map<SkeletonNodeHandle, uint32_t> node_to_profile;
  for (uint32_t pi = 0; pi < static_cast<uint32_t>(profile_to_node_.size()); pi++) {
    node_to_profile[profile_to_node_[pi]] = pi;
  }

  // Compute max_root_distance for strand_radius_distribution normalisation.
  float max_root_distance = 0.0f;
  for (const auto& node_handle : sorted_list) {
    const auto& node = skeleton.PeekNode(node_handle);
    max_root_distance = glm::max(max_root_distance, node.info.root_distance + node.info.length);
  }
  if (max_root_distance < glm::epsilon<float>())
    max_root_distance = 1.0f;

  // We must produce a dense array indexed by node_handle, so find the max handle.
  int max_handle = 0;
  for (const auto& nh : sorted_list) {
    max_handle = glm::max(max_handle, nh);
  }
  cpu_skeleton_nodes_.resize(static_cast<size_t>(max_handle) + 1);

  for (const auto& node_handle : sorted_list) {
    const auto& node = skeleton.PeekNode(node_handle);
    auto& gpu_node = cpu_skeleton_nodes_[node_handle];

    gpu_node.global_position = node.info.global_position;
    gpu_node.length = node.info.length;

    const auto& q = node.info.regulated_global_rotation;
    gpu_node.regulated_global_rotation = glm::vec4(q.x, q.y, q.z, q.w);

    gpu_node.global_end_position = node.info.GetGlobalEndPosition();

    // Compute strand_radius from distribution mean (deterministic, same formula as ApplyProfiles).
    gpu_node.strand_radius =
        glm::max(0.0f, params.strand_radius_distribution.mean.GetValue(node.info.root_distance / max_root_distance));

    gpu_node.root_distance = node.info.root_distance;
    gpu_node.max_root_distance = max_root_distance;
    gpu_node.parent_handle = node.GetParentHandle();
    gpu_node.is_end_node = node.IsEndNode() ? 1 : 0;

    // Profile index for surface mesh generation (node_handle → profile mapping).
    auto it = node_to_profile.find(node_handle);
    gpu_node.profile_index = (it != node_to_profile.end()) ? it->second : 0xFFFFFFFF;

    if (node.data.HasStrandData()) {
      const auto& profile = node.data.strand_data->profile;
      uint32_t boundary_count = 0;
      for (const auto& p : profile.PeekParticles()) {
        if (p.IsBoundary())
          ++boundary_count;
      }
      gpu_node.boundary_particle_count = boundary_count;
      gpu_node.total_particle_count = static_cast<uint32_t>(profile.PeekParticles().size());
    } else {
      gpu_node.boundary_particle_count = 0;
      gpu_node.total_particle_count = 0;
    }
  }

  total_skeleton_nodes_ = static_cast<uint32_t>(cpu_skeleton_nodes_.size());

  // Upload to GPU.
  skeleton_nodes_buffer_->UploadVector(cpu_skeleton_nodes_);
}

// ---------------------------------------------------------------------------
// Surface mesh generation — lazy initialization
// ---------------------------------------------------------------------------

void GpuProfileSimulator::InitSurfaceMesh() {
  if (surface_mesh_initialized_)
    return;

  // Descriptor set layout: 7 SSBOs for surface mesh generation.
  if (!surface_mesh_layout_) {
    surface_mesh_layout_ = std::make_shared<DescriptorSetLayout>();
    for (uint32_t i = 0; i < 7; i++) {
      surface_mesh_layout_->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    }
    surface_mesh_layout_->Initialize();
  }

  // Create compute pipelines for the two surface mesh passes.
  auto create_sm_pipeline = [](const std::filesystem::path& shader_path, std::shared_ptr<ComputePipeline>& pipeline,
                               const uint32_t push_constant_size) -> bool {
    if (pipeline)
      return true;
    auto shader = std::make_shared<Shader>();
    if (!shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(), shader_path)) {
      EVOENGINE_ERROR("GpuProfileSimulator: failed to compile " + shader_path.string());
      return false;
    }
    auto new_pipeline = std::make_shared<ComputePipeline>();
    new_pipeline->compute_shader = shader;
    new_pipeline->descriptor_set_layouts.emplace_back(surface_mesh_layout_);
    auto& range = new_pipeline->push_constant_ranges.emplace_back();
    range.size = push_constant_size;
    range.offset = 0;
    range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    new_pipeline->Initialize();
    pipeline = new_pipeline;
    return true;
  };

  const auto base = std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/SurfaceMesh";
  bool all_ok = true;
  all_ok &= create_sm_pipeline(base / "ExtractAndProject.comp", extract_project_pipeline_, sizeof(ExtractProjectPushConstant));
  all_ok &= create_sm_pipeline(base / "StitchContours.comp", stitch_contours_pipeline_, sizeof(StitchContoursPushConstant));

  if (!all_ok) {
    EVOENGINE_ERROR("GpuProfileSimulator: surface mesh shaders failed — GPU surface mesh disabled.");
    return;
  }

  // Create GPU buffers for surface mesh generation.
  VkBufferCreateInfo buf_info{};
  buf_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buf_info.usage =
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buf_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buf_info.size = 1;
  VmaAllocationCreateInfo alloc_info{};
  alloc_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

  contour_info_buffer_ = std::make_shared<Buffer>(buf_info, alloc_info);
  counters_buffer_ = std::make_shared<Buffer>(buf_info, alloc_info);

  // Vertex and index buffers also need acceleration structure build flags for Phase 3.
  buf_info.usage |= VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
                    VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
  vertex_buffer_ = std::make_shared<Buffer>(buf_info, alloc_info);
  index_buffer_ = std::make_shared<Buffer>(buf_info, alloc_info);

  const auto max_frames = Platform::GetMaxFramesInFlight();
  surface_mesh_descriptor_sets_.resize(max_frames);
  for (auto& ds : surface_mesh_descriptor_sets_) {
    ds = std::make_shared<DescriptorSet>(surface_mesh_layout_);
  }

  surface_mesh_initialized_ = true;
}

// ---------------------------------------------------------------------------
// Surface mesh generation — two-pass compute dispatch
// ---------------------------------------------------------------------------

void GpuProfileSimulator::GenerateSurfaceMesh() {
  if (!initialized_ || total_profiles_ == 0 || total_skeleton_nodes_ == 0)
    return;

  InitSurfaceMesh();
  if (!surface_mesh_initialized_)
    return;

  if (!extract_project_pipeline_ || !stitch_contours_pipeline_)
    return;

  // Conservative upper bounds for buffer sizing.
  // Synthetic ring fallback emits up to SYNTH_RING_VERTS (8) per profile even for
  // single-particle profiles, so account for both real and synthetic vertices.
  const uint32_t synth_ring_verts = 8;  // must match SYNTH_RING_VERTS in ExtractAndProject.comp
  const uint32_t max_vertices = glm::max(total_particles_, total_profiles_ * synth_ring_verts);
  // At branching nodes all children stitch to the full parent contour, so the parent's
  // boundary count is counted once per child.  Use a generous multiplier.
  const uint32_t max_triangles = 8 * max_vertices;

  // Ensure contour_info and counters buffers are large enough.
  // UploadVector/Upload auto-resize via Buffer::Resize. We also zero them inside
  // the compute command buffer (vkCmdFillBuffer) to avoid cross-submit sync issues,
  // but the upload ensures the VkBuffer is the right size for descriptor binding.
  {
    std::vector<GpuContourInfo> empty_contour_infos(total_profiles_);
    contour_info_buffer_->UploadVector(empty_contour_infos);
    GpuMeshCounters zero_counters{0, 0};
    counters_buffer_->Upload(zero_counters);
  }

  // Allocate vertex/index buffers large enough for the output.
  // Re-create with correct usage flags and size.
  {
    VkBufferCreateInfo buf_info{};
    buf_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buf_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                     VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
                     VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
    buf_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VmaAllocationCreateInfo alloc_info{};
    alloc_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

    buf_info.size = static_cast<VkDeviceSize>(max_vertices) * sizeof(Vertex);
    if (buf_info.size < 4)
      buf_info.size = 4;
    vertex_buffer_ = std::make_shared<Buffer>(buf_info, alloc_info);

    buf_info.size = static_cast<VkDeviceSize>(max_triangles) * 3 * sizeof(uint32_t);  // flat uint array (no uvec3 padding)
    if (buf_info.size < 4)
      buf_info.size = 4;
    index_buffer_ = std::make_shared<Buffer>(buf_info, alloc_info);
  }

  // Update descriptor set bindings.
  for (auto& ds : surface_mesh_descriptor_sets_) {
    ds->UpdateBufferDescriptorBinding(0, particles_buffer_);
    ds->UpdateBufferDescriptorBinding(1, profiles_buffer_);
    ds->UpdateBufferDescriptorBinding(2, skeleton_nodes_buffer_);
    ds->UpdateBufferDescriptorBinding(3, contour_info_buffer_);
    ds->UpdateBufferDescriptorBinding(4, vertex_buffer_);
    ds->UpdateBufferDescriptorBinding(5, index_buffer_);
    ds->UpdateBufferDescriptorBinding(6, counters_buffer_);
  }

  const VkDescriptorSet ds = surface_mesh_descriptor_sets_[0]->GetVkDescriptorSet();

  Platform::ImmediateSubmit([&](const VkCommandBuffer cmd) {
    // Zero the counters and contour_info buffers INSIDE the command buffer
    // to guarantee they are visible before the compute dispatches.
    vkCmdFillBuffer(cmd, counters_buffer_->GetVkBuffer(), 0, sizeof(GpuMeshCounters), 0);
    vkCmdFillBuffer(cmd, contour_info_buffer_->GetVkBuffer(), 0,
                    static_cast<VkDeviceSize>(total_profiles_) * sizeof(GpuContourInfo), 0);
    Platform::EverythingBarrier(cmd);

    // Pass 1: Extract boundary contours, sort by angle, project 2D→3D.
    // Dispatch one workgroup per profile.
    {
      ExtractProjectPushConstant pc{};
      pc.total_profiles = total_profiles_;
      pc.max_vertices = max_vertices;
      extract_project_pipeline_->Bind(cmd);
      extract_project_pipeline_->BindDescriptorSet(cmd, 0, ds);
      extract_project_pipeline_->PushConstant(cmd, 0, pc);
      vkCmdDispatch(cmd, total_profiles_, 1, 1);
      Platform::EverythingBarrier(cmd);
    }

    // Pass 2: Stitch adjacent contours into triangle strips.
    // Dispatch one workgroup per profile (non-root profiles will produce triangles).
    {
      StitchContoursPushConstant pc{};
      pc.total_profiles = total_profiles_;
      pc.max_triangles = max_triangles;
      stitch_contours_pipeline_->Bind(cmd);
      stitch_contours_pipeline_->BindDescriptorSet(cmd, 0, ds);
      stitch_contours_pipeline_->PushConstant(cmd, 0, pc);
      vkCmdDispatch(cmd, total_profiles_, 1, 1);
      Platform::EverythingBarrier(cmd);
    }
  });

  // Read back counters to know the actual mesh size.
  GpuMeshCounters result_counters{};
  counters_buffer_->Download(result_counters);

  // Clamp to allocated capacity (shader bounds-checks should prevent overflows,
  // but the atomic counters still accumulate beyond the limit).
  surface_vertex_count_ = glm::min(result_counters.total_vertices, max_vertices);
  surface_triangle_count_ = glm::min(result_counters.total_triangles, max_triangles);

  if (result_counters.total_vertices > max_vertices || result_counters.total_triangles > max_triangles) {
    EVOENGINE_WARNING("GPU surface mesh overflow — vertices: " + std::to_string(result_counters.total_vertices) +
                      "/" + std::to_string(max_vertices) + ", triangles: " +
                      std::to_string(result_counters.total_triangles) + "/" + std::to_string(max_triangles));
  }

  if (surface_vertex_count_ > 0 && surface_triangle_count_ > 0) {
    EVOENGINE_LOG("GPU surface mesh: " + std::to_string(surface_vertex_count_) + " vertices, " +
                  std::to_string(surface_triangle_count_) + " triangles");
  }
}

bool GpuProfileSimulator::DownloadSurfaceMesh(std::vector<Vertex>& vertices,
                                              std::vector<glm::uvec3>& triangles) const {
  if (surface_vertex_count_ == 0 || surface_triangle_count_ == 0 || !vertex_buffer_ || !index_buffer_)
    return false;

  vertex_buffer_->DownloadVector(vertices, surface_vertex_count_);

  // Download flat uint index data (GPU SSBO is uint[], not uvec3[], to avoid std430 padding).
  std::vector<uint32_t> flat_indices;
  index_buffer_->DownloadVector(flat_indices, surface_triangle_count_ * 3);
  triangles.resize(surface_triangle_count_);
  for (uint32_t i = 0; i < surface_triangle_count_; i++) {
    triangles[i] = glm::uvec3(flat_indices[i * 3 + 0], flat_indices[i * 3 + 1], flat_indices[i * 3 + 2]);
  }

  // --- Diagnostic validation (remove once mesh looks correct) ---
  uint32_t nan_count = 0;
  glm::vec3 bbox_min(FLT_MAX), bbox_max(-FLT_MAX);
  for (uint32_t i = 0; i < vertices.size(); i++) {
    const auto& v = vertices[i];
    if (glm::any(glm::isnan(v.position)) || glm::any(glm::isinf(v.position))) {
      nan_count++;
      continue;
    }
    bbox_min = glm::min(bbox_min, v.position);
    bbox_max = glm::max(bbox_max, v.position);
  }
  uint32_t bad_tri_count = 0;
  std::string first_bad;
  for (uint32_t ti = 0; ti < triangles.size(); ti++) {
    const auto& tri = triangles[ti];
    if (tri.x >= surface_vertex_count_ || tri.y >= surface_vertex_count_ || tri.z >= surface_vertex_count_) {
      bad_tri_count++;
      if (first_bad.empty()) {
        first_bad = " first_bad[" + std::to_string(ti) + "]=(" + std::to_string(tri.x) + "," +
                    std::to_string(tri.y) + "," + std::to_string(tri.z) + ")";
      }
    }
  }

  // Also validate contour_infos for offset correctness.
  std::string contour_diag;
  if (contour_info_buffer_ && total_profiles_ > 0) {
    std::vector<GpuContourInfo> ci_data;
    contour_info_buffer_->DownloadVector(ci_data, total_profiles_);
    uint32_t bad_ci = 0;
    for (uint32_t pi = 0; pi < ci_data.size(); pi++) {
      const auto& ci = ci_data[pi];
      if (ci.vertex_count > 0 && ci.vertex_offset + ci.vertex_count > surface_vertex_count_)
        bad_ci++;
    }
    contour_diag = ", " + std::to_string(bad_ci) + " bad-contour-infos";
    if (bad_ci > 0 && !ci_data.empty()) {
      contour_diag += " ci[0]=(off=" + std::to_string(ci_data[0].vertex_offset) +
                      ",cnt=" + std::to_string(ci_data[0].vertex_count) +
                      ",node=" + std::to_string(ci_data[0].node_handle) + ")";
    }
  }

  EVOENGINE_LOG("GPU mesh diag: " + std::to_string(vertices.size()) + " verts, " +
                std::to_string(triangles.size()) + " tris, " + std::to_string(nan_count) + " NaN verts, " +
                std::to_string(bad_tri_count) + " bad-index tris" + first_bad + contour_diag + ", bbox=(" +
                std::to_string(bbox_min.x) + "," + std::to_string(bbox_min.y) + "," + std::to_string(bbox_min.z) +
                ") to (" + std::to_string(bbox_max.x) + "," + std::to_string(bbox_max.y) + "," +
                std::to_string(bbox_max.z) + ")");

  return true;
}

// ---------------------------------------------------------------------------
// Explicit template instantiations
// ---------------------------------------------------------------------------
template void GpuProfileSimulator::UploadFromSkeleton(DevelopmentalStrandModelSkeleton&,
                                                      const std::vector<SkeletonNodeHandle>&, bool);
template void GpuProfileSimulator::UploadFromSkeleton(RootDevelopmentalStrandModelSkeleton&,
                                                      const std::vector<SkeletonNodeHandle>&, bool);
template void GpuProfileSimulator::DownloadToSkeleton(DevelopmentalStrandModelSkeleton&);
template void GpuProfileSimulator::DownloadToSkeleton(RootDevelopmentalStrandModelSkeleton&);
