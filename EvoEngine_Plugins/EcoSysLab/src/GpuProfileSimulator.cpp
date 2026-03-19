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

  const auto max_frames = Platform::GetMaxFramesInFlight();
  descriptor_sets_.resize(max_frames);
  for (auto& ds : descriptor_sets_) {
    ds = std::make_shared<DescriptorSet>(profile_packing_layout_);
  }

  initialized_ = true;
}

void GpuProfileSimulator::UploadFromSkeleton(ProceduralStrandModelSkeleton& skeleton,
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

void GpuProfileSimulator::DownloadToSkeleton(ProceduralStrandModelSkeleton& skeleton) {
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
