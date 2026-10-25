#include "WindParticleField.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "Application.hpp"
#include "Blower.hpp"
#include "Climate.hpp"
#include "GpuTreePhysics.hpp"
#include "RenderLayer.hpp"
#include "RenderTexture.hpp"
#include "Shader.hpp"

using namespace eco_sys_lab_package;

bool WindParticleField::Prepare() {
  if (render_pipeline_)
    return advect_pipeline_->Initialized() && emit_pipeline_->Initialized() && clear_grid_pipeline_->Initialized() &&
           grid_pipeline_->Initialized() && air_update_pipeline_->Initialized() && trace_pipeline_->Initialized() &&
           clear_rods_pipeline_->Initialized() && build_rods_pipeline_->Initialized() &&
           rod_air_pipeline_->Initialized() && render_pipeline_->Initialized() && grid_render_pipeline_->Initialized();
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer)
    return false;
  layout_ = std::make_shared<DescriptorSetLayout>();
  layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                 VK_SHADER_STAGE_COMPUTE_BIT | VK_SHADER_STAGE_VERTEX_BIT, 0);
  layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout_->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout_->Initialize();

  VkBufferCreateInfo buffer_info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  buffer_info.size = capacity * sizeof(Particle);
  buffer_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation_info{};
  allocation_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  particles_ = std::make_shared<Buffer>(buffer_info, allocation_info);
  particles_->UploadVector(std::vector<Particle>(capacity));
  buffer_info.size = grid_bucket_count * sizeof(uint32_t);
  grid_heads_ = std::make_shared<Buffer>(buffer_info, allocation_info);
  buffer_info.size = capacity * sizeof(uint32_t);
  grid_next_ = std::make_shared<Buffer>(buffer_info, allocation_info);
  buffer_info.size = grid_bucket_count * sizeof(uint32_t);
  rod_heads_ = std::make_shared<Buffer>(buffer_info, allocation_info);
  buffer_info.size = sizeof(uint32_t);
  rod_next_ = std::make_shared<Buffer>(buffer_info, allocation_info);
  rod_max_extent_ = std::make_shared<Buffer>(buffer_info, allocation_info);
  descriptor_set_ = std::make_shared<DescriptorSet>(layout_);
  descriptor_set_->UpdateBufferDescriptorBinding(0, particles_);
  descriptor_set_->UpdateBufferDescriptorBinding(1, grid_heads_);
  descriptor_set_->UpdateBufferDescriptorBinding(2, grid_next_);
  air_layout_ = std::make_shared<DescriptorSetLayout>();
  air_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  air_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  air_layout_->Initialize();
  grid_render_layout_ = std::make_shared<DescriptorSetLayout>();
  grid_render_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_VERTEX_BIT, 0);
  grid_render_layout_->Initialize();
  air_update_layout_ = std::make_shared<DescriptorSetLayout>();
  for (uint32_t i = 0; i < 5; ++i)
    air_update_layout_->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  air_update_layout_->Initialize();
  trace_layout_ = std::make_shared<DescriptorSetLayout>();
  for (uint32_t i = 0; i < 3; ++i)
    trace_layout_->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  trace_layout_->Initialize();
  rod_layout_ = std::make_shared<DescriptorSetLayout>();
  for (uint32_t i = 0; i < 6; ++i)
    rod_layout_->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  rod_layout_->Initialize();
  rod_frames_.resize(Platform::GetMaxFramesInFlight());

  const auto compute_shader = [](const char* path) {
    return Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                   std::filesystem::path("./EcoSysLabResources/Shaders/Compute/Wind") / path);
  };
  advect_pipeline_ = std::make_shared<ComputePipeline>();
  advect_pipeline_->compute_shader = compute_shader("Advect.slang");
  advect_pipeline_->descriptor_set_layouts = {layout_};
  advect_pipeline_->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(AdvectParameters)});
  advect_pipeline_->Initialize();
  emit_pipeline_ = std::make_shared<ComputePipeline>();
  emit_pipeline_->compute_shader = compute_shader("Emit.slang");
  emit_pipeline_->descriptor_set_layouts = {layout_};
  emit_pipeline_->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(EmitParameters)});
  emit_pipeline_->Initialize();
  clear_grid_pipeline_ = std::make_shared<ComputePipeline>();
  clear_grid_pipeline_->compute_shader = compute_shader("ClearGrid.slang");
  clear_grid_pipeline_->descriptor_set_layouts = {layout_};
  clear_grid_pipeline_->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(GridParameters)});
  clear_grid_pipeline_->Initialize();
  grid_pipeline_ = std::make_shared<ComputePipeline>();
  grid_pipeline_->compute_shader = compute_shader("BuildGrid.slang");
  grid_pipeline_->descriptor_set_layouts = {layout_};
  grid_pipeline_->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(GridParameters)});
  grid_pipeline_->Initialize();
  air_update_pipeline_ = std::make_shared<ComputePipeline>();
  air_update_pipeline_->compute_shader = compute_shader("UpdateAir.slang");
  air_update_pipeline_->descriptor_set_layouts = {air_update_layout_};
  air_update_pipeline_->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(AirParameters)});
  air_update_pipeline_->Initialize();
  trace_pipeline_ = std::make_shared<ComputePipeline>();
  trace_pipeline_->compute_shader = compute_shader("TraceAir.slang");
  trace_pipeline_->descriptor_set_layouts = {trace_layout_};
  trace_pipeline_->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(TraceParameters)});
  trace_pipeline_->Initialize();
  const auto rod_pipeline = [&](const char* path, uint32_t parameter_size) {
    auto pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = compute_shader(path);
    pipeline->descriptor_set_layouts = {rod_layout_};
    pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, parameter_size});
    pipeline->Initialize();
    return pipeline;
  };
  clear_rods_pipeline_ = rod_pipeline("ClearRods.slang", sizeof(RodParameters));
  build_rods_pipeline_ = rod_pipeline("BuildRods.slang", sizeof(RodParameters));
  rod_air_pipeline_ = rod_pipeline("RodAir.slang", sizeof(RodAirParameters));

  const auto graphics_shader = [](ShaderType type, const char* path) {
    return Shader::CreateTemporary(type, Platform::GetShaderGlobalDefines(),
                                   std::filesystem::path("./EcoSysLabResources/Shaders/Graphics") / path);
  };
  render_pipeline_ = std::make_shared<GraphicsPipeline>();
  render_pipeline_->vertex_shader = graphics_shader(ShaderType::Vertex, "WindParticles.vert.slang");
  render_pipeline_->fragment_shader = graphics_shader(ShaderType::Fragment, "WindParticles.frag.slang");
  render_pipeline_->geometry_type = GeometryType::Mesh;
  render_pipeline_->vertex_input_enabled = false;
  render_pipeline_->primitive_topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  render_pipeline_->descriptor_set_layouts = {render_layer->GetPerFrameDescriptorSetLayout(), layout_};
  render_pipeline_->depth_attachment_format = Platform::Constants::render_texture_depth;
  render_pipeline_->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  render_pipeline_->color_attachment_formats = {Platform::Constants::render_texture_color};
  render_pipeline_->push_constant_ranges.push_back({VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(RenderParameters)});
  render_pipeline_->Initialize();
  grid_render_pipeline_ = std::make_shared<GraphicsPipeline>();
  grid_render_pipeline_->vertex_shader = graphics_shader(ShaderType::Vertex, "AirGrid.vert.slang");
  grid_render_pipeline_->fragment_shader = graphics_shader(ShaderType::Fragment, "AirGrid.frag.slang");
  grid_render_pipeline_->geometry_type = GeometryType::Mesh;
  grid_render_pipeline_->vertex_input_enabled = false;
  grid_render_pipeline_->primitive_topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
  grid_render_pipeline_->descriptor_set_layouts = {render_layer->GetPerFrameDescriptorSetLayout(), grid_render_layout_};
  grid_render_pipeline_->depth_attachment_format = Platform::Constants::render_texture_depth;
  grid_render_pipeline_->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  grid_render_pipeline_->color_attachment_formats = {Platform::Constants::render_texture_color};
  grid_render_pipeline_->push_constant_ranges.push_back({VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(GridRenderParameters)});
  grid_render_pipeline_->Initialize();
  return advect_pipeline_->Initialized() && emit_pipeline_->Initialized() && clear_grid_pipeline_->Initialized() &&
         grid_pipeline_->Initialized() && air_update_pipeline_->Initialized() && trace_pipeline_->Initialized() &&
         clear_rods_pipeline_->Initialized() && build_rods_pipeline_->Initialized() &&
         rod_air_pipeline_->Initialized() && render_pipeline_->Initialized() && grid_render_pipeline_->Initialized();
}

void WindParticleField::InvalidateAirSets() {
  const auto frame = Platform::GetFrameCount();
  for (const auto& set : {air_descriptor_set_, air_update_set_, trace_set_, grid_render_set_})
    if (set)
      retired_air_sets_.push_back({frame, set});
  air_descriptor_set_.reset();
  air_update_set_.reset();
  trace_set_.reset();
  grid_render_set_.reset();
}

void WindParticleField::RegisterTreeCoverage(const std::vector<DynamicTreeSkeleton::GpuRestState>& trees) {
  tree_bounds_.clear();
  for (const auto& tree : trees) {
    if (tree.vertices.empty()) {
      tree_bounds_.push_back({});
      continue;
    }
    glm::vec3 lower(tree.vertices.front().position_inverse_mass);
    glm::vec3 upper = lower;
    const glm::mat4 inverse_model = glm::inverse(tree.initial_model);
    glm::vec3 local_lower(inverse_model * glm::vec4(lower, 1.0f));
    glm::vec3 local_upper = local_lower;
    for (const auto& vertex : tree.vertices) {
      const glm::vec3 point(vertex.position_inverse_mass);
      lower = glm::min(lower, point);
      upper = glm::max(upper, point);
      const glm::vec3 local(inverse_model * glm::vec4(point, 1.0f));
      local_lower = glm::min(local_lower, local);
      local_upper = glm::max(local_upper, local);
    }
    tree_bounds_.push_back({local_lower, local_upper, tree.initial_model});
    const glm::vec3 center = (lower + upper) * 0.5f;
    const float height = glm::max(upper.y - lower.y, air_grid_.GetCellSize());
    const glm::vec3 half_width = glm::max((upper - lower) * 0.5f, glm::vec3(air_grid_.GetCellSize() * 0.5f));
    lower = {center.x - 2.0f * half_width.x, lower.y - 0.2f * height, center.z - 2.0f * half_width.z};
    upper = {center.x + 2.0f * half_width.x, upper.y + 0.3f * height, center.z + 2.0f * half_width.z};
    if (air_grid_.RegisterBox(lower, upper)) {
      InvalidateAirSets();
    }
  }
}

void WindParticleField::RegisterTreeModels(const std::vector<glm::mat4>& models) {
  for (size_t i = 0; i < std::min(models.size(), tree_bounds_.size()); ++i) {
    auto& bounds = tree_bounds_[i];
    if (bounds.model == models[i])
      continue;
    glm::vec3 lower(std::numeric_limits<float>::max());
    glm::vec3 upper(std::numeric_limits<float>::lowest());
    for (uint32_t corner = 0; corner < 8; ++corner) {
      const glm::vec3 local{corner & 1 ? bounds.upper_local.x : bounds.lower_local.x,
                            corner & 2 ? bounds.upper_local.y : bounds.lower_local.y,
                            corner & 4 ? bounds.upper_local.z : bounds.lower_local.z};
      const glm::vec3 point(models[i] * glm::vec4(local, 1.0f));
      lower = glm::min(lower, point);
      upper = glm::max(upper, point);
    }
    const glm::vec3 center = (lower + upper) * 0.5f;
    const glm::vec3 half = glm::max((upper - lower) * 0.5f, glm::vec3(air_grid_.GetCellSize() * 0.5f));
    const float height = glm::max(upper.y - lower.y, air_grid_.GetCellSize());
    if (air_grid_.RegisterBox({center.x - 2.0f * half.x, lower.y - 0.2f * height, center.z - 2.0f * half.z},
                              {center.x + 2.0f * half.x, upper.y + 0.3f * height, center.z + 2.0f * half.z})) {
      InvalidateAirSets();
    }
    bounds.model = models[i];
  }
}

void WindParticleField::ResetAirGrid(const std::vector<DynamicTreeSkeleton::GpuRestState>& trees,
                                     const std::shared_ptr<GpuTreePhysics>& physics) {
  auto poses = trees;
  if (physics) {
    size_t count = 0;
    for (const auto& tree : poses)
      count += tree.vertices.size();
    std::vector<DynamicTreeSkeleton::GpuRestState::Vertex> vertices;
    physics->GetVertexBuffer()->DownloadVector(vertices, count);
    size_t offset = 0;
    for (auto& tree : poses)
      for (auto& vertex : tree.vertices)
        vertex.position_inverse_mass = vertices[offset++].position_inverse_mass;
  }
  air_grid_.Reset();
  InvalidateAirSets();
  RegisterTreeCoverage(poses);
}

void WindParticleField::Step(const std::shared_ptr<Scene>& scene, const float time_step,
                             const std::shared_ptr<GpuTreePhysics>& physics) {
  if (scene_.lock() != scene) {
    const bool replacing_scene = !scene_.expired();
    particles_.reset();
    grid_heads_.reset();
    grid_next_.reset();
    rod_heads_.reset();
    rod_next_.reset();
    rod_max_extent_.reset();
    descriptor_set_.reset();
    layout_.reset();
    advect_pipeline_.reset();
    emit_pipeline_.reset();
    clear_grid_pipeline_.reset();
    grid_pipeline_.reset();
    render_pipeline_.reset();
    if (replacing_scene)
      air_grid_.Reset();
    air_layout_.reset();
    grid_render_layout_.reset();
    grid_render_set_.reset();
    grid_render_pipeline_.reset();
    air_update_layout_.reset();
    air_descriptor_set_.reset();
    air_update_set_.reset();
    air_update_pipeline_.reset();
    trace_layout_.reset();
    trace_set_.reset();
    trace_pipeline_.reset();
    rod_layout_.reset();
    rod_frames_.clear();
    clear_rods_pipeline_.reset();
    build_rods_pipeline_.reset();
    rod_air_pipeline_.reset();
    retired_air_sets_.clear();
    emission_remainders_.clear();
    emitted_ = 0;
    scene_ = scene;
  }
  const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<Blower>();
  if ((!owners || owners->empty()) && !emitted_ && !air_grid_.GetCellCount())
    return;
  if (!Prepare())
    return;
  const auto frame = Platform::GetFrameCount();
  retired_air_sets_.erase(std::remove_if(retired_air_sets_.begin(), retired_air_sets_.end(),
                                         [&](const auto& retired) {
                                           return frame - retired.first > Platform::GetMaxFramesInFlight() + 1;
                                         }),
                          retired_air_sets_.end());
  if (air_grid_.GetCellCount() && !air_descriptor_set_) {
    air_descriptor_set_ = std::make_shared<DescriptorSet>(air_layout_);
    air_descriptor_set_->UpdateBufferDescriptorBinding(0, air_grid_.GetHeads());
    air_descriptor_set_->UpdateBufferDescriptorBinding(1, air_grid_.GetAddresses());
    air_update_set_ = std::make_shared<DescriptorSet>(air_update_layout_);
    air_update_set_->UpdateBufferDescriptorBinding(0, particles_);
    air_update_set_->UpdateBufferDescriptorBinding(1, grid_heads_);
    air_update_set_->UpdateBufferDescriptorBinding(2, grid_next_);
    air_update_set_->UpdateBufferDescriptorBinding(3, air_grid_.GetHeads());
    air_update_set_->UpdateBufferDescriptorBinding(4, air_grid_.GetAddresses());
    trace_set_ = std::make_shared<DescriptorSet>(trace_layout_);
    trace_set_->UpdateBufferDescriptorBinding(0, particles_);
    trace_set_->UpdateBufferDescriptorBinding(1, air_grid_.GetHeads());
    trace_set_->UpdateBufferDescriptorBinding(2, air_grid_.GetAddresses());
    grid_render_set_ = std::make_shared<DescriptorSet>(grid_render_layout_);
    grid_render_set_->UpdateBufferDescriptorBinding(0, air_grid_.GetAddresses());
  }
  const float dt = glm::clamp(time_step, 0.0f, 0.1f);
  if (dt <= 0.0f)
    return;
  if (emitted_) {
    const AdvectParameters parameters{dt, static_cast<uint32_t>(std::min<uint64_t>(emitted_, capacity))};
    Platform::RecordCommandsMainQueue([&](VkCommandBuffer command_buffer) {
      advect_pipeline_->Bind(command_buffer);
      advect_pipeline_->BindDescriptorSet(command_buffer, 0, descriptor_set_->GetVkDescriptorSet());
      advect_pipeline_->PushConstant(command_buffer, 0, parameters);
      advect_pipeline_->Dispatch(command_buffer, (parameters.count + 63) / 64, 1, 1);
      Platform::EverythingBarrier(command_buffer);
    });
  }
  if (owners)
    for (const auto entity : *owners) {
      if (!scene->IsEntityEnabled(entity))
        continue;
      const auto blower = scene->GetOrSetPrivateComponent<Blower>(entity).lock();
      if (!blower || !blower->IsEnabled())
        continue;
      const uint64_t key = (static_cast<uint64_t>(entity.GetVersion()) << 32) | entity.GetIndex();
      float& remainder = emission_remainders_[key];
      const float requested = remainder + glm::max(0.0f, blower->particle_rate) * dt;
      const uint32_t count = static_cast<uint32_t>(glm::min(std::floor(requested), 2048.0f));
      remainder = requested - std::floor(requested);
      if (!count)
        continue;
      EmitParameters parameters;
      parameters.model = scene->GetDataComponent<GlobalTransform>(entity).value;
      parameters.speed_spread_lifetime = {glm::max(0.0f, blower->speed),
                                          glm::radians(glm::clamp(blower->spread_degrees, 0.0f, 90.0f)), 3.0f, dt};
      parameters.collision.x = glm::clamp(blower->particle_radius, 0.0f, 1.0f);
      parameters.collision.y = glm::clamp(blower->wind_strength, 0.0f, 20.0f);
      parameters.collision.z =
          Climate::SpecificHumidity(blower->temperature_c, blower->relative_humidity, ambient_pressure_pa);
      parameters.collision.w = blower->temperature_c + 273.15f;
      parameters.base_count_seed_capacity = {static_cast<uint32_t>(emitted_), count, blower->seed, capacity};
      Platform::RecordCommandsMainQueue([&](VkCommandBuffer command_buffer) {
        emit_pipeline_->Bind(command_buffer);
        emit_pipeline_->BindDescriptorSet(command_buffer, 0, descriptor_set_->GetVkDescriptorSet());
        emit_pipeline_->PushConstant(command_buffer, 0, parameters);
        emit_pipeline_->Dispatch(command_buffer, (count + 63) / 64, 1, 1);
        Platform::EverythingBarrier(command_buffer);
      });
      emitted_ += count;
    }
  const auto build_particle_grid = [&] {
    Platform::RecordCommandsMainQueue([&](VkCommandBuffer command_buffer) {
      const GridParameters parameters{GetParticleCount(), grid_bucket_count, grid_cell_size};
      clear_grid_pipeline_->Bind(command_buffer);
      clear_grid_pipeline_->BindDescriptorSet(command_buffer, 0, descriptor_set_->GetVkDescriptorSet());
      clear_grid_pipeline_->PushConstant(command_buffer, 0, parameters);
      clear_grid_pipeline_->Dispatch(command_buffer, (grid_bucket_count + 63) / 64, 1, 1);
      Platform::EverythingBarrier(command_buffer);
      if (parameters.count) {
        grid_pipeline_->Bind(command_buffer);
        grid_pipeline_->BindDescriptorSet(command_buffer, 0, descriptor_set_->GetVkDescriptorSet());
        grid_pipeline_->PushConstant(command_buffer, 0, parameters);
        grid_pipeline_->Dispatch(command_buffer, (parameters.count + 63) / 64, 1, 1);
        Platform::EverythingBarrier(command_buffer);
      }
    });
  };
  build_particle_grid();
  if (air_update_set_) {
    const AirParameters parameters{
        air_grid_.GetCellCount(), grid_bucket_count, air_grid_.GetCellSize(),    dt,
        air_coupling_rate,        air_friction,      ambient_temperature_kelvin, ambient_specific_humidity,
        ambient_pressure_pa};
    Platform::RecordCommandsMainQueue([&](VkCommandBuffer command_buffer) {
      const auto timing = Platform::BeginGpuTimestampScope(
          command_buffer, {"EcoSysLabAirField", "EcoSysLab Air Field", "EcoSysLab", GpuTimestampQueue::Compute});
      air_update_pipeline_->Bind(command_buffer);
      air_update_pipeline_->BindDescriptorSet(command_buffer, 0, air_update_set_->GetVkDescriptorSet());
      air_update_pipeline_->PushConstant(command_buffer, 0, parameters);
      air_update_pipeline_->Dispatch(command_buffer, (parameters.cell_count + 63) / 64, 1, 1);
      Platform::EverythingBarrier(command_buffer);
      const TraceParameters trace{GetParticleCount(), GpuSpatialGrid::bucket_count, air_grid_.GetCellSize(), dt,
                                  air_coupling_rate};
      if (trace.particle_count) {
        trace_pipeline_->Bind(command_buffer);
        trace_pipeline_->BindDescriptorSet(command_buffer, 0, trace_set_->GetVkDescriptorSet());
        trace_pipeline_->PushConstant(command_buffer, 0, trace);
        trace_pipeline_->Dispatch(command_buffer, (trace.particle_count + 63) / 64, 1, 1);
        Platform::EverythingBarrier(command_buffer);
      }
    });
  }
  const uint32_t rod_count = physics ? physics->GetRodCount() : 0;
  if (rod_count && air_update_set_) {
    if (rod_next_->GetSize() < static_cast<VkDeviceSize>(rod_count) * sizeof(uint32_t)) {
      VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
      info.size = static_cast<VkDeviceSize>(rod_count) * sizeof(uint32_t);
      info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
      info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
      VmaAllocationCreateInfo allocation{};
      allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
      rod_next_ = std::make_shared<Buffer>(info, allocation);
    }
    auto& frame = rod_frames_[Platform::GetCurrentFrameIndex()];
    if (frame.frame_count != Platform::GetFrameCount()) {
      frame.frame_count = Platform::GetFrameCount();
      frame.used = 0;
      frame.physics.clear();
      frame.buffers.clear();
    }
    const auto acquire = [&] {
      if (frame.used == frame.sets.size())
        frame.sets.push_back(std::make_shared<DescriptorSet>(rod_layout_));
      return frame.sets[frame.used++];
    };
    const auto rods = acquire();
    rods->UpdateBufferDescriptorBinding(0, particles_);
    rods->UpdateBufferDescriptorBinding(1, rod_heads_);
    rods->UpdateBufferDescriptorBinding(2, rod_next_);
    rods->UpdateBufferDescriptorBinding(3, rod_max_extent_);
    rods->UpdateBufferDescriptorBinding(4, physics->GetVertexBuffer());
    rods->UpdateBufferDescriptorBinding(5, physics->GetRodBuffer());
    const auto air = acquire();
    air->UpdateBufferDescriptorBinding(0, air_grid_.GetAddresses());
    air->UpdateBufferDescriptorBinding(1, rod_heads_);
    air->UpdateBufferDescriptorBinding(2, rod_next_);
    air->UpdateBufferDescriptorBinding(3, rod_max_extent_);
    air->UpdateBufferDescriptorBinding(4, physics->GetVertexBuffer());
    air->UpdateBufferDescriptorBinding(5, physics->GetRodBuffer());
    frame.physics.push_back(physics);
    frame.buffers.push_back(rod_next_);
    const RodParameters rod_parameters{GetParticleCount(), rod_count, grid_bucket_count, air_grid_.GetCellSize()};
    const RodAirParameters air_parameters{air_grid_.GetCellCount(), rod_count, grid_bucket_count,
                                          air_grid_.GetCellSize(), dt};
    Platform::RecordCommandsMainQueue([&, rods, air](VkCommandBuffer command_buffer) {
      const auto timing = Platform::BeginGpuTimestampScope(
          command_buffer, {"EcoSysLabRodAir", "EcoSysLab Rod Air", "EcoSysLab", GpuTimestampQueue::Compute});
      const auto dispatch_rods = [&](const std::shared_ptr<ComputePipeline>& pipeline, uint32_t count) {
        pipeline->Bind(command_buffer);
        pipeline->BindDescriptorSet(command_buffer, 0, rods->GetVkDescriptorSet());
        pipeline->PushConstant(command_buffer, 0, rod_parameters);
        pipeline->Dispatch(command_buffer, (count + 63) / 64, 1, 1);
        Platform::EverythingBarrier(command_buffer);
      };
      dispatch_rods(clear_rods_pipeline_, grid_bucket_count);
      dispatch_rods(build_rods_pipeline_, rod_count);
      rod_air_pipeline_->Bind(command_buffer);
      rod_air_pipeline_->BindDescriptorSet(command_buffer, 0, air->GetVkDescriptorSet());
      rod_air_pipeline_->PushConstant(command_buffer, 0, air_parameters);
      rod_air_pipeline_->Dispatch(command_buffer, (air_parameters.cell_count + 63) / 64, 1, 1);
      Platform::EverythingBarrier(command_buffer);
    });
  }
}

void WindParticleField::RegisterRender(const bool particles_visible, const bool grid_visible) {
  if ((!particles_visible || !emitted_) && (!grid_visible || !air_grid_.GetCellCount()))
    return;
  if (!render_pipeline_ || !render_pipeline_->Initialized() || !grid_render_pipeline_ ||
      !grid_render_pipeline_->Initialized())
    return;
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer)
    return;
  const auto self = shared_from_this();
  render_layer->ForwardRenderingAllCameras(
      [self, particles_visible, grid_visible](VkCommandBuffer command_buffer, const std::shared_ptr<Camera>& camera,
                                              const RenderLayer::ForwardRenderingView& view) {
        return self->Draw(command_buffer, camera, view.viewport, view.camera_index, particles_visible, grid_visible);
      });
}

uint32_t WindParticleField::Draw(VkCommandBuffer command_buffer, const std::shared_ptr<Camera>& camera,
                                 const glm::ivec4& viewport, const uint32_t camera_index, const bool particles_visible,
                                 const bool grid_visible) {
  if (!camera || !camera->GetRenderTexture() || !RenderLayer::GetPerFrameDescriptorSet())
    return 0;
  std::vector<VkRenderingAttachmentInfo> colors;
  camera->GetRenderTexture()->AppendColorAttachmentInfos(colors, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                         VK_ATTACHMENT_STORE_OP_STORE);
  const auto depth =
      camera->GetRenderTexture()->GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE);
  VkRenderingInfo rendering{VK_STRUCTURE_TYPE_RENDERING_INFO};
  rendering.renderArea = {{viewport.x, viewport.y},
                          {static_cast<uint32_t>(viewport.z), static_cast<uint32_t>(viewport.w)}};
  rendering.layerCount = 1;
  rendering.colorAttachmentCount = static_cast<uint32_t>(colors.size());
  rendering.pColorAttachments = colors.data();
  rendering.pDepthAttachment = &depth;
  Platform::BeginRendering(command_buffer, rendering);
  uint32_t primitives = 0;
  if (particles_visible && emitted_) {
    render_pipeline_->states.ResetAllStates(1);
    render_pipeline_->states.SetViewportScissor(viewport);
    render_pipeline_->states.cull_mode = VK_CULL_MODE_NONE;
    render_pipeline_->states.depth_test = true;
    render_pipeline_->states.depth_write = false;
    render_pipeline_->states.depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;
    render_pipeline_->states.ApplyAllStates(command_buffer);
    render_pipeline_->Bind(command_buffer);
    render_pipeline_->BindDescriptorSet(command_buffer, 0,
                                        RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
    render_pipeline_->BindDescriptorSet(command_buffer, 1, descriptor_set_->GetVkDescriptorSet());
    render_pipeline_->PushConstant(command_buffer, 0, RenderParameters{camera_index, {viewport.z, viewport.w}, 3.0f});
    const uint32_t count = GetParticleCount();
    Platform::Draw(command_buffer, 6, count);
    primitives += count * 2;
  }
  if (grid_visible && air_grid_.GetCellCount() && grid_render_set_) {
    grid_render_pipeline_->states.ResetAllStates(1);
    grid_render_pipeline_->states.SetViewportScissor(viewport);
    grid_render_pipeline_->states.depth_test = true;
    grid_render_pipeline_->states.depth_write = false;
    grid_render_pipeline_->states.depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;
    grid_render_pipeline_->states.color_blend_attachment_states[0].blendEnable = VK_TRUE;
    grid_render_pipeline_->states.ApplyAllStates(command_buffer);
    grid_render_pipeline_->Bind(command_buffer);
    grid_render_pipeline_->BindDescriptorSet(command_buffer, 0,
                                             RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
    grid_render_pipeline_->BindDescriptorSet(command_buffer, 1, grid_render_set_->GetVkDescriptorSet());
    grid_render_pipeline_->PushConstant(command_buffer, 0,
                                        GridRenderParameters{camera_index, air_grid_.GetCellSize(), 0.1f, 0.0f});
    Platform::Draw(command_buffer, 30, air_grid_.GetCellCount());
    primitives += air_grid_.GetCellCount() * 15;
  }
  Platform::EndRendering(command_buffer);
  return primitives;
}
