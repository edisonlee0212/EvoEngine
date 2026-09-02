#include "UniverseLayer.hpp"

#include "Application.hpp"
#include "ComputePipeline.hpp"
#include "EditorLayer.hpp"
#include "GpuProfiler.hpp"
#include "GraphicsPipeline.hpp"
#include "ProjectManager.hpp"
#include "RenderTexture.hpp"
#include "Shader.hpp"
#include "Times.hpp"
#include "UniverseProfiler.hpp"

using namespace universe_package;

namespace {
constexpr uint32_t kWorkgroupSize = 256;

std::shared_ptr<Buffer> CreateDeviceBuffer(const VkDeviceSize size, const VkBufferUsageFlags usage) {
  VkBufferCreateInfo create_info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  create_info.size = (std::max)(size, VkDeviceSize{1});
  create_info.usage = usage;
  create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation_info{};
  allocation_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  return std::make_shared<Buffer>(create_info, allocation_info);
}

std::shared_ptr<Buffer> CreateHostReadbackBuffer(const VkDeviceSize size) {
  VkBufferCreateInfo create_info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  create_info.size = (std::max)(size, VkDeviceSize{1});
  create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation_info{};
  allocation_info.usage = VMA_MEMORY_USAGE_AUTO;
  allocation_info.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
  return std::make_shared<Buffer>(create_info, allocation_info);
}

struct StarRenderPushConstant {
  int32_t camera_index = 0;
};

bool UsesHostCoherentMemory(const Buffer& buffer) {
  const auto memory_type = buffer.GetVmaAllocationInfo().memoryType;
  const auto physical_device = Platform::GetSelectedPhysicalDevice();
  return physical_device && memory_type < physical_device->vk_physical_device_memory_properties.memoryTypeCount &&
         (physical_device->vk_physical_device_memory_properties.memoryTypes[memory_type].propertyFlags &
          VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) != 0;
}

bool IsProceduralGalaxyProjectPath() {
  const auto project_path = ProjectManager::GetProjectPath();
  return project_path.filename() == "ProceduralGalaxy.eveproj" && project_path.parent_path().filename() == "Universe" &&
         project_path.parent_path().parent_path().filename() == "EvoEngine-DemoProjects";
}

void CheckLod(std::mutex& mutex, const std::shared_ptr<TerrainChunk>& chunk, const PlanetInfo& info,
              const GlobalTransform& planet_transform, const GlobalTransform& camera_transform) {
  if (glm::distance(glm::dvec3(chunk->ChunkCenterPosition(planet_transform.GetPosition(), info.radius,
                                                          planet_transform.GetRotation())),
                    glm::dvec3(camera_transform.GetPosition())) <
      info.lod_distance * info.radius / glm::pow(2, chunk->detail_level + 1)) {
    if (chunk->detail_level < info.max_lod_level)
      chunk->Expand(mutex);
  }
  if (chunk->c0)
    CheckLod(mutex, chunk->c0, info, planet_transform, camera_transform);
  if (chunk->c1)
    CheckLod(mutex, chunk->c1, info, planet_transform, camera_transform);
  if (chunk->c2)
    CheckLod(mutex, chunk->c2, info, planet_transform, camera_transform);
  if (chunk->c3)
    CheckLod(mutex, chunk->c3, info, planet_transform, camera_transform);
  if (glm::distance(glm::dvec3(chunk->ChunkCenterPosition(planet_transform.GetPosition(), info.radius,
                                                          planet_transform.GetRotation())),
                    glm::dvec3(camera_transform.GetPosition())) >
      info.lod_distance * info.radius / glm::pow(2, chunk->detail_level + 1))
    chunk->Collapse();
}

void RenderChunk(const std::shared_ptr<TerrainChunk>& chunk, const std::shared_ptr<Material>& material,
                 const GlobalTransform& transform) {
  if (chunk->active)
    ApplicationContext::Get().GetLayer<RenderLayer>()->DrawMesh(chunk->mesh, material, transform, true);
  if (chunk->children_active) {
    RenderChunk(chunk->c0, material, transform);
    RenderChunk(chunk->c1, material, transform);
    RenderChunk(chunk->c2, material, transform);
    RenderChunk(chunk->c3, material, transform);
  }
}
}  // namespace

void UniverseLayer::RegisterTypes(Application&) {
}

bool universe_package::InspectUniverseLayer(InspectorContext&, UniverseLayer& layer) {
  const auto window_title = layer.GetLayerName();
  bool open = layer.enable_inspection;
  if (!ImGui::Begin(window_title.c_str(), &open)) {
    ImGui::End();
    layer.enable_inspection = open;
    return false;
  }
  ImGui::Text("Global simulation time: %.3f", layer.global_simulation_time_);
  ImGui::Text("Frame: %llu", static_cast<unsigned long long>(layer.frame_number_));
  ImGui::Text("FP64 compute: %s", layer.fp64_supported_ ? "supported" : "unavailable");
  ImGui::Text("Compute pipeline: %s", layer.compute_ready_ ? "ready" : "unavailable");
  ImGui::Text("Forward pipeline: %s", layer.render_ready_ ? "ready" : "unavailable");
  ImGui::Text("Registered clusters: %u, stars: %u", layer.registered_render_cluster_count_,
              layer.registered_render_star_count_);
  ImGui::TextWrapped("GPU status: %s", layer.gpu_status_.c_str());
  ImGui::End();
  layer.enable_inspection = open;
  return false;
}

void UniverseLayer::OnCreate() {
  global_simulation_time_ = 0.0;
  frame_number_ = 0;
  registered_render_cluster_count_ = 0;
  registered_render_star_count_ = 0;
  procedural_galaxy_scene_.reset();
  parameter_upload_arena_ = std::make_unique<BufferUploadArena>();
  InitializeGpuResources();
}

void UniverseLayer::InitializeGpuResources() {
  compute_ready_ = false;
  render_ready_ = false;
  const auto physical_device = Platform::GetSelectedPhysicalDevice();
  fp64_supported_ = physical_device && physical_device->features.shaderFloat64 == VK_TRUE;
  if (!fp64_supported_) {
    gpu_status_ = "Disabled: Vulkan shaderFloat64 is unavailable; FP32 fallback is intentionally unsupported.";
    EVOENGINE_ERROR(gpu_status_);
    return;
  }

  star_cluster_layout_ = std::make_shared<DescriptorSetLayout>();
  star_cluster_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  star_cluster_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  star_cluster_layout_->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  star_cluster_layout_->Initialize();

  const auto shader = std::make_shared<Shader>();
  const auto shader_path = std::filesystem::path("./UniverseResources/Shaders/Compute/StarCluster.slang");
  if (!shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(), shader_path)) {
    gpu_status_ = "Disabled: failed to compile " + shader_path.string();
    EVOENGINE_ERROR(gpu_status_);
    star_cluster_layout_.reset();
    return;
  }
  star_cluster_pipeline_ = std::make_shared<ComputePipeline>();
  star_cluster_pipeline_->compute_shader = shader;
  star_cluster_pipeline_->descriptor_set_layouts.emplace_back(star_cluster_layout_);
  star_cluster_pipeline_->Initialize();

  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer) {
    gpu_status_ = "Disabled: RenderLayer is unavailable.";
    EVOENGINE_ERROR(gpu_status_);
    return;
  }
  star_render_layout_ = std::make_shared<DescriptorSetLayout>();
  star_render_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_VERTEX_BIT, 0);
  star_render_layout_->Initialize();

  const auto vertex_path = std::filesystem::path("./UniverseResources/Shaders/Graphics/StarCluster.vert.slang");
  const auto fragment_path = std::filesystem::path("./UniverseResources/Shaders/Graphics/StarCluster.frag.slang");
  auto vertex_shader = std::make_shared<Shader>();
  auto fragment_shader = std::make_shared<Shader>();
  if (!vertex_shader->TryCompile(ShaderType::Vertex, Platform::GetShaderGlobalDefines(), vertex_path) ||
      !fragment_shader->TryCompile(ShaderType::Fragment, Platform::GetShaderGlobalDefines(), fragment_path)) {
    gpu_status_ = "Disabled: failed to compile Star Cluster forward shaders.";
    EVOENGINE_ERROR(gpu_status_);
    star_render_layout_.reset();
    return;
  }
  star_render_pipeline_ = std::make_shared<GraphicsPipeline>();
  star_render_pipeline_->vertex_shader = vertex_shader;
  star_render_pipeline_->fragment_shader = fragment_shader;
  star_render_pipeline_->geometry_type = GeometryType::Mesh;
  star_render_pipeline_->vertex_input_enabled = false;
  star_render_pipeline_->primitive_topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  star_render_pipeline_->descriptor_set_layouts.emplace_back(render_layer->GetPerFrameDescriptorSetLayout());
  star_render_pipeline_->descriptor_set_layouts.emplace_back(star_render_layout_);
  star_render_pipeline_->depth_attachment_format = Platform::Constants::render_texture_depth;
  star_render_pipeline_->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  star_render_pipeline_->color_attachment_formats = {Platform::Constants::render_texture_color};
  auto& push_constant = star_render_pipeline_->push_constant_ranges.emplace_back();
  push_constant.size = sizeof(StarRenderPushConstant);
  push_constant.offset = 0;
  push_constant.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
  star_render_pipeline_->Initialize();
  compute_ready_ = star_cluster_pipeline_->Initialized();
  render_ready_ = star_render_pipeline_->Initialized();
  gpu_status_ = compute_ready_ && render_ready_ ? "FP64 compute and direct forward pipelines ready"
                                                : "Disabled: Star Cluster GPU pipeline initialization failed.";
}

void UniverseLayer::OnDestroy() {
  Platform::WaitForFrameSubmissions("Destroy Universe star-cluster resources");
  star_cluster_pipeline_.reset();
  star_cluster_layout_.reset();
  star_render_pipeline_.reset();
  star_render_layout_.reset();
  parameter_upload_arena_.reset();
  compute_ready_ = false;
  render_ready_ = false;
  procedural_galaxy_scene_.reset();
}

void UniverseLayer::ConfigureProceduralGalaxyDemoIfNeeded() {
  const auto scene = GetScene();
  if (!scene || procedural_galaxy_scene_.lock() == scene || !IsProceduralGalaxyProjectPath())
    return;
  procedural_galaxy_scene_ = scene;
  const auto owners = scene->UnsafeGetPrivateComponentOwnersList<StarCluster>();
  if (owners && !owners->empty())
    return;

  const auto first_entity = scene->CreateEntity("Star Cluster A");
  const auto first = scene->GetOrSetPrivateComponent<StarCluster>(first_entity).lock();
  first->seed = 1;
  (void)first->AddStars(25000);

  const auto second_entity = scene->CreateEntity("Star Cluster B");
  Transform second_transform;
  second_transform.SetPosition(glm::vec3(180.0f, 20.0f, -80.0f));
  scene->SetDataComponent(second_entity, second_transform);
  const auto second = scene->GetOrSetPrivateComponent<StarCluster>(second_entity).lock();
  second->seed = 2;
  second->disk_color = glm::vec3(1.0f, 0.15f, 0.05f);
  second->core_color = glm::vec3(0.7f, 0.25f, 1.0f);
  second->disk_diameter = 1800.0;
  second->disk_eccentricity = 0.65;
  second->time_scale = -35.0;
  (void)second->AddStars(25000);
}

void UniverseLayer::EnsureClusterResources(StarCluster& cluster) {
  if (!compute_ready_ || !render_ready_ || !star_cluster_layout_ || !star_render_layout_)
    return;
  const size_t frame_count = static_cast<size_t>((std::max)(Platform::GetMaxFramesInFlight(), 1));
  const size_t required_count = cluster.GetStarCount();
  const bool initialize_slots = cluster.frame_slots_.size() != frame_count;
  const bool population_changed = cluster.gpu_resources_dirty_;
  if (!initialize_slots && !population_changed)
    return;

  if (population_changed && (cluster.base_sample_buffer_ || !cluster.frame_slots_.empty()))
    Platform::WaitForFrameSubmissions("Edit Star Cluster population");

  size_t capacity = (std::max)(cluster.capacity_, size_t{1});
  while (capacity < required_count)
    capacity *= 2;
  const bool replace_allocations = initialize_slots || capacity != cluster.capacity_;
  cluster.capacity_ = capacity;
  if (replace_allocations) {
    cluster.inspection_readback_supported_ = true;
    cluster.base_sample_buffer_ = CreateDeviceBuffer(
        capacity * sizeof(StarBaseSample), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
    cluster.frame_slots_.assign(frame_count, {});
    for (auto& slot : cluster.frame_slots_) {
      slot.parameter_buffer = CreateDeviceBuffer(sizeof(StarClusterGpuParameters),
                                                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
      slot.result_buffer = CreateDeviceBuffer(capacity * sizeof(StarClusterGpuResult),
                                              VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT);
      slot.inspection_readback_buffer = CreateHostReadbackBuffer(capacity * sizeof(StarClusterGpuResult));
      if (!UsesHostCoherentMemory(*slot.inspection_readback_buffer)) {
        cluster.inspection_readback_supported_ = false;
        cluster.last_readback_status_ = "Unavailable: inspection requires host-coherent readback memory";
        EVOENGINE_WARNING(cluster.last_readback_status_);
      }
      slot.compute_descriptor_set = std::make_shared<DescriptorSet>(star_cluster_layout_);
      slot.compute_descriptor_set->UpdateBufferDescriptorBinding(0, slot.parameter_buffer);
      slot.compute_descriptor_set->UpdateBufferDescriptorBinding(1, cluster.base_sample_buffer_);
      slot.compute_descriptor_set->UpdateBufferDescriptorBinding(2, slot.result_buffer);
      slot.render_descriptor_set = std::make_shared<DescriptorSet>(star_render_layout_);
      slot.render_descriptor_set->UpdateBufferDescriptorBinding(0, slot.result_buffer);
    }
  }
  if (!cluster.base_samples_.empty())
    cluster.base_sample_buffer_->UploadData(cluster.base_samples_.size() * sizeof(StarBaseSample),
                                            cluster.base_samples_.data());
  cluster.gpu_resources_dirty_ = false;
}

void UniverseLayer::ConsumeCompletedSlot(StarCluster& cluster, const uint32_t frame_index) {
  if (frame_index >= cluster.frame_slots_.size())
    return;
  auto& slot = cluster.frame_slots_[frame_index];
  if (!slot.readback_pending)
    return;
  if (slot.submitted_population_revision != cluster.population_revision_) {
    slot.readback_pending = false;
    cluster.last_readback_status_ = "Discarded stale population readback";
    return;
  }

  std::vector<StarClusterGpuResult> results(slot.submitted_count);
  {
    const ProfilerScope readback_scope(universe_profiler::GetItems().readback);
    if (!results.empty())
      slot.inspection_readback_buffer->DownloadData(results.size() * sizeof(StarClusterGpuResult), results.data());
  }
  cluster.completed_world_positions_.resize(results.size());
  for (size_t i = 0; i < results.size(); ++i)
    cluster.completed_world_positions_[i] = glm::dvec3(results[i].world_position_radius);
  cluster.completed_simulation_time_ = slot.submitted_simulation_time;
  ++cluster.completed_update_count_;
  cluster.last_readback_status_ = "Completed " + std::to_string(results.size()) + " stars";
  slot.readback_pending = false;
}

std::function<void(VkCommandBuffer)> UniverseLayer::PrepareClusterCompute(StarCluster& cluster,
                                                                          const uint32_t frame_index) {
  auto& slot = cluster.frame_slots_.at(frame_index);
  slot.submitted_population_revision = cluster.population_revision_;
  slot.submitted_count = cluster.GetStarCount();
  slot.submitted_frame = frame_number_;
  cluster.computed_population_revision_ = slot.submitted_population_revision;
  const bool request_readback = cluster.position_readback_requested_ && cluster.inspection_readback_supported_;
  cluster.position_readback_requested_ = false;
  slot.readback_pending = request_readback;
  if (slot.submitted_count == 0) {
    cluster.last_compute_status_ = "Submitted empty population";
    slot.readback_pending = false;
    return {};
  }

  const auto pipeline = star_cluster_pipeline_;
  const auto descriptor_set = slot.compute_descriptor_set;
  const auto result_buffer = slot.result_buffer;
  const auto readback_buffer = slot.inspection_readback_buffer;
  const VkDeviceSize result_size = slot.submitted_count * sizeof(StarClusterGpuResult);
  cluster.last_compute_status_ = "Submitted " + std::to_string(slot.submitted_count) + " stars";
  cluster.last_readback_status_ = request_readback ? "Inspection copy submitted" : "Idle (positions collapsed)";
  return [pipeline, descriptor_set, result_buffer, readback_buffer, result_size,
          request_readback](const VkCommandBuffer command_buffer) {
    {
      const GpuProfilerCommandScope compute_scope(command_buffer, universe_profiler::GetItems().compute);
      pipeline->Bind(command_buffer);
      pipeline->BindDescriptorSet(command_buffer, 0, descriptor_set->GetVkDescriptorSet());
      pipeline->Dispatch(
          command_buffer,
          Platform::DivUp(static_cast<uint32_t>(result_size / sizeof(StarClusterGpuResult)), kWorkgroupSize), 1, 1);
    }
    Platform::BufferMemoryBarrier(command_buffer, *result_buffer, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                  VK_ACCESS_2_SHADER_WRITE_BIT,
                                  VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                                  VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_TRANSFER_READ_BIT);
    if (request_readback) {
      const GpuProfilerCommandScope copy_scope(command_buffer, universe_profiler::GetItems().inspection_copy);
      Platform::CopyBuffer(command_buffer, *result_buffer, *readback_buffer, result_size);
      Platform::BufferMemoryBarrier(command_buffer, *readback_buffer, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                                    VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_HOST_BIT,
                                    VK_ACCESS_2_HOST_READ_BIT);
    }
  };
}

void UniverseLayer::RegisterForwardRendering(std::vector<StarClusterRenderPacket> render_packets) {
  registered_render_cluster_count_ = static_cast<uint32_t>(render_packets.size());
  registered_render_star_count_ = 0;
  for (const auto& packet : render_packets)
    registered_render_star_count_ += packet.star_count;
  if (render_packets.empty() || !render_ready_ || !star_render_pipeline_)
    return;
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer)
    return;
  const ProfilerScope registration_scope(universe_profiler::GetItems().render_registration);
  const auto pipeline = star_render_pipeline_;
  render_layer->ForwardRenderingAllCameras([pipeline, render_packets = std::move(render_packets)](
                                               const VkCommandBuffer command_buffer,
                                               const std::shared_ptr<Camera>& camera,
                                               const RenderLayer::ForwardRenderingView& view) -> uint32_t {
    if (!camera || !camera->GetRenderTexture() || !RenderLayer::GetPerFrameDescriptorSet())
      return 0;
    pipeline->states.ResetAllStates(1);
    pipeline->states.SetViewportScissor(view.viewport);
    pipeline->states.cull_mode = VK_CULL_MODE_NONE;
    pipeline->states.depth_test = true;
    pipeline->states.depth_write = false;
    pipeline->states.depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;
    auto& blend = pipeline->states.color_blend_attachment_states[0];
    blend.blendEnable = VK_TRUE;
    blend.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
    blend.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
    blend.colorBlendOp = VK_BLEND_OP_ADD;
    blend.srcAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
    blend.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    blend.alphaBlendOp = VK_BLEND_OP_ADD;
    blend.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

    uint32_t primitive_count = 0;
    camera->GetRenderTexture()->Render(command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
      const GpuProfilerCommandScope render_scope(command_buffer, universe_profiler::GetItems().forward_render);
      pipeline->states.ApplyAllStates(command_buffer);
      pipeline->Bind(command_buffer);
      pipeline->BindDescriptorSet(command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
      pipeline->PushConstant(command_buffer, 0, StarRenderPushConstant{view.camera_index});
      for (const auto& packet : render_packets)
        primitive_count += StarCluster::RecordForwardDraw(packet, command_buffer, *pipeline);
    });
    return primitive_count;
  });
}

void UniverseLayer::UpdatePlanetTerrain(const std::shared_ptr<Scene>& scene) const {
  const auto terrains = scene->UnsafeGetPrivateComponentOwnersList<PlanetTerrain>();
  const auto main_camera = scene->main_camera.Get<Camera>();
  if (!terrains || !main_camera)
    return;
  std::mutex mesh_gen_lock;
  const auto camera_transform = scene->GetDataComponent<GlobalTransform>(main_camera->GetOwner());
  for (const auto entity : *terrains) {
    const auto terrain = scene->GetOrSetPrivateComponent<PlanetTerrain>(entity).lock();
    if (!scene->IsEntityEnabled(entity) || !terrain || !terrain->IsEnabled())
      continue;
    const auto planet_transform = scene->GetDataComponent<GlobalTransform>(entity);
    for (auto& chunk : terrain->chunks_)
      CheckLod(mesh_gen_lock, chunk, terrain->info_, planet_transform, camera_transform);
    GlobalTransform render_transform;
    render_transform.value =
        glm::translate(glm::mat4_cast(planet_transform.GetRotation()), glm::vec3(planet_transform.GetPosition()));
    if (const auto material = terrain->surface_material.Get<Material>()) {
      for (const auto& chunk : terrain->chunks_)
        RenderChunk(chunk, material, render_transform);
    }
  }
}

void UniverseLayer::Update() {
  const auto scene = GetScene();
  if (!scene)
    return;
  ConfigureProceduralGalaxyDemoIfNeeded();
  UpdatePlanetTerrain(scene);
  global_simulation_time_ += ApplicationContext::Get().GetTimes().DeltaTime();
  ++frame_number_;
  registered_render_cluster_count_ = 0;
  registered_render_star_count_ = 0;
  if (!compute_ready_ || !render_ready_)
    return;

  const uint32_t frame_index = Platform::GetCurrentFrameIndex();
  const auto owners = scene->UnsafeGetPrivateComponentOwnersList<StarCluster>();
  if (!owners)
    return;
  std::vector<std::function<void(VkCommandBuffer)>> dispatches;
  dispatches.reserve(owners->size());
  std::vector<StarClusterRenderPacket> render_packets;
  render_packets.reserve(owners->size());
  std::vector<StarClusterGpuParameters> parameter_payloads;
  parameter_payloads.reserve(owners->size());
  BufferUploadBatch parameter_uploads;
  for (const auto entity : *owners) {
    const auto cluster = scene->GetOrSetPrivateComponent<StarCluster>(entity).lock();
    if (!cluster || !scene->IsEntityEnabled(entity) || !cluster->IsEnabled())
      continue;
    EnsureClusterResources(*cluster);
    if (!compute_ready_)
      break;
    ConsumeCompletedSlot(*cluster, frame_index);
    if (cluster->GetStarCount() != 0) {
      const double simulation_time = cluster->AdvanceClock(global_simulation_time_);
      parameter_payloads.emplace_back(cluster->BuildGpuParameters(
          glm::dmat4(scene->GetDataComponent<GlobalTransform>(entity).value), simulation_time));
      cluster->frame_slots_.at(frame_index).submitted_simulation_time = simulation_time;
      parameter_uploads.Add(cluster->frame_slots_.at(frame_index).parameter_buffer, parameter_payloads.back());
    } else {
      (void)cluster->AdvanceClock(global_simulation_time_);
    }
    if (auto dispatch = PrepareClusterCompute(*cluster, frame_index))
      dispatches.emplace_back(std::move(dispatch));
    if (auto packet = cluster->BuildRenderPacket(frame_index); packet.star_count != 0)
      render_packets.emplace_back(std::move(packet));
  }
  if (parameter_upload_arena_)
    parameter_uploads.Record(*parameter_upload_arena_);
  if (!dispatches.empty()) {
    Platform::RecordCommandsMainQueue([dispatches = std::move(dispatches)](const VkCommandBuffer command_buffer) {
      for (const auto& dispatch : dispatches)
        dispatch(command_buffer);
    });
  }
  RegisterForwardRendering(std::move(render_packets));
}
