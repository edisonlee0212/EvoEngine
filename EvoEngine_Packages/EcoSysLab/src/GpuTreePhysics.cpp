#include "GpuTreePhysics.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>

#include "Profiler.hpp"
#include "Shader.hpp"

using namespace eco_sys_lab_package;

namespace {
template <typename T>
std::shared_ptr<Buffer> MakeBuffer(const std::vector<T>& source) {
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = std::max<size_t>(1, source.size()) * sizeof(T);
  info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT |
               VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
  info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation{};
  allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  auto buffer = std::make_shared<Buffer>(info, allocation);
  if (source.empty())
    buffer->UploadVector(std::vector<T>(1));
  else
    buffer->UploadVector(source);
  return buffer;
}
template <typename T>
std::shared_ptr<Buffer> MakeAttributeBuffer(const std::vector<T>& source, bool arrays) {
  if (!arrays)
    return MakeBuffer(source);
  static_assert(sizeof(T) % 16 == 0);
  constexpr size_t fields = sizeof(T) / 16;
  std::vector<glm::uvec4> packed(source.size() * fields);
  for (size_t i = 0; i < source.size(); ++i)
    for (size_t field = 0; field < fields; ++field)
      std::memcpy(&packed[field * source.size() + i], reinterpret_cast<const char*>(&source[i]) + field * 16, 16);
  return MakeBuffer(packed);
}
template <typename T>
void DownloadAttributes(const std::shared_ptr<Buffer>& buffer, uint32_t count, bool arrays, std::vector<T>& output) {
  if (!arrays) {
    buffer->DownloadVector(output, count);
    return;
  }
  constexpr size_t fields = sizeof(T) / 16;
  std::vector<glm::uvec4> packed;
  buffer->DownloadVector(packed, count * fields);
  output.resize(count);
  for (size_t i = 0; i < count; ++i)
    for (size_t field = 0; field < fields; ++field)
      std::memcpy(reinterpret_cast<char*>(&output[i]) + field * 16, &packed[field * count + i], 16);
}
}  // namespace

bool GpuTreePhysics::Prepare() {
  if (layout_)
    return predict_->Initialized() && solve_position_->Initialized() && solve_rotation_->Initialized() &&
           update_velocity_->Initialized() && prepare_coefficients_->Initialized();
  layout_ = std::make_shared<DescriptorSetLayout>();
  for (uint32_t i = 0; i < 5; ++i)
    layout_->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout_->Initialize();
  wind_layout_ = std::make_shared<DescriptorSetLayout>();
  for (uint32_t i = 0; i < 2; ++i)
    wind_layout_->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  wind_layout_->Initialize();
  empty_wind_set_ = std::make_shared<DescriptorSet>(wind_layout_);
  empty_wind_buffers_ = {MakeBuffer(std::vector<uint32_t>(1, UINT32_MAX)), MakeBuffer(std::vector<glm::u64vec2>(1))};
  for (uint32_t i = 0; i < 2; ++i)
    empty_wind_set_->UpdateBufferDescriptorBinding(i, empty_wind_buffers_[i]);
  reference_ = std::getenv("EVOENGINE_TREE_PHYSICS_REFERENCE") || std::getenv("EVOENGINE_TREE_PHYSICS_FULL_WRITES");
  const std::string reference_define = reference_ ? "\n#define EE_TREE_PHYSICS_REFERENCE 1\n" : "";
  const std::string write_define =
      std::getenv("EVOENGINE_TREE_PHYSICS_FULL_WRITES") ? "\n#define EE_TREE_PHYSICS_FULL_WRITES 1\n" : "";
  const auto pipeline = [&](uint32_t pass) {
    auto result = std::make_shared<ComputePipeline>();
    result->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute,
                                Platform::GetShaderGlobalDefines() + write_define + reference_define +
                                    "\n#define TREE_PHYSICS_PASS " + std::to_string(pass) + "\n",
                                "./EcoSysLabResources/Shaders/Compute/TreePhysics.slang");
    result->descriptor_set_layouts = {layout_, wind_layout_};
    result->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(Parameters)});
    result->Initialize();
    return result;
  };
  predict_ = pipeline(0);
  solve_position_ = pipeline(1);
  solve_rotation_ = pipeline(2);
  update_velocity_ = pipeline(3);
  prepare_coefficients_ = pipeline(4);
  return predict_->Initialized() && solve_position_->Initialized() && solve_rotation_->Initialized() &&
         update_velocity_->Initialized() && prepare_coefficients_->Initialized();
}

bool GpuTreePhysics::Initialize(const DynamicTreeSkeleton::GpuRestState& state) {
  return Initialize(std::vector<DynamicTreeSkeleton::GpuRestState>{state});
}
bool GpuTreePhysics::Initialize(const std::vector<DynamicTreeSkeleton::GpuRestState>& states) {
  if (states.empty() || !Prepare())
    return false;
  for (const auto& state : states)
    if (state.vertices.empty() || state.rods.empty())
      return false;
  for (uint32_t i = 0; i < trees_.size(); ++i)
    RemoveTree(i);
  trees_.clear();
  active_ids_.clear();
  std::vector<uint32_t> ids;
  for (const auto& state : states)
    ids.push_back(RegisterTree(state));
  SetActiveTrees(ids);
  return true;
}
uint32_t GpuTreePhysics::RegisterTree(const DynamicTreeSkeleton::GpuRestState& rest, uint32_t id) {
  if (rest.vertices.empty() || rest.rods.empty() || !Prepare())
    return UINT32_MAX;
  if (id == UINT32_MAX) {
    id = static_cast<uint32_t>(trees_.size());
    trees_.push_back({});
  }
  if (trees_[id])
    retired_states_.emplace_back(Platform::GetFrameCount(), trees_[id]);
  auto state = std::make_shared<GpuTreePhysicsState>();
  state->attribute_arrays = !reference_;
  state->vertices = {MakeAttributeBuffer(rest.vertices, !reference_), MakeAttributeBuffer(rest.vertices, !reference_)};
  state->rods = {MakeAttributeBuffer(rest.rods, !reference_), MakeAttributeBuffer(rest.rods, !reference_)};
  state->vertex_coefficients = MakeBuffer(std::vector<glm::vec2>(rest.vertices.size()));
  state->rod_coefficients = MakeBuffer(std::vector<glm::vec2>(rest.rods.size()));
  state->angle_coefficients = MakeBuffer(std::vector<float>(rest.angles.size()));
  state->angle_count = static_cast<uint32_t>(rest.angles.size());
  state->angles = MakeBuffer(rest.angles);
  state->vertex_rods = MakeBuffer(rest.vertex_rods);
  state->rod_angles = MakeBuffer(rest.rod_angles);
  state->node_rods = MakeBuffer(rest.node_rods);
  state->vertex_count = static_cast<uint32_t>(rest.vertices.size());
  state->rod_count = static_cast<uint32_t>(rest.rods.size());
  state->node_count = static_cast<uint32_t>(rest.node_rods.size());
  state->predicted = MakeBuffer(std::vector<glm::vec4>(state->vertex_count));
  state->previous = MakeBuffer(std::vector<glm::vec4>(state->vertex_count));
  state->previous_model = rest.initial_model;
  GlobalTransform transform;
  transform.value = rest.initial_model;
  state->initial_rotation = transform.GetRotation();
  trees_[id] = std::move(state);
  groups_dirty_ = true;
  return id;
}
void GpuTreePhysics::RemoveTree(uint32_t id) {
  if (id >= trees_.size() || !trees_[id])
    return;
  retired_states_.emplace_back(Platform::GetFrameCount(), std::move(trees_[id]));
  active_ids_.erase(std::remove(active_ids_.begin(), active_ids_.end(), id), active_ids_.end());
  groups_dirty_ = true;
}
void GpuTreePhysics::SetActiveTrees(const std::vector<uint32_t>& ids) {
  if (active_ids_ != ids) {
    active_ids_ = ids;
    groups_dirty_ = true;
  }
}
void GpuTreePhysics::PrepareRegistry() {
  const uint32_t frame = Platform::GetFrameCount();
  const auto expired = [&](const auto& entry) {
    return frame - entry.first > Platform::GetMaxFramesInFlight() + 1;
  };
  retired_states_.erase(std::remove_if(retired_states_.begin(), retired_states_.end(), expired), retired_states_.end());
  retired_buffers_.erase(std::remove_if(retired_buffers_.begin(), retired_buffers_.end(), expired),
                         retired_buffers_.end());
  retired_sets_.erase(std::remove_if(retired_sets_.begin(), retired_sets_.end(), expired), retired_sets_.end());
  std::vector<TreeAddresses> addresses(trees_.size());
  for (uint32_t id = 0; id < trees_.size(); ++id) {
    if (!trees_[id])
      continue;
    const auto& t = *trees_[id];
    auto& a = addresses[id];
    for (uint32_t i = 0; i < 2; ++i) {
      a.vertices[i] = t.vertices[t.vertex_index ^ i]->GetDeviceAddress();
      a.rods[i] = t.rods[t.rod_index ^ i]->GetDeviceAddress();
    }
    a.angles = t.angles->GetDeviceAddress();
    a.vertex_rods = t.vertex_rods->GetDeviceAddress();
    a.rod_angles = t.rod_angles->GetDeviceAddress();
    a.node_rods = t.node_rods->GetDeviceAddress();
    a.predicted = t.predicted->GetDeviceAddress();
    a.previous = t.previous->GetDeviceAddress();
    a.vertex_coefficients = t.vertex_coefficients->GetDeviceAddress();
    a.rod_coefficients = t.rod_coefficients->GetDeviceAddress();
    a.angle_coefficients = t.angle_coefficients->GetDeviceAddress();
    a.counts = {t.vertex_count, t.rod_count, t.angle_count, t.attribute_arrays ? 1u : 0u};
  }
  bool bindings_changed = false;
  const auto ensure = [&](auto& buffer, size_t bytes) {
    const auto required = std::max<size_t>(16, bytes);
    if (buffer && buffer->GetSize() >= required)
      return;
    const auto capacity = buffer ? std::max<VkDeviceSize>(required, buffer->GetSize() * 2) : required;
    if (buffer)
      retired_buffers_.emplace_back(frame, buffer);
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = capacity;
    info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    VmaAllocationCreateInfo allocation{};
    allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    buffer = std::make_shared<Buffer>(info, allocation);
    bindings_changed = true;
  };
  BufferUploadBatch upload;
  std::vector<glm::uvec2> vertices, rods, coefficients;
  if (groups_dirty_) {
    for (const auto id : active_ids_) {
      if (!trees_[id])
        continue;
      const auto& t = *trees_[id];
      for (uint32_t base = 0; base < std::max({t.vertex_count, t.rod_count, t.angle_count}); base += 64)
        coefficients.emplace_back(id, base);
      for (uint32_t base = 0; base < t.vertex_count; base += 64)
        vertices.emplace_back(id, base);
      for (uint32_t base = 0; base < t.rod_count; base += 64)
        rods.emplace_back(id, base);
    }
    vertex_group_count_ = static_cast<uint32_t>(vertices.size());
    rod_group_count_ = static_cast<uint32_t>(rods.size());
    ensure(vertex_groups_, vertices.size() * sizeof(glm::uvec2));
    ensure(rod_groups_, rods.size() * sizeof(glm::uvec2));
    if (!vertices.empty())
      upload.AddVector(vertex_groups_, vertices,
                       {BufferUploadUsage::StorageRead, BufferUploadCapacityPolicy::GrowIfNeeded});
    if (!rods.empty())
      upload.AddVector(rod_groups_, rods, {BufferUploadUsage::StorageRead, BufferUploadCapacityPolicy::GrowIfNeeded});
    coefficient_group_count_ = static_cast<uint32_t>(coefficients.size());
    ensure(coefficient_groups_, coefficients.size() * sizeof(glm::uvec2));
    if (!coefficients.empty())
      upload.AddVector(coefficient_groups_, coefficients,
                       {BufferUploadUsage::StorageRead, BufferUploadCapacityPolicy::GrowIfNeeded});
    groups_dirty_ = false;
  }
  ensure(registry_, addresses.size() * sizeof(TreeAddresses));
  ensure(tree_parameters_, trees_.size() * sizeof(TreeParameters));
  if (cached_addresses_.size() != addresses.size() ||
      (!addresses.empty() &&
       std::memcmp(cached_addresses_.data(), addresses.data(), addresses.size() * sizeof(TreeAddresses)))) {
    if (!addresses.empty())
      upload.AddVector(registry_, addresses,
                       {BufferUploadUsage::StorageRead, BufferUploadCapacityPolicy::GrowIfNeeded});
    cached_addresses_ = std::move(addresses);
  }
  upload.Record(upload_arena_);
  if (bindings_changed || !descriptor_set_) {
    if (descriptor_set_)
      retired_sets_.emplace_back(frame, descriptor_set_);
    descriptor_set_ = std::make_shared<DescriptorSet>(layout_);
    descriptor_set_->UpdateBufferDescriptorBinding(0, registry_);
    descriptor_set_->UpdateBufferDescriptorBinding(1, vertex_groups_);
    descriptor_set_->UpdateBufferDescriptorBinding(2, rod_groups_);
    descriptor_set_->UpdateBufferDescriptorBinding(3, tree_parameters_);
    descriptor_set_->UpdateBufferDescriptorBinding(4, coefficient_groups_);
  }
}

void GpuTreePhysics::Step(const DynamicTreeSkeleton::PhysicsSettings& settings) {
  if (active_ids_.empty())
    return;
  GlobalTransform model;
  model.value = trees_[active_ids_[0]]->previous_model;
  Step(std::vector<TreeStep>{{settings, true, model}});
}
void GpuTreePhysics::Step(const std::vector<TreeStep>& trees) {
  Step(trees, WindInput{});
}
void GpuTreePhysics::Step(const std::vector<TreeStep>& trees, const WindInput& wind) {
  const ProfilerScope scope("EcoSysLab / Tree Solver", "EcoSysLab");
  if (trees.empty() || trees.size() != active_ids_.size() || !Prepare())
    return;
  PrepareRegistry();
  int substeps = 0, iterations = 0;
  bool coefficients_dirty = false;
  std::vector<TreeParameters> tree_parameters(trees_.size());
  for (size_t i = 0; i < trees.size(); ++i) {
    const auto id = active_ids_[i];
    auto& state = *trees_[id];
    const auto& tree = trees[i];
    const auto& settings = tree.settings;
    const int tree_substeps = glm::clamp(settings.substeps, 1, 64),
              tree_iterations = glm::clamp(settings.iterations, 1, 64);
    if (tree.running) {
      substeps = glm::max(substeps, tree_substeps);
      iterations = glm::max(iterations, tree_iterations);
    } else if (tree.model.value != state.previous_model) {
      substeps = glm::max(substeps, 1);
      iterations = glm::max(iterations, 1);
    }
    GlobalTransform previous;
    previous.value = state.previous_model;
    const auto rotation = tree.model.GetRotation();
    const auto previous_rotation = previous.GetRotation();
    const auto relative_previous = glm::normalize(previous_rotation * glm::inverse(state.initial_rotation));
    const auto relative_current = glm::normalize(rotation * glm::inverse(state.initial_rotation));
    const auto delta_rotation = glm::normalize(rotation * glm::inverse(previous_rotation));
    tree_parameters[id] = {glm::vec4(glm::clamp(settings.time_step, 1e-5f, 0.1f) / tree_substeps,
                                     glm::max(0.0f, settings.stretch_stiffness),
                                     glm::max(0.0f, settings.bend_stiffness), glm::max(0.0f, settings.damping)),
                           glm::vec4(settings.gravity, glm::max(0.0f, settings.wind_drag)),
                           glm::uvec4(tree_substeps, tree_iterations, tree.running ? 1u : 0u,
                                      tree.model.value != state.previous_model ? 1u : 0u),
                           state.previous_model,
                           tree.model.value,
                           tree.model.value * glm::inverse(state.previous_model),
                           {relative_previous.x, relative_previous.y, relative_previous.z, relative_previous.w},
                           {relative_current.x, relative_current.y, relative_current.z, relative_current.w},
                           {delta_rotation.x, delta_rotation.y, delta_rotation.z, delta_rotation.w},
                           glm::vec4(settings.anchor_offset, 0.0f)};
    if (!reference_ &&
        (!state.coefficients_ready || state.coefficient_mechanics != glm::vec3(tree_parameters[id].mechanics))) {
      tree_parameters[id].control.w |= 2u;
      coefficients_dirty = true;
    }
  }

  if (!substeps)
    return;
  BufferUploadBatch upload;
  upload.AddVector(tree_parameters_, tree_parameters,
                   {BufferUploadUsage::StorageRead, BufferUploadCapacityPolicy::GrowIfNeeded});
  upload.Record(upload_arena_);
  const auto wind_set = wind.descriptor_set ? wind.descriptor_set : empty_wind_set_;
  const bool phase_timings = std::getenv("EVOENGINE_TREE_PHYSICS_PHASE_TIMINGS") != nullptr;
  uint32_t vertex_index = 0, rod_index = 0;
  Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
    const auto timing = Platform::BeginGpuTimestampScope(
        command, {"EcoSysLabTreePhysics", "EcoSysLab Tree Physics", "EcoSysLab", GpuTimestampQueue::Compute});
    Parameters parameters{0, 0, 0, 0, wind.descriptor_set ? wind.cell_count : 0, wind.bucket_count, wind.cell_size, 0};
    static const GpuTimestampScopeMetadata phases[] = {
        {"EcoSysLabTreePredict", "EcoSysLab Tree Predict", "EcoSysLab", GpuTimestampQueue::Compute},
        {"EcoSysLabTreePosition", "EcoSysLab Tree Position", "EcoSysLab", GpuTimestampQueue::Compute},
        {"EcoSysLabTreeRotation", "EcoSysLab Tree Rotation", "EcoSysLab", GpuTimestampQueue::Compute},
        {"EcoSysLabTreeVelocity", "EcoSysLab Tree Velocity", "EcoSysLab", GpuTimestampQueue::Compute}};
    const auto dispatch = [&](const auto& pipeline, uint32_t count, uint32_t phase) {
      const auto phase_timing =
          phase_timings ? Platform::BeginGpuTimestampScope(command, phases[phase]) : GpuTimestampScopeToken{};
      parameters.vertex_index = vertex_index;
      parameters.rod_index = rod_index;
      pipeline->Bind(command);
      pipeline->BindDescriptorSet(command, 0, descriptor_set_->GetVkDescriptorSet());
      pipeline->BindDescriptorSet(command, 1, wind_set->GetVkDescriptorSet());
      pipeline->PushConstant(command, 0, parameters);
      pipeline->Dispatch(command, count, 1, 1);
      if (reference_)
        Platform::EverythingBarrier(command);
      else
        Platform::ComputeBarrier(command);
      if (phase_timings)
        Platform::EndGpuTimestampScope(command, phase_timing);
    };
    if (coefficients_dirty) {
      prepare_coefficients_->Bind(command);
      prepare_coefficients_->BindDescriptorSet(command, 0, descriptor_set_->GetVkDescriptorSet());
      prepare_coefficients_->BindDescriptorSet(command, 1, wind_set->GetVkDescriptorSet());
      prepare_coefficients_->PushConstant(command, 0, parameters);
      prepare_coefficients_->Dispatch(command, coefficient_group_count_, 1, 1);
      Platform::ComputeBarrier(command);
    }
    for (int substep = 0; substep < substeps; ++substep) {
      parameters.substep = substep;
      dispatch(predict_, vertex_group_count_, 0);
      vertex_index ^= 1;
      for (int iteration = 0; iteration < iterations; ++iteration) {
        parameters.iteration = iteration;
        dispatch(solve_position_, vertex_group_count_, 1);
        vertex_index ^= 1;
        dispatch(solve_rotation_, rod_group_count_, 2);
        rod_index ^= 1;
      }
      dispatch(update_velocity_, vertex_group_count_, 3);
      vertex_index ^= 1;
    }
    Platform::EverythingBarrier(command);
    Platform::EndGpuTimestampScope(command, timing);
  });
  for (size_t i = 0; i < trees.size(); ++i) {
    auto& state = *trees_[active_ids_[i]];
    state.previous_model = trees[i].model.value;
    state.coefficients_ready = !reference_;
    state.coefficient_mechanics = glm::vec3(tree_parameters[active_ids_[i]].mechanics);
    state.vertex_index ^= vertex_index;
    state.rod_index ^= rod_index;
  }
}

void GpuTreePhysics::DownloadVertices(std::vector<DynamicTreeSkeleton::GpuRestState::Vertex>& output, uint32_t id,
                                      uint32_t copy) const {
  const auto& t = *trees_[id];
  DownloadAttributes(t.vertices[copy == UINT32_MAX ? t.vertex_index : copy], t.vertex_count, t.attribute_arrays,
                     output);
}
void GpuTreePhysics::DownloadRods(std::vector<DynamicTreeSkeleton::GpuRestState::Rod>& output, uint32_t id,
                                  uint32_t copy) const {
  const auto& t = *trees_[id];
  DownloadAttributes(t.rods[copy == UINT32_MAX ? t.rod_index : copy], t.rod_count, t.attribute_arrays, output);
}
