#include "GpuTreePhysics.hpp"

#include "Shader.hpp"

using namespace eco_sys_lab_package;

namespace {
template <typename T>
std::shared_ptr<Buffer> MakeBuffer(const std::vector<T>& source) {
  const std::vector<T> data = source.empty() ? std::vector<T>(1) : source;
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = data.size() * sizeof(T);
  info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation{};
  allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  auto buffer = std::make_shared<Buffer>(info, allocation);
  buffer->UploadVector(data);
  return buffer;
}
}  // namespace

GpuTreePhysics::PackedRestState GpuTreePhysics::PackRestStates(
    const std::vector<DynamicTreeSkeleton::GpuRestState>& states) {
  PackedRestState packed;
  for (uint32_t tree = 0; tree < states.size(); ++tree) {
    const auto& source = states[tree];
    if (source.vertices.empty() || source.rods.empty())
      return {};
    auto& target = packed.state;
    const TreeRange range{static_cast<uint32_t>(target.vertices.size()), static_cast<uint32_t>(source.vertices.size()),
                          static_cast<uint32_t>(target.rods.size()), static_cast<uint32_t>(source.rods.size()),
                          static_cast<uint32_t>(target.node_rods.size())};
    const auto vertex_rod_base = static_cast<uint32_t>(target.vertex_rods.size());
    const auto rod_angle_base = static_cast<uint32_t>(target.rod_angles.size());
    const auto angle_base = static_cast<uint32_t>(target.angles.size());
    for (auto vertex : source.vertices) {
      vertex.incident_rods.x += vertex_rod_base;
      vertex.incident_rods.z = tree;
      target.vertices.push_back(vertex);
    }
    for (auto rod : source.rods) {
      rod.links.x += range.vertex_base;
      rod.links.y += range.vertex_base;
      rod.incident_angles.x += rod_angle_base;
      rod.incident_angles.z = tree;
      target.rods.push_back(rod);
    }
    for (auto angle : source.angles) {
      angle.rods.x += range.rod_base;
      angle.rods.y += range.rod_base;
      target.angles.push_back(angle);
    }
    for (const auto rod : source.vertex_rods)
      target.vertex_rods.push_back(rod + range.rod_base);
    for (const auto angle : source.rod_angles)
      target.rod_angles.push_back(angle + angle_base);
    for (const auto rod : source.node_rods)
      target.node_rods.push_back(rod == UINT32_MAX ? rod : rod + range.rod_base);
    packed.ranges.push_back(range);
  }
  return packed;
}

bool GpuTreePhysics::Initialize(const DynamicTreeSkeleton::GpuRestState& rest_state) {
  return Initialize(std::vector<DynamicTreeSkeleton::GpuRestState>{rest_state});
}

bool GpuTreePhysics::Initialize(const std::vector<DynamicTreeSkeleton::GpuRestState>& rest_states) {
  auto packed = PackRestStates(rest_states);
  if (packed.ranges.size() != rest_states.size() || packed.ranges.empty())
    return false;
  ranges_ = std::move(packed.ranges);
  previous_models_.clear();
  initial_rotations_.clear();
  for (const auto& state : rest_states) {
    previous_models_.push_back(state.initial_model);
    GlobalTransform model;
    model.value = state.initial_model;
    initial_rotations_.push_back(model.GetRotation());
  }
  const auto& rest_state = packed.state;
  vertex_count_ = static_cast<uint32_t>(rest_state.vertices.size());
  rod_count_ = static_cast<uint32_t>(rest_state.rods.size());
  vertex_index_ = rod_index_ = 0;
  vertices_[0] = MakeBuffer(rest_state.vertices);
  vertices_[1] = MakeBuffer(rest_state.vertices);
  rods_[0] = MakeBuffer(rest_state.rods);
  rods_[1] = MakeBuffer(rest_state.rods);
  angles_ = MakeBuffer(rest_state.angles);
  vertex_rods_ = MakeBuffer(rest_state.vertex_rods);
  rod_angles_ = MakeBuffer(rest_state.rod_angles);
  node_rods_ = MakeBuffer(rest_state.node_rods);
  predicted_ = MakeBuffer(std::vector<glm::vec4>(vertex_count_));
  previous_ = MakeBuffer(std::vector<glm::vec4>(vertex_count_));
  tree_parameters_ = MakeBuffer(std::vector<TreeParameters>(ranges_.size()));

  layout_ = std::make_shared<DescriptorSetLayout>();
  for (uint32_t i = 0; i < 10; ++i)
    layout_->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout_->Initialize();
  wind_layout_ = std::make_shared<DescriptorSetLayout>();
  wind_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  wind_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  wind_layout_->Initialize();
  empty_wind_set_ = std::make_shared<DescriptorSet>(wind_layout_);
  empty_wind_buffers_ = {MakeBuffer(std::vector<uint32_t>(1, UINT32_MAX)), MakeBuffer(std::vector<glm::u64vec2>(1))};
  for (uint32_t i = 0; i < empty_wind_buffers_.size(); ++i)
    empty_wind_set_->UpdateBufferDescriptorBinding(i, empty_wind_buffers_[i]);
  for (uint32_t vertex = 0; vertex < 2; ++vertex) {
    for (uint32_t rod = 0; rod < 2; ++rod) {
      auto& set = descriptor_sets_[vertex * 2 + rod];
      set = std::make_shared<DescriptorSet>(layout_);
      set->UpdateBufferDescriptorBinding(0, vertices_[vertex]);
      set->UpdateBufferDescriptorBinding(1, vertices_[1 - vertex]);
      set->UpdateBufferDescriptorBinding(2, rods_[rod]);
      set->UpdateBufferDescriptorBinding(3, rods_[1 - rod]);
      set->UpdateBufferDescriptorBinding(4, angles_);
      set->UpdateBufferDescriptorBinding(5, vertex_rods_);
      set->UpdateBufferDescriptorBinding(6, rod_angles_);
      set->UpdateBufferDescriptorBinding(7, predicted_);
      set->UpdateBufferDescriptorBinding(8, previous_);
      set->UpdateBufferDescriptorBinding(9, tree_parameters_);
    }
  }
  const auto make_pipeline = [&](const uint32_t pass) {
    auto pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute,
        Platform::GetShaderGlobalDefines() + "\n#define TREE_PHYSICS_PASS " + std::to_string(pass) + "\n",
        "./EcoSysLabResources/Shaders/Compute/TreePhysics.slang");
    pipeline->descriptor_set_layouts = {layout_, wind_layout_};
    pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(Parameters)});
    pipeline->Initialize();
    return pipeline;
  };
  predict_ = make_pipeline(0);
  solve_position_ = make_pipeline(1);
  solve_rotation_ = make_pipeline(2);
  update_velocity_ = make_pipeline(3);
  return predict_->Initialized() && solve_position_->Initialized() && solve_rotation_->Initialized() &&
         update_velocity_->Initialized();
}

void GpuTreePhysics::Step(const DynamicTreeSkeleton::PhysicsSettings& settings) {
  GlobalTransform model;
  if (!previous_models_.empty())
    model.value = previous_models_[0];
  Step(std::vector<TreeStep>{{settings, true, model}});
}

void GpuTreePhysics::Step(const std::vector<TreeStep>& trees) {
  Step(trees, WindInput{});
}

void GpuTreePhysics::Step(const std::vector<TreeStep>& trees, const WindInput& wind) {
  if (!vertex_count_ || !predict_ || !predict_->Initialized() || !solve_position_->Initialized() ||
      !solve_rotation_->Initialized() || !update_velocity_->Initialized() || trees.size() != ranges_.size())
    return;
  int substeps = 0;
  int iterations = 0;
  std::vector<TreeParameters> tree_parameters;
  tree_parameters.reserve(trees.size());
  for (size_t i = 0; i < trees.size(); ++i) {
    const auto& tree = trees[i];
    const auto& settings = tree.settings;
    const int tree_substeps = glm::clamp(settings.substeps, 1, 64);
    const int tree_iterations = glm::clamp(settings.iterations, 1, 64);
    if (tree.running) {
      substeps = glm::max(substeps, tree_substeps);
      iterations = glm::max(iterations, tree_iterations);
    } else if (tree.model.value != previous_models_[i]) {
      substeps = glm::max(substeps, 1);
      iterations = glm::max(iterations, 1);
    }
    GlobalTransform previous;
    previous.value = previous_models_[i];
    const auto rotation = tree.model.GetRotation();
    const auto previous_rotation = previous.GetRotation();
    const auto relative_previous = glm::normalize(previous_rotation * glm::inverse(initial_rotations_[i]));
    const auto relative_current = glm::normalize(rotation * glm::inverse(initial_rotations_[i]));
    const auto delta_rotation = glm::normalize(rotation * glm::inverse(previous_rotation));
    tree_parameters.push_back({glm::vec4(glm::clamp(settings.time_step, 1e-5f, 0.1f) / tree_substeps,
                                         glm::max(0.0f, settings.stretch_stiffness),
                                         glm::max(0.0f, settings.bend_stiffness), glm::max(0.0f, settings.damping)),
                               glm::vec4(settings.gravity, glm::max(0.0f, settings.wind_drag)),
                               glm::uvec4(tree_substeps, tree_iterations, tree.running ? 1u : 0u,
                                          tree.model.value != previous_models_[i] ? 1u : 0u),
                               previous_models_[i],
                               tree.model.value,
                               tree.model.value * glm::inverse(previous_models_[i]),
                               {relative_previous.x, relative_previous.y, relative_previous.z, relative_previous.w},
                               {relative_current.x, relative_current.y, relative_current.z, relative_current.w},
                               {delta_rotation.x, delta_rotation.y, delta_rotation.z, delta_rotation.w},
                               glm::vec4(settings.anchor_offset, 0.0f)});
  }
  if (!substeps)
    return;
  BufferUploadBatch upload;
  upload.AddVector(tree_parameters_, tree_parameters,
                   {BufferUploadUsage::StorageRead, BufferUploadCapacityPolicy::GrowIfNeeded});
  upload.Record(upload_arena_);
  const auto wind_set = wind.descriptor_set ? wind.descriptor_set : empty_wind_set_;
  const uint32_t cell_count = wind.descriptor_set ? wind.cell_count : 0;
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
    const auto timing = Platform::BeginGpuTimestampScope(
        command, {"EcoSysLabTreePhysics", "EcoSysLab Tree Physics", "EcoSysLab", GpuTimestampQueue::Compute});
    Parameters parameters{vertex_count_, rod_count_, 0, 0, cell_count, wind.bucket_count, wind.cell_size, 0.0f};
    const auto dispatch = [&](const std::shared_ptr<ComputePipeline>& pipeline, const uint32_t count) {
      pipeline->Bind(command);
      pipeline->BindDescriptorSet(command, 0, descriptor_sets_[vertex_index_ * 2 + rod_index_]->GetVkDescriptorSet());
      pipeline->BindDescriptorSet(command, 1, wind_set->GetVkDescriptorSet());
      pipeline->PushConstant(command, 0, parameters);
      pipeline->Dispatch(command, (count + 63) / 64, 1, 1);
      Platform::EverythingBarrier(command);
    };
    for (int step = 0; step < substeps; ++step) {
      parameters.substep = step;
      dispatch(predict_, vertex_count_);
      vertex_index_ ^= 1;
      for (int iteration = 0; iteration < iterations; ++iteration) {
        parameters.iteration = iteration;
        dispatch(solve_position_, vertex_count_);
        vertex_index_ ^= 1;
        dispatch(solve_rotation_, rod_count_);
        rod_index_ ^= 1;
      }
      dispatch(update_velocity_, vertex_count_);
      vertex_index_ ^= 1;
    }
    Platform::EndGpuTimestampScope(command, timing);
  });
  for (size_t i = 0; i < trees.size(); ++i)
    previous_models_[i] = trees[i].model.value;
}
