#include "GpuTreeClusterPhysics.hpp"
#include <algorithm>
#include <glm/gtx/quaternion.hpp>
#include <unordered_set>
#include "GpuTreePhysicsBuffers.hpp"
#include "Shader.hpp"

using namespace eco_sys_lab_package;
using namespace eco_sys_lab_package::tree_physics_buffers;
namespace {
glm::vec4 Q(const glm::quat& q) {
  return {q.x, q.y, q.z, q.w};
}
}  // namespace

bool GpuTreeClusterPhysics::Initialize(const DynamicTreeSkeleton::GpuRestState& rest,
                                       const std::vector<TreeClusterRule>& rules,
                                       const std::shared_ptr<GpuTreePhysicsState>& state) {
  if (tree_)
    return false;
  const auto hierarchy = BuildTreeClusterHierarchy(rest);
  if (!hierarchy || hierarchy->candidates.empty())
    return false;
  auto selection = SelectTreeClusters(*hierarchy, rules);
  std::vector<TreeSubtreeBody> bodies;
  for (const auto& element : selection.elements) {
    std::vector<uint32_t> members(hierarchy->preorder.begin() + element.begin,
                                  hierarchy->preorder.begin() + element.end);
    // Interior fixed bodies create additional boundary conditions beyond a rooted articulated tree.
    if (element.parent != UINT32_MAX && std::any_of(members.begin(), members.end(), [&](uint32_t id) {
          return rest.rods[id].links.w != 0;
        }))
      return false;
    auto body = BuildTreeRigidBody(rest, members);
    if (!body)
      return false;
    bodies.push_back(std::move(*body));
  }
  layout_ = std::make_shared<DescriptorSetLayout>();
  for (uint32_t i = 0; i < 8; ++i)
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
  for (uint32_t pass = 0; pass < pipelines_.size(); ++pass) {
    auto& pipeline = pipelines_[pass];
    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute,
        Platform::GetShaderGlobalDefines() + "\n#define CLUSTER_PASS " + std::to_string(pass) + "\n",
        "./EcoSysLabResources/Shaders/Compute/TreeClusterPhysics.slang");
    pipeline->descriptor_set_layouts = {layout_, wind_layout_};
    pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(Parameters)});
    pipeline->Initialize();
    if (!pipeline->Initialized())
      return false;
  }
  bodies_ = std::move(bodies);
  selection_ = std::move(selection);
  GlobalTransform initial;
  initial.value = rest.initial_model;
  initial_position_ = initial.GetPosition();
  initial_rotation_ = initial.GetRotation();
  std::vector<Body> properties;
  std::vector<Motion> motions;
  std::vector<TreeSubtreeBody::Rod> rods;
  std::vector<TreeSubtreeBody::Vertex> vertices;
  std::vector<bool> owned(rest.vertices.size(), false);
  for (uint32_t id = 0; id < bodies_.size(); ++id) {
    const auto& body = bodies_[id];
    const auto parent = selection_.elements[id].parent;
    Body p;
    p.center_mass = {body.center_of_mass, body.mass};
    p.inertia0 = {body.inertia_pivot[0], 0};
    p.inertia1 = {body.inertia_pivot[1], 0};
    p.inertia2 = {body.inertia_pivot[2], 0};
    p.rest_anchor = {body.anchor, 0};
    p.rest_rotation = Q(body.rotation);
    p.links = {rods.size(), body.rods.size(), parent, parent == UINT32_MAX ? 1u : 0u};
    if (parent != UINT32_MAX) {
      p.parent_offset = {glm::inverse(bodies_[parent].rotation) * (body.anchor - bodies_[parent].anchor), 0};
      p.relative_rotation = Q(glm::inverse(bodies_[parent].rotation) * body.rotation);
    }
    properties.push_back(p);
    Motion motion;
    motion.rotation = p.rest_rotation;
    motion.anchor = p.rest_anchor;
    motions.push_back(motion);
    for (auto rod : body.rods) {
      rod.indices.w = id;
      rods.push_back(rod);
    }
    for (auto vertex : body.vertices) {
      if (owned[vertex.indices.x])
        continue;
      owned[vertex.indices.x] = true;
      vertex.indices.w = id;
      vertices.push_back(vertex);
    }
  }
  std::vector<uint32_t> depths(bodies_.size());
  std::vector<glm::uvec4> schedule(bodies_.size(), glm::uvec4(UINT32_MAX));
  for (uint32_t i = 0; i < bodies_.size(); ++i) {
    const auto parent = selection_.elements[i].parent;
    if (parent != UINT32_MAX) {
      depths[i] = depths[parent] + 1;
      schedule[i].z = schedule[parent].y;
      schedule[parent].y = i;
    }
    if (levels_.size() <= depths[i])
      levels_.resize(depths[i] + 1);
    ++levels_[depths[i]].y;
  }
  uint32_t offset = 0;
  for (auto& level : levels_) {
    level.x = offset;
    offset += level.y;
  }
  std::vector<uint32_t> cursors(levels_.size());
  for (uint32_t i = 0; i < bodies_.size(); ++i)
    schedule[levels_[depths[i]].x + cursors[depths[i]]++].x = i;
  schedule_ = MakeBuffer(schedule);
  std::vector<glm::uvec4> ranges;
  for (const auto& level : levels_)
    ranges.emplace_back(level, 0, 0);
  level_ranges_ = MakeBuffer(ranges);
  contributions_ = Allocate(14 * sizeof(glm::vec4) * bodies_.size());
  tree_ = state ? state : MakeGpuTreePhysicsState(rest, true);
  properties_ = MakeBuffer(properties);
  motion_ = MakeBuffer(motions);
  rods_ = MakeBuffer(rods);
  rod_loads_ = Allocate(2 * sizeof(glm::vec4) * rods.size());
  vertices_ = MakeBuffer(vertices);
  registry_ = MakeBuffer(std::vector<GpuTreePhysicsAddresses>{GetGpuTreePhysicsAddresses(*tree_)});
  work_ = Allocate(14 * sizeof(glm::vec4) * bodies_.size());
  set_ = std::make_shared<DescriptorSet>(layout_);
  const std::array buffers{properties_, motion_, rods_, vertices_, registry_, work_, schedule_, contributions_};
  for (uint32_t i = 0; i < buffers.size(); ++i)
    set_->UpdateBufferDescriptorBinding(i, buffers[i]);
  rod_count_ = static_cast<uint32_t>(rods.size());
  vertex_count_ = static_cast<uint32_t>(vertices.size());
  return true;
}

GpuTreeClusterPhysics::Parameters GpuTreeClusterPhysics::MakeParameters(const Settings& settings,
                                                                        const GlobalTransform& model,
                                                                        const GpuTreePhysics::WindInput& wind) const {
  Parameters p;
  const auto delta = glm::normalize(model.GetRotation() * glm::inverse(initial_rotation_));
  p.rotation = Q(delta);
  p.translation = model.GetPosition() - delta * initial_position_;
  p.gravity = settings.gravity;
  p.wind_density = {settings.wind_velocity, std::max(0.0f, settings.air_density)};
  p.stiffness_damping = {glm::max(settings.angular_stiffness, glm::vec3(0)), std::max(0.0f, settings.damping)};
  p.base_stiffness = glm::max(settings.base_angular_stiffness, glm::vec3(0));
  if (wind.descriptor_set && wind.bucket_count && wind.cell_size > 0) {
    p.wind_cell_count = wind.cell_count;
    p.wind_bucket_count = wind.bucket_count;
    p.wind_cell_size = wind.cell_size;
  }
  p.mechanics = {settings.time_step, std::max(0.0f, settings.wind_drag)};
  p.counts = {bodies_.size(), rod_count_, vertex_count_, glm::clamp(settings.substeps, 1u, 64u)};
  p.level_begin = tree_->vertex_index;
  p.substep = tree_->rod_index;
  return p;
}

std::array<uint64_t, 18> GpuTreeClusterPhysics::Addresses() const {
  std::array<uint64_t, 18> result{properties_->GetDeviceAddress(), motion_->GetDeviceAddress(),
                                  rods_->GetDeviceAddress(),       vertices_->GetDeviceAddress(),
                                  registry_->GetDeviceAddress(),   work_->GetDeviceAddress(),
                                  schedule_->GetDeviceAddress(),   contributions_->GetDeviceAddress()};
  for (size_t i = 0; i < lod_buffers_.size(); ++i)
    if (lod_buffers_[i])
      result[8 + i] = lod_buffers_[i]->GetDeviceAddress();
  result[16] = level_ranges_->GetDeviceAddress();
  result[17] = rod_loads_->GetDeviceAddress();
  return result;
}

bool GpuTreeClusterPhysics::InitializeDynamic(const DynamicTreeSkeleton::GpuRestState& rest,
                                              const std::shared_ptr<GpuTreePhysicsState>& state) {
  if (!Initialize(rest, {}, state))
    return false;
  const auto hierarchy = BuildTreeClusterHierarchy(rest);
  struct Candidate {
    glm::uvec4 links;
    glm::vec4 bounds;
  };
  std::vector<Candidate> candidates;
  for (const auto& c : hierarchy->candidates)
    candidates.push_back({{c.parent, c.begin, c.end, c.branch_order},
                          {c.extent / hierarchy->tree_height, c.attachment_radius / hierarchy->trunk_radius,
                           c.collapsible ? 1.0f : 0.0f, 0}});
  std::vector<Body> original(rest.rods.size());
  for (size_t i = 0; i < bodies_.size(); ++i) {
    auto& body = original[selection_.elements[i].root_rod];
    body.rest_anchor = {bodies_[i].anchor, 0};
    body.rest_rotation = Q(bodies_[i].rotation);
    body.links.w = selection_.elements[i].parent == UINT32_MAX ? 1 : 0;
  }
  for (const auto& angle : rest.angles)
    original[angle.rods.y].rest_anchor.w =
        angle.bend_scale.x / std::max(0.5f * (rest.rods[angle.rods.x].length_scale_mass_radius.x +
                                              rest.rods[angle.rods.y].length_scale_mass_radius.x),
                                      1e-6f);
  // State header, prior rule values and material padding; updated only by the GPU.
  lod_buffers_[0] = MakeBuffer(std::vector<glm::uvec4>(6));
  lod_buffers_[1] = MakeBuffer(candidates);
  lod_buffers_[2] = MakeBuffer(hierarchy->preorder);
  lod_buffers_[3] = MakeBuffer(original);
  lod_buffers_[4] = MakeBuffer(selection_.rod_elements);
  lod_buffers_[5] = Allocate(sizeof(uint32_t) * rest.rods.size());
  lod_buffers_[6] = Allocate(sizeof(Motion) * rest.rods.size());
  lod_buffers_[7] = Allocate(sizeof(glm::uvec4) * rest.rods.size());
  level_ranges_ = Allocate(sizeof(glm::uvec4) * rest.rods.size());
  vertices_ = Allocate(sizeof(TreeSubtreeBody::Vertex) * rest.vertices.size());
  vertex_count_ = static_cast<uint32_t>(rest.vertices.size());
  set_->UpdateBufferDescriptorBinding(3, vertices_);
  return true;
}

void GpuTreeClusterPhysics::Step(const Settings& settings, const GlobalTransform& model, ExecutionMode execution,
                                 const GpuTreePhysics::WindInput& wind) {
  if (!tree_ || settings.time_step <= 0)
    return;
  auto p = MakeParameters(settings, model, wind);
  Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
    const auto timing = Platform::BeginGpuTimestampScope(
        command, {"EcoSysLabClusterPhysics", "EcoSysLab Cluster Physics", "EcoSysLab", GpuTimestampQueue::Compute});
    const auto dispatch = [&](uint32_t pass, uint32_t count) {
      pipelines_[pass]->Bind(command);
      pipelines_[pass]->BindDescriptorSet(command, 0, set_->GetVkDescriptorSet());
      pipelines_[pass]->BindDescriptorSet(
          command, 1, (wind.descriptor_set ? wind.descriptor_set : empty_wind_set_)->GetVkDescriptorSet());
      pipelines_[pass]->PushConstant(command, 0, p);
      pipelines_[pass]->Dispatch(command, (count + 63) / 64, 1, 1);
      Platform::ComputeBarrier(command);
    };
    if (execution == ExecutionMode::Serial) {
      dispatch(0, 1);
    } else {
      for (p.substep = 0; p.substep < p.counts.w; ++p.substep) {
        for (uint32_t pass = 2; pass < 6; ++pass) {
          for (uint32_t depth = 0; depth < levels_.size(); ++depth) {
            const auto level = levels_[pass == 3 ? levels_.size() - 1 - depth : depth];
            p.level_begin = level.x;
            p.counts.x = level.y;
            dispatch(pass, level.y);
          }
        }
      }
    }
    dispatch(1, std::max(rod_count_, vertex_count_));
    Platform::EndGpuTimestampScope(command, timing);
    Platform::EverythingBarrier(command);
  });
}

bool GpuTreeClusterForest::Initialize(const std::vector<std::shared_ptr<GpuTreeClusterPhysics>>& trees) {
  if (!trees_.empty() || trees.empty() || trees.size() > 65535)
    return false;
  std::unordered_set<const GpuTreeClusterPhysics*> unique;
  for (const auto& tree : trees)
    if (!tree || !tree->tree_ || !unique.insert(tree.get()).second)
      return false;
  layout_ = std::make_shared<DescriptorSetLayout>();
  for (uint32_t i = 0; i < 3; ++i)
    layout_->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout_->Initialize();
  for (uint32_t pass = 0; pass < pipelines_.size(); ++pass) {
    auto& pipeline = pipelines_[pass];
    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = Shader::CreateTemporary(ShaderType::Compute,
                                                       Platform::GetShaderGlobalDefines() +
                                                           "\n#define CLUSTER_FOREST 1\n#define CLUSTER_PASS " +
                                                           std::to_string(pass < 2 ? pass : pass + 4) + "\n",
                                                       "./EcoSysLabResources/Shaders/Compute/TreeClusterPhysics.slang");
    pipeline->descriptor_set_layouts = {layout_, trees[0]->wind_layout_};
    pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(glm::uvec4)});
    pipeline->Initialize();
    if (!pipeline->Initialized())
      return false;
  }
  trees_ = trees;
  records_.resize(trees.size());
  for (size_t i = 0; i < trees.size(); ++i) {
    records_[i].addresses = trees[i]->Addresses();
    reconstruction_count_ = std::max(reconstruction_count_, std::max(trees[i]->rod_count_, trees[i]->vertex_count_));
  }
  uint32_t job_capacity = 0;
  for (const auto& tree : trees)
    job_capacity += (tree->rod_count_ + 63) / 64;
  transition_jobs_ = Allocate(sizeof(glm::uvec2) * job_capacity);
  transition_dispatch_ = Allocate(sizeof(glm::uvec4), VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT);
  records_buffer_ = MakeBuffer(records_);
  set_ = std::make_shared<DescriptorSet>(layout_);
  set_->UpdateBufferDescriptorBinding(0, records_buffer_);
  set_->UpdateBufferDescriptorBinding(1, transition_jobs_);
  set_->UpdateBufferDescriptorBinding(2, transition_dispatch_);
  return true;
}

void GpuTreeClusterForest::Step(const std::vector<TreeStep>& steps, const GpuTreePhysics::WindInput& wind) {
  if (trees_.empty() || steps.size() != trees_.size())
    return;
  bool active = false, dynamic_lod = false;
  uint32_t substeps = 0;
  for (size_t i = 0; i < steps.size(); ++i) {
    records_[i].parameters = trees_[i]->MakeParameters(steps[i].settings, steps[i].model, wind);
    if (!steps[i].running || steps[i].settings.time_step <= 0)
      records_[i].parameters.counts.w = 0;
    records_[i].lod.control = {steps[i].lod.enabled_policies, steps[i].pose_revision,
                               static_cast<uint32_t>(trees_[i]->levels_.size()),
                               static_cast<uint32_t>(steps[i].scheduling)};
    records_[i].lod.rules = steps[i].lod.rules;
    records_[i].lod.material = {std::max(steps[i].mass_scale, 1e-6f), 0, 0, 0};
    substeps = std::max(substeps, records_[i].parameters.counts.w);
    active |= records_[i].parameters.counts.w != 0 || records_[i].addresses[8] != 0;
    dynamic_lod |= records_[i].addresses[8] != 0;
  }
  if (!active)
    return;
  BufferUploadBatch upload;
  upload.AddVector(records_buffer_, records_,
                   {BufferUploadUsage::StorageReadWrite, BufferUploadCapacityPolicy::GrowIfNeeded});
  upload.Record(upload_arena_);
  Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
    const auto timing = Platform::BeginGpuTimestampScope(
        command, {"EcoSysLabClusterForest", "EcoSysLab Cluster Forest", "EcoSysLab", GpuTimestampQueue::Compute});
    uint32_t substep = 0;
    const auto dispatch = [&](uint32_t pass, uint32_t x, uint32_t y = 1, bool indirect = false) {
      auto& pipeline = pipelines_[pass];
      pipeline->Bind(command);
      pipeline->BindDescriptorSet(command, 0, set_->GetVkDescriptorSet());
      pipeline->BindDescriptorSet(
          command, 1, (wind.descriptor_set ? wind.descriptor_set : trees_[0]->empty_wind_set_)->GetVkDescriptorSet());
      pipeline->PushConstant(command, 0, glm::uvec4(trees_.size(), substep, 0, 0));
      if (indirect)
        pipeline->DispatchIndirect(command, *transition_dispatch_);
      else
        pipeline->Dispatch(command, x, y, 1);
      Platform::ComputeBarrier(command);
    };
    if (dynamic_lod) {
      dispatch(2, 1);
      dispatch(3, (static_cast<uint32_t>(trees_.size()) + 63) / 64);
      Platform::EverythingBarrier(command);
      dispatch(4, 0, 1, true);
      dispatch(5, 0, 1, true);
    }
    for (substep = 0; substep < substeps; ++substep) {
      dispatch(7, (reconstruction_count_ + 63) / 64, static_cast<uint32_t>(trees_.size()));
      dispatch(0, (static_cast<uint32_t>(trees_.size()) + 63) / 64);
      dispatch(6, static_cast<uint32_t>(trees_.size()));
    }
    substep = 0;
    dispatch(1, (reconstruction_count_ + 63) / 64, static_cast<uint32_t>(trees_.size()));
    Platform::EndGpuTimestampScope(command, timing);
    Platform::EverythingBarrier(command);
  });
}
