#include "GpuTreeSubtreePhysics.hpp"

#include <algorithm>
#include <glm/gtx/quaternion.hpp>
#include <unordered_set>
#include "GpuTreePhysicsBuffers.hpp"
#include "Shader.hpp"

using namespace eco_sys_lab_package;
using namespace eco_sys_lab_package::tree_physics_buffers;

namespace {
glm::mat3 ParallelAxis(const glm::vec3& offset) {
  return glm::dot(offset, offset) * glm::mat3(1) - glm::outerProduct(offset, offset);
}
glm::vec4 QuaternionVector(const glm::quat& q) {
  return {q.x, q.y, q.z, q.w};
}
std::optional<TreeSubtreeBody> BuildBody(const DynamicTreeSkeleton::GpuRestState& rest, uint32_t root_rod,
                                         uint32_t excluded = UINT32_MAX) {
  if (root_rod >= rest.rods.size())
    return {};
  std::vector<std::vector<uint32_t>> children(rest.vertices.size());
  for (uint32_t i = 0; i < rest.rods.size(); ++i)
    children[rest.rods[i].links.x].push_back(i);
  std::vector<uint32_t> queue{root_rod}, members;
  for (size_t i = 0; i < queue.size(); ++i) {
    const auto id = queue[i];
    if (id == excluded)
      continue;
    const auto& rod = rest.rods[id];
    if (rod.links.w != 0)
      return {};
    members.push_back(id);
    queue.insert(queue.end(), children[rod.links.y].begin(), children[rod.links.y].end());
  }
  return BuildTreeRigidBody(rest, members);
}

}  // namespace

std::optional<TreeSubtreeBody> eco_sys_lab_package::BuildTreeRigidBody(const DynamicTreeSkeleton::GpuRestState& rest,
                                                                       const std::vector<uint32_t>& members) {
  if (members.empty() || members.front() >= rest.rods.size())
    return {};
  TreeSubtreeBody body;
  const auto& root = rest.rods[members.front()];
  body.anchor = rest.vertices[root.links.x].position_inverse_mass;
  body.rotation = glm::normalize(glm::quat(root.rotation.w, root.rotation.x, root.rotation.y, root.rotation.z));
  const auto inverse_rotation = glm::inverse(body.rotation);
  std::unordered_set<uint32_t> included;
  for (const auto id : members) {
    if (id >= rest.rods.size())
      return {};
    const auto& rod = rest.rods[id];
    if (rod.length_scale_mass_radius.z <= 0 || rod.length_scale_mass_radius.w <= 0)
      return {};
    const glm::vec3 start =
        inverse_rotation * (glm::vec3(rest.vertices[rod.links.x].position_inverse_mass) - body.anchor);
    const glm::vec3 end =
        inverse_rotation * (glm::vec3(rest.vertices[rod.links.y].position_inverse_mass) - body.anchor);
    const float length = glm::length(end - start);
    if (length <= 0)
      return {};
    const float mass = rod.length_scale_mass_radius.z, radius = rod.length_scale_mass_radius.w;
    const auto relative = inverse_rotation * glm::quat(rod.rotation.w, rod.rotation.x, rod.rotation.y, rod.rotation.z);
    body.rods.push_back(
        {{start, radius}, {end, length}, QuaternionVector(relative), {id, rod.links.x, rod.links.y, 0}});
    const glm::vec3 center = 0.5f * (start + end), axis = (end - start) / length;
    const float axial = 0.5f * mass * radius * radius;
    const float transverse = mass * (3 * radius * radius + length * length) / 12;
    body.inertia_pivot +=
        transverse * glm::mat3(1) + (axial - transverse) * glm::outerProduct(axis, axis) + mass * ParallelAxis(center);
    body.center_of_mass += mass * center;
    body.mass += mass;
    for (const auto vertex : {rod.links.x, rod.links.y}) {
      if (!included.insert(vertex).second)
        continue;
      body.vertices.push_back(
          {{inverse_rotation * (glm::vec3(rest.vertices[vertex].position_inverse_mass) - body.anchor), 0},
           {vertex, 0, 0, 0}});
    }
  }
  body.center_of_mass /= body.mass;
  body.inertia_center = body.inertia_pivot - body.mass * ParallelAxis(body.center_of_mass);
  return body;
}

std::optional<TreeSubtreeBody> eco_sys_lab_package::BuildTreeSubtreeBody(const DynamicTreeSkeleton::GpuRestState& rest,
                                                                         uint32_t root_rod) {
  return BuildBody(rest, root_rod);
}
std::optional<std::array<TreeSubtreeBody, 2>> eco_sys_lab_package::BuildTreeSubtreePair(
    const DynamicTreeSkeleton::GpuRestState& rest, uint32_t parent_rod, uint32_t child_rod) {
  const auto whole = BuildBody(rest, parent_rod);
  if (!whole || parent_rod == child_rod || std::none_of(whole->rods.begin(), whole->rods.end(), [&](const auto& rod) {
        return rod.indices.x == child_rod;
      }))
    return {};
  auto parent = BuildBody(rest, parent_rod, child_rod), child = BuildBody(rest, child_rod);
  if (!parent || !child)
    return {};
  return std::array<TreeSubtreeBody, 2>{std::move(*parent), std::move(*child)};
}

bool GpuTreeSubtreePhysics::Initialize(const DynamicTreeSkeleton::GpuRestState& rest, uint32_t root_rod) {
  const auto body = BuildTreeSubtreeBody(rest, root_rod);
  return body && InitializeBodies(rest, {*body});
}
bool GpuTreeSubtreePhysics::InitializeCoupled(const DynamicTreeSkeleton::GpuRestState& rest, uint32_t parent_rod,
                                              uint32_t child_rod) {
  const auto bodies = BuildTreeSubtreePair(rest, parent_rod, child_rod);
  return bodies && InitializeBodies(rest, {bodies->at(0), bodies->at(1)});
}
bool GpuTreeSubtreePhysics::InitializeBodies(const DynamicTreeSkeleton::GpuRestState& rest,
                                             const std::vector<TreeSubtreeBody>& bodies) {
  if (tree_)
    return false;
  layout_ = std::make_shared<DescriptorSetLayout>();
  for (uint32_t i = 0; i < 6; ++i)
    layout_->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout_->Initialize();
  for (uint32_t pass = 0; pass < pipelines_.size(); ++pass) {
    auto& pipeline = pipelines_[pass];
    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute,
        Platform::GetShaderGlobalDefines() + "\n#define SUBTREE_PASS " + std::to_string(pass) + "\n",
        "./EcoSysLabResources/Shaders/Compute/TreeSubtreePhysics.slang");
    pipeline->descriptor_set_layouts = {layout_};
    pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(Parameters)});
    pipeline->Initialize();
    if (!pipeline->Initialized())
      return false;
  }
  tree_ = MakeGpuTreePhysicsState(rest, true);
  std::vector<Properties> properties;
  std::vector<Motion> motions;
  std::vector<TreeSubtreeBody::Rod> rods;
  std::vector<TreeSubtreeBody::Vertex> vertices;
  std::vector<bool> owned(rest.vertices.size(), false);
  for (uint32_t id = 0; id < bodies.size(); ++id) {
    const auto& body = bodies[id];
    Properties p;
    p.center_mass = {body.center_of_mass, body.mass};
    p.inertia0 = {body.inertia_pivot[0], 0};
    p.inertia1 = {body.inertia_pivot[1], 0};
    p.inertia2 = {body.inertia_pivot[2], 0};
    p.counts = {body.rods.size(), body.vertices.size(), rods.size(), vertices.size()};
    properties.push_back(p);
    Motion motion;
    motion.rotation = QuaternionVector(body.rotation);
    motion.anchor = {body.anchor, 0};
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
  properties_ = MakeBuffer(properties);
  motion_ = MakeBuffer(motions);
  rods_ = MakeBuffer(rods);
  vertices_ = MakeBuffer(vertices);
  Joint joint;
  if (bodies.size() == 2) {
    joint.parent_offset = {glm::inverse(bodies[0].rotation) * (bodies[1].anchor - bodies[0].anchor), 0};
    joint.rest_rotation = QuaternionVector(glm::inverse(bodies[0].rotation) * bodies[1].rotation);
  }
  joint_ = MakeBuffer(std::vector<Joint>{joint});
  registry_ = MakeBuffer(std::vector<GpuTreePhysicsAddresses>{GetGpuTreePhysicsAddresses(*tree_)});
  set_ = std::make_shared<DescriptorSet>(layout_);
  const std::array buffers{properties_, motion_, rods_, vertices_, registry_, joint_};
  for (uint32_t i = 0; i < buffers.size(); ++i)
    set_->UpdateBufferDescriptorBinding(i, buffers[i]);
  rod_count_ = static_cast<uint32_t>(rods.size());
  vertex_count_ = static_cast<uint32_t>(vertices.size());
  body_count_ = static_cast<uint32_t>(bodies.size());
  return true;
}

void GpuTreeSubtreePhysics::Step(const Settings& settings, const glm::vec3& anchor, const glm::quat& target_rotation) {
  if (!tree_ || settings.time_step <= 0)
    return;
  Parameters parameters;
  parameters.anchor = {anchor, 0};
  parameters.target_rotation = QuaternionVector(glm::normalize(target_rotation));
  parameters.gravity = {settings.gravity, 0};
  parameters.wind_density = {settings.wind_velocity, std::max(0.0f, settings.air_density)};
  parameters.stiffness_damping = {glm::max(settings.angular_stiffness, glm::vec3(0)), std::max(0.0f, settings.damping)};
  parameters.step = {settings.time_step, std::max(0.0f, settings.wind_drag), 0, 0};
  parameters.base_stiffness = {glm::max(settings.base_angular_stiffness, glm::vec3(0)), 0};
  parameters.control = {glm::clamp(settings.substeps, 1u, 64u), body_count_, rod_count_, vertex_count_};
  Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
    for (uint32_t pass = 0; pass < pipelines_.size(); ++pass) {
      const auto& pipeline = pipelines_[pass];
      pipeline->Bind(command);
      pipeline->BindDescriptorSet(command, 0, set_->GetVkDescriptorSet());
      pipeline->PushConstant(command, 0, parameters);
      pipeline->Dispatch(command, pass == 0 ? 1 : (std::max(rod_count_, vertex_count_) + 63) / 64, 1, 1);
      Platform::ComputeBarrier(command);
    }
    Platform::EverythingBarrier(command);
  });
}
