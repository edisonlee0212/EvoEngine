#include "RayTracer.hpp"

#include "Material.hpp"
#include "Shader.hpp"
#include "MeshRenderer.hpp"
#include "ProjectManager.hpp"

using namespace evo_engine;
void RayTracer::Initialize(
    const std::shared_ptr<Scene>& input_scene,
    const std::function<void(uint32_t mesh_index, const std::shared_ptr<Mesh>& mesh)>& mesh_binding,
    const std::function<void(uint32_t node_index, const Entity& entity)>& node_binding) {
  Clear();
  uint32_t mesh_index = 0;
  std::map<Handle, uint32_t> mesh_index_map;
  const auto entities = input_scene->UnsafeGetPrivateComponentOwnersList<MeshRenderer>();
  if (!entities)
    return;

  for (const auto& entity : *entities) {
    if (!input_scene->IsEntityEnabled(entity))
      continue;
    const auto mesh_renderer = input_scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
    if (!mesh_renderer->IsEnabled())
      continue;
    const auto mesh = mesh_renderer->mesh.Get<Mesh>();
    const auto material = mesh_renderer->material.Get<Material>();
    if (!mesh || !material)
      continue;
    mesh_index_map[mesh->GetHandle()] = mesh_index;

    geometry_instances_.emplace_back();
    auto& mesh_instance = geometry_instances_.back();
    mesh_instance.Initialize(mesh);
    mesh_binding(mesh_index, mesh);
    mesh_index++;
  }
  uint32_t node_index = 0;
  for (const auto& entity : *entities) {
    if (!input_scene->IsEntityEnabled(entity))
      continue;
    const auto mesh_renderer = input_scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
    if (!mesh_renderer->IsEnabled())
      continue;
    const auto mesh = mesh_renderer->mesh.Get<Mesh>();
    const auto material = mesh_renderer->material.Get<Material>();
    if (!mesh || !material)
      continue;
    node_instances_.emplace_back();
    auto& node_instance = node_instances_.back();
    node_instance.Initialize(input_scene, entity, geometry_instances_, mesh_index_map);
    node_binding(node_index, entity);
    node_index++;
  }

  Bvh scene_bvh;
  scene_bvh.element_indices.resize(node_instances_.size());
  std::vector<Bound> element_aabbs(node_instances_.size());
  scene_bvh.aabb = {};
  for (uint32_t i = 0; i < node_instances_.size(); i++) {
    scene_bvh.element_indices[i] = i;
    const auto& node_instance = node_instances_[i];
    auto node_aabb = node_instance.aabb;
    node_aabb.ApplyTransform(node_instance.transformation.value);
    element_aabbs[i] = node_aabb;
    scene_bvh.aabb.min = glm::min(scene_bvh.aabb.min, node_aabb.min);
    scene_bvh.aabb.max = glm::max(scene_bvh.aabb.max, node_aabb.max);
  }
  BinaryDivisionBvh(scene_bvh, 0, element_aabbs);
  aabb_ = scene_bvh.aabb;
  FlattenBvh(scene_bvh, flattened_bvh_node_group_, 0);
}

bool RayAabb(const glm::vec3& r_o, const glm::vec3& r_inv_d, const Bound& aabb) {
  const auto tx1 = (aabb.min.x - r_o.x) * r_inv_d.x;
  const auto tx2 = (aabb.max.x - r_o.x) * r_inv_d.x;

  float t_min = std::min(tx1, tx2);
  float t_max = std::max(tx1, tx2);

  const auto ty1 = (aabb.min.y - r_o.y) * r_inv_d.y;
  const auto ty2 = (aabb.max.y - r_o.y) * r_inv_d.y;

  t_min = std::max(t_min, std::min(ty1, ty2));
  t_max = std::min(t_max, std::max(ty1, ty2));

  const auto tz1 = (aabb.min.z - r_o.z) * r_inv_d.z;
  const auto tz2 = (aabb.max.z - r_o.z) * r_inv_d.z;

  t_min = std::max(t_min, std::min(tz1, tz2));
  t_max = std::min(t_max, std::max(tz1, tz2));

  return t_max >= t_min;
}

void RayTracer::Trace(const RayDescriptor& ray_descriptor,
                         const std::function<void(const HitInfo& hit_info)>& closest_hit_func,
                         const std::function<void()>& miss_func,
                         const std::function<void(const HitInfo& hit_info)>& any_hit_func) const {
  HitInfo closest_hit_info{};
  closest_hit_info.has_hit = true;
  closest_hit_info.distance = FLT_MAX;
  bool has_hit = false;
  const auto flags = static_cast<unsigned>(ray_descriptor.flags);
  const bool enforce_any_hit = flags & static_cast<unsigned>(TraceFlags::EnforceAnyHit);
  const bool cull_back_face = flags & static_cast<unsigned>(TraceFlags::CullBackFace);
  const bool cull_front_face = flags & static_cast<unsigned>(TraceFlags::CullFrontFace);
  float test_distance = ray_descriptor.t_max;
  

  const auto scene_space_ray_direction = glm::normalize(ray_descriptor.direction);
  const auto& scene_space_ray_origin = ray_descriptor.origin;
  const auto scene_space_inv_ray_direction = glm::vec3(
      1.f / scene_space_ray_direction.x, 1.f / scene_space_ray_direction.y, 1.f / scene_space_ray_direction.z);

  uint32_t node_group_index = 0;
  while (node_group_index < flattened_bvh_node_group_.nodes.size()) {
    const auto& node_group = flattened_bvh_node_group_.nodes[node_group_index];
    if (!RayAabb(scene_space_ray_origin, scene_space_inv_ray_direction, node_group.aabb)) {
      node_group_index = node_group.alternate_node_index;
      continue;
    }
    for (uint32_t test_node_index = node_group.begin_next_level_element_index;
         test_node_index < node_group.end_next_level_element_index; ++test_node_index) {
      uint32_t mesh_group_index = 0;
      const auto& node_index = flattened_bvh_node_group_.element_indices[test_node_index];
      const auto& node_instance = node_instances_[node_index];
      const auto& node_global_transform = node_instance.transformation;
      const auto& node_inverse_global_transform = node_instance.inverse_transformation;
      const auto node_space_ray_origin = node_inverse_global_transform.TransformPoint(scene_space_ray_origin);
      const auto node_space_ray_direction =
          glm::normalize(node_inverse_global_transform.TransformVector(scene_space_ray_direction));
      const auto node_space_inv_ray_direction = glm::vec3(
          1.f / node_space_ray_direction.x, 1.f / node_space_ray_direction.y, 1.f / node_space_ray_direction.z);

      while (mesh_group_index < node_instance.flattened_bvh_mesh_group.nodes.size()) {
        const auto& mesh_group = node_instance.flattened_bvh_mesh_group.nodes[mesh_group_index];
        if (!RayAabb(node_space_ray_origin, node_space_inv_ray_direction, mesh_group.aabb)) {
          mesh_group_index = mesh_group.alternate_node_index;
          continue;
        }
        for (uint32_t test_mesh_index = mesh_group.begin_next_level_element_index;
             test_mesh_index < mesh_group.end_next_level_element_index; ++test_mesh_index) {
          uint32_t triangle_group_index = 0;
          const auto& mesh_index = node_instance.flattened_bvh_mesh_group.element_indices[test_mesh_index];
          const auto& mesh_instance = geometry_instances_[mesh_index];
          while (triangle_group_index < mesh_instance.flattened_bvh_triangle_group.nodes.size()) {
            const auto& triangle_group = mesh_instance.flattened_bvh_triangle_group.nodes[triangle_group_index];
            if (!RayAabb(node_space_ray_origin, node_space_inv_ray_direction, triangle_group.aabb)) {
              triangle_group_index = triangle_group.alternate_node_index;
              continue;
            }
            for (uint32_t test_triangle_index = triangle_group.begin_next_level_element_index;
                 test_triangle_index < triangle_group.end_next_level_element_index; ++test_triangle_index) {
              const auto& triangle_index =
                  mesh_instance.flattened_bvh_triangle_group.element_indices[test_triangle_index];
              const auto& triangle = mesh_instance.triangles[triangle_index];
              const auto& p0 = mesh_instance.vertex_positions[triangle.x];
              const auto& p1 = mesh_instance.vertex_positions[triangle.y];
              const auto& p2 = mesh_instance.vertex_positions[triangle.z];
              if (p0 == p1 && p1 == p2)
                continue;
              const auto node_space_triangle_normal = glm::normalize(glm::cross(p1 - p0, p2 - p0));
              const auto normal_test = glm::dot(node_space_ray_direction, node_space_triangle_normal);
              if ((cull_back_face && normal_test > 0.f) || (cull_front_face && normal_test < 0.f) || normal_test == 0.f)
                continue;

              // node_space_hit_distance > 0 instead of node_space_hit_distance >= 0 to avoid self-intersection
              if (const auto node_space_hit_distance = (glm::dot(p0, node_space_triangle_normal) -
                                                        glm::dot(node_space_ray_origin, node_space_triangle_normal)) /
                                                       normal_test;
                  node_space_hit_distance > 0) {
                const auto node_space_hit = node_space_ray_origin + node_space_ray_direction * node_space_hit_distance;
                const auto scene_space_hit = node_global_transform.TransformPoint(node_space_hit);
                if (const auto scene_hit_distance = glm::distance(scene_space_ray_origin, scene_space_hit);
                    scene_hit_distance >= ray_descriptor.t_min && scene_hit_distance <= test_distance) {
                  if (const auto barycentric = Barycentric(node_space_hit, p0, p1, p2);
                      barycentric.x >= 0.f && barycentric.x <= 1.f && barycentric.y >= 0.f && barycentric.y <= 1.f &&
                      barycentric.z >= 0.f && barycentric.z <= 1.f) {
                    HitInfo any_hit_info;
                    any_hit_info.has_hit = true;
                    any_hit_info.hit = scene_space_hit;
                    any_hit_info.normal = node_global_transform.TransformVector(node_space_triangle_normal);
                    any_hit_info.distance = scene_hit_distance;
                    any_hit_info.barycentric = barycentric;
                    any_hit_info.back_face = normal_test > 0.f;
                    any_hit_info.triangle_index = triangle_index;
                    any_hit_info.mesh_index = mesh_index;
                    any_hit_info.node_index = node_index;
                    any_hit_func(any_hit_info);
                    if (!enforce_any_hit) {
                      test_distance = scene_hit_distance;
                    }
                    if (any_hit_info.distance < closest_hit_info.distance) {
                      has_hit = true;
                      closest_hit_info = any_hit_info;
                    }
                  }
                }
              }
            }
            triangle_group_index++;
          }
        }
        mesh_group_index++;
      }
    }
    node_group_index++;
  }
  if (has_hit) {
    closest_hit_func(closest_hit_info);
  } else {
    miss_func();
  }
}

void RayTracer::Clear() noexcept {
  flattened_bvh_node_group_.nodes.clear();
  flattened_bvh_node_group_.element_indices.clear();
  geometry_instances_.clear();
  node_instances_.clear();
}

void RayTracer::Initialize(const std::shared_ptr<Mesh>& input_mesh) {
  Clear();
  geometry_instances_.resize(1);
  node_instances_.resize(1);
  geometry_instances_[0].Initialize(input_mesh);
  aabb_ = node_instances_[0].aabb = geometry_instances_[0].aabb;
  node_instances_[0].flattened_bvh_mesh_group.element_indices = {0};
  node_instances_[0].flattened_bvh_mesh_group.nodes.resize(1);
  node_instances_[0].transformation = GlobalTransform();
  node_instances_[0].inverse_transformation.value = glm::inverse(node_instances_[0].transformation.value);
  auto& node = node_instances_[0].flattened_bvh_mesh_group.nodes[0];
  node.begin_next_level_element_index = 0;
  node.end_next_level_element_index = 1;
  node.alternate_node_index = 1;
  node.aabb = aabb_;

  Bvh scene_bvh;
  scene_bvh.element_indices.resize(node_instances_.size());
  std::vector<Bound> element_aabbs(node_instances_.size());
  scene_bvh.aabb = {};
  scene_bvh.element_indices[0] = 0;
  const auto& node_instance = node_instances_[0];
  auto node_aabb = node_instance.aabb;
  node_aabb.ApplyTransform(node_instance.transformation.value);
  element_aabbs[0] = node_aabb;
  scene_bvh.aabb.min = glm::min(scene_bvh.aabb.min, node_aabb.min);
  scene_bvh.aabb.max = glm::max(scene_bvh.aabb.max, node_aabb.max);

  BinaryDivisionBvh(scene_bvh, 0, element_aabbs);
  aabb_ = scene_bvh.aabb;
  FlattenBvh(scene_bvh, flattened_bvh_node_group_, 0);
}

void RayTracer::BucketBound::AddPoint(const glm::vec3& p) {
  min_v = glm::min(min_v, p);
  max_v = glm::max(max_v, p);
}

void RayTracer::BucketBound::Combine(const BucketBound& other) {
  min_v = glm::min(min_v, other.min_v);
  max_v = glm::max(max_v, other.max_v);
}

float RayTracer::BucketBound::ComputeCost(const int triangle_count) const {
  const auto size = max_v - min_v;
  const float surface_area = 2.0f * (size.x * size.y + size.x * size.z + size.x * size.y);
  return static_cast<float>(triangle_count) * surface_area;
}

RayTracer::BucketSplit RayTracer::SelectSplitFromBuckets(const uint32_t buckets[16],
                                                               const BucketBound buckets_aabb[16],
                                                               const size_t triangle_count) {
  // Pass to compute AABB on the right side of the split
  BucketBound aabb_right_side[15];
  aabb_right_side[14] = buckets_aabb[15];
  for (int i = 13; i >= 0; --i) {
    aabb_right_side[i] = aabb_right_side[i + 1];
    aabb_right_side[i].Combine(buckets_aabb[i + 1]);
  }

  float best_cost = FLT_MAX;
  int best_split_idx = 0;

  BucketBound aabb_left;
  int count_left = 0;
  int count_right = static_cast<int>(triangle_count);
  for (int i = 0; i < 15; ++i) {
    const auto c = static_cast<int>(buckets[i]);
    count_left += c;
    count_right -= c;
    aabb_left.Combine(buckets_aabb[i]);
    BucketBound aabb_right = aabb_right_side[i];
    const float cost_left = aabb_left.ComputeCost(count_left);
    const float cost_right = aabb_right.ComputeCost(count_right);
    if (const float cost = cost_left + cost_right; cost < best_cost) {
      best_cost = cost;
      best_split_idx = i;
    }
  }

  return BucketSplit{static_cast<size_t>(best_split_idx), best_cost};
}

RayTracer::SplitResult RayTracer::FindBestSplit(const Bvh& parent, const std::vector<Bound>& aabbs) {
  SplitResult ret;

  BucketBound centroids_aabb;
  for (const auto& element_index : parent.element_indices) {
    centroids_aabb.AddPoint(aabbs[element_index].Center());
  }

  uint32_t buckets_x[16] = {};
  uint32_t buckets_y[16] = {};
  uint32_t buckets_z[16] = {};
  BucketBound buckets_aabb_x[16];
  BucketBound buckets_aabb_y[16];
  BucketBound buckets_aabb_z[16];

  for (const auto& element_index : parent.element_indices) {
    const auto& element_aabb = aabbs[element_index];
    const auto ijk =
        (element_aabb.Center() - centroids_aabb.min_v) / (centroids_aabb.max_v - centroids_aabb.min_v) * 16.f;
    const size_t i =
        std::min(std::isnan(ijk.x) ? static_cast<size_t>(0) : static_cast<size_t>(ijk.x), static_cast<size_t>(15));
    const size_t j =
        std::min(std::isnan(ijk.y) ? static_cast<size_t>(0) : static_cast<size_t>(ijk.y), static_cast<size_t>(15));
    const size_t k =
        std::min(std::isnan(ijk.z) ? static_cast<size_t>(0) : static_cast<size_t>(ijk.z), static_cast<size_t>(15));
    ++buckets_x[i];
    ++buckets_y[j];
    ++buckets_z[k];
    buckets_aabb_x[i].AddPoint(element_aabb.min);
    buckets_aabb_x[i].AddPoint(element_aabb.max);
    buckets_aabb_y[j].AddPoint(element_aabb.min);
    buckets_aabb_y[j].AddPoint(element_aabb.max);
    buckets_aabb_z[k].AddPoint(element_aabb.min);
    buckets_aabb_z[k].AddPoint(element_aabb.max);
  }

  const uint32_t node_count = static_cast<uint32_t>(parent.element_indices.size());
  const auto split_x = SelectSplitFromBuckets(buckets_x, buckets_aabb_x, node_count);
  const auto split_y = SelectSplitFromBuckets(buckets_y, buckets_aabb_y, node_count);
  const auto split_z = SelectSplitFromBuckets(buckets_z, buckets_aabb_z, node_count);

  if (split_x.cost <= split_y.cost && split_x.cost <= split_z.cost) {
    ret.axis = Axis::X;
    ret.split = centroids_aabb.min_v.x +
                (centroids_aabb.max_v.x - centroids_aabb.min_v.x) / 16.0f * static_cast<float>(split_x.split_idx + 1);
  } else if (split_y.cost <= split_x.cost && split_y.cost <= split_z.cost) {
    ret.axis = Axis::Y;
    ret.split = centroids_aabb.min_v.y +
                (centroids_aabb.max_v.y - centroids_aabb.min_v.y) / 16.0f * static_cast<float>(split_y.split_idx + 1);
  } else {
    ret.axis = Axis::Z;
    ret.split = centroids_aabb.min_v.z +
                (centroids_aabb.max_v.z - centroids_aabb.min_v.z) / 16.0f * static_cast<float>(split_z.split_idx + 1);
  }
  return ret;
}

void RayTracer::BinaryDivisionBvh(Bvh& parent, const uint32_t current_tree_depth, const std::vector<Bound>& aabbs) {
  if (parent.element_indices.size() == 1) {
    parent.subtree_element_size = static_cast<uint32_t>(1);
    return;
  }
  parent.children.resize(2);
  parent.children[0].aabb = {};
  parent.children[1].aabb = {};
  const SplitResult split = FindBestSplit(parent, aabbs);
  for (const auto& element_index : parent.element_indices) {
    const auto& current_aabb = aabbs[element_index];
    const auto current_aabb_center = current_aabb.Center();
    if ((split.axis == Axis::X && current_aabb_center.x <= split.split) ||
        (split.axis == Axis::Y && current_aabb_center.y <= split.split) ||
        (split.axis == Axis::Z && current_aabb_center.z <= split.split)) {
      parent.children[0].element_indices.emplace_back(element_index);
      parent.children[0].aabb.min = glm::min(parent.children[0].aabb.min, current_aabb.min);
      parent.children[0].aabb.max = glm::max(parent.children[0].aabb.max, current_aabb.max);
    } else {
      parent.children[1].element_indices.emplace_back(element_index);
      parent.children[1].aabb.min = glm::min(parent.children[1].aabb.min, current_aabb.min);
      parent.children[1].aabb.max = glm::max(parent.children[1].aabb.max, current_aabb.max);
    }
  }
  if (parent.children[0].element_indices.empty() || parent.children[1].element_indices.empty()) {
    parent.children.clear();
    parent.subtree_element_size = static_cast<uint32_t>(parent.element_indices.size());
    return;
  }
  parent.element_indices.clear();
  BinaryDivisionBvh(parent.children[0], current_tree_depth + 1, aabbs);
  BinaryDivisionBvh(parent.children[1], current_tree_depth + 1, aabbs);
  parent.subtree_element_size = parent.children[0].subtree_element_size + parent.children[1].subtree_element_size;
}

void RayTracer::FlattenBvh(const Bvh& current_bvh, FlattenedBvh& flattened_bvh, const uint32_t level) {
  if (current_bvh.children.empty() && current_bvh.element_indices.empty()) {
    // Children scene without triangles? Skip it
    return;
  }
  if (!current_bvh.children.empty()) {
    // If one of the children does not contain any triangles
    // we can skip this scene completely as it is an extra AABB test
    // for nothing
    if (current_bvh.children[0].subtree_element_size > 0 && current_bvh.children[1].subtree_element_size == 0) {
      FlattenBvh(current_bvh.children[0], flattened_bvh, level + 1);
      return;
    }
    if (current_bvh.children[1].subtree_element_size > 0 && current_bvh.children[0].subtree_element_size == 0) {
      FlattenBvh(current_bvh.children[1], flattened_bvh, level + 1);
      return;
    }
  }
  auto& node = flattened_bvh.nodes.emplace_back();
  node.aabb = current_bvh.aabb;
  node.begin_next_level_element_index = static_cast<uint32_t>(flattened_bvh.element_indices.size());
  flattened_bvh.element_indices.insert(flattened_bvh.element_indices.end(), current_bvh.element_indices.begin(),
                                       current_bvh.element_indices.end());
  node.end_next_level_element_index = static_cast<uint32_t>(flattened_bvh.element_indices.size());
  const size_t current_scene_index = flattened_bvh.nodes.size() - 1;
  if (!current_bvh.children.empty()) {
    FlattenBvh(current_bvh.children[0], flattened_bvh, level + 1);
    FlattenBvh(current_bvh.children[1], flattened_bvh, level + 1);
  }
  flattened_bvh.nodes[current_scene_index].alternate_node_index = static_cast<uint32_t>(flattened_bvh.nodes.size());
}

glm::vec3 RayTracer::Barycentric(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
  const auto v0 = b - a;
  const auto v1 = c - a;
  const auto v2 = p - a;
  const auto d00 = dot(v0, v0);
  const auto d01 = dot(v0, v1);
  const auto d11 = dot(v1, v1);
  const auto d20 = dot(v2, v0);
  const auto d21 = dot(v2, v1);
  const auto denominator = d00 * d11 - d01 * d01;
  const auto y = (d11 * d20 - d01 * d21) / denominator;
  const auto z = (d00 * d21 - d01 * d20) / denominator;
  return {1.0f - y - z, y, z};
}



float UintBitsToFloat(const uint32_t src) {
  union FloatAndUInt {
    float f;
    uint32_t i;
  };

  FloatAndUInt f_and_ui;
  f_and_ui.i = src;
  return f_and_ui.f;
}

uint32_t FloatBitsToUint(const float src) {
  union FloatAndUInt {
    float f;
    uint32_t i;
  };

  FloatAndUInt f_and_ui;
  f_and_ui.f = src;

  return f_and_ui.i;
}

constexpr uint32_t DivUp(uint32_t a, uint32_t b) {
  return (a + b - 1) / b;
}

void RayTracer::AggregatedScene::InitializeBuffers() {
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

  scene_level_bvh_nodes_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  node_indices_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  node_info_list_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  node_level_bvh_nodes_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  mesh_indices_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  mesh_info_list_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  mesh_level_bvh_nodes_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  triangle_indices_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  local_triangle_indices_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  scene_triangles_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  scene_vertex_positions_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  scene_info_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  #pragma region Upload functions
  auto upload_bvh_nodes = [&](const std::shared_ptr<Buffer> &buffer,
                              const std::vector<BvhNode>& bvh_nodes) {
    struct GpuBvhNode {
      glm::vec4 aabb_min{};
      glm::vec4 aabb_max{};
      glm::vec4 indices{};
      GpuBvhNode() = default;

      explicit GpuBvhNode(const BvhNode& bvh_node) {
        aabb_min = glm::vec4(bvh_node.aabb.min.x, bvh_node.aabb.min.y, bvh_node.aabb.min.z, 0.f);
        aabb_max = glm::vec4(bvh_node.aabb.max.x, bvh_node.aabb.max.y, bvh_node.aabb.max.z, 0.f);
        indices[0] = UintBitsToFloat(bvh_node.alternate_node_index);
        indices[1] = UintBitsToFloat(bvh_node.begin_next_level_element_index);
        indices[2] = UintBitsToFloat(bvh_node.end_next_level_element_index);
      }
    };
    std::vector<GpuBvhNode> gpu_bvh_nodes;
    gpu_bvh_nodes.resize(bvh_nodes.size());
    Jobs::RunParallelFor(bvh_nodes.size(), [&](const auto i) {
        gpu_bvh_nodes[i] = GpuBvhNode(bvh_nodes[i]);
    });
    buffer->UploadVector(gpu_bvh_nodes);
  };

  auto upload_node_info_list = [&](const std::shared_ptr<Buffer> &buffer,
                                   const std::vector<GlobalTransform>& transforms,
                                   const std::vector<GlobalTransform>& inverse_transforms,
                                   const std::vector<uint32_t>& node_level_bvh_node_offsets,
                                   const std::vector<uint32_t>& node_level_bvh_node_sizes,
                                   const std::vector<uint32_t>& mesh_indices_offsets,
                                   const std::vector<uint32_t>& mesh_indices_sizes) {
    struct GpuNodeInfo {
      glm::mat4 transform{};
      glm::mat4 inverse_transform{};
      glm::vec4 offsets{};
      GpuNodeInfo() = default;

      explicit GpuNodeInfo(const glm::mat4& src_transform, const glm::mat4& src_inverse_transform,
                           const uint32_t node_level_bvh_node_offset, const uint32_t node_level_bvh_node_size,
                           const uint32_t mesh_indices_offset, const uint32_t mesh_indices_size) {
        transform = src_transform;
        inverse_transform = src_inverse_transform;
        offsets[0] = UintBitsToFloat(node_level_bvh_node_offset);
        offsets[1] = UintBitsToFloat(node_level_bvh_node_size);
        offsets[2] = UintBitsToFloat(mesh_indices_offset);
        offsets[3] = UintBitsToFloat(mesh_indices_size);
      }
    };

    auto node_infos = std::vector(transforms.size(), GpuNodeInfo());
    Jobs::RunParallelFor(transforms.size(), [&](const auto i) {
      node_infos[i] = GpuNodeInfo(transforms[i].value, inverse_transforms[i].value, node_level_bvh_node_offsets[i],
                                    node_level_bvh_node_sizes[i], mesh_indices_offsets[i], mesh_indices_sizes[i]);
    });
    buffer->UploadVector(node_infos);
  };

  auto upload_mesh_info_list =
      [&](const std::shared_ptr<Buffer> &buffer, const std::vector<uint32_t>& mesh_level_offsets,
          const std::vector<uint32_t>& mesh_level_sizes, const std::vector<uint32_t>& triangle_indices_offsets,
          const std::vector<uint32_t>& triangle_indices_sizes) {
        struct GpuMeshInfo {
          glm::vec4 offsets{};
          GpuMeshInfo() = default;

          explicit GpuMeshInfo(const uint32_t mesh_level_bvh_node_offset, const uint32_t mesh_level_bvh_node_size,
                               const uint32_t triangle_indices_offset, const uint32_t triangle_indices_size) {
            offsets[0] = UintBitsToFloat(mesh_level_bvh_node_offset);
            offsets[1] = UintBitsToFloat(mesh_level_bvh_node_size);
            offsets[2] = UintBitsToFloat(triangle_indices_offset);
            offsets[3] = UintBitsToFloat(triangle_indices_size);
          }
        };

        auto mesh_infos = std::vector(mesh_level_offsets.size(), GpuMeshInfo());
        Jobs::RunParallelFor(mesh_level_offsets.size(),
                          [&](const auto i) {
                              mesh_infos[i] = GpuMeshInfo(mesh_level_offsets[i], mesh_level_sizes[i],
                                                          triangle_indices_offsets[i], triangle_indices_sizes[i]);
                            
                          });
        buffer->UploadVector(mesh_infos);
      };

  auto upload_indices = [&](const std::shared_ptr<Buffer> &buffer, const std::vector<uint32_t>& src) {
    const auto src_length = src.size();
    const auto dst_length = DivUp(static_cast<uint32_t>(src_length), 4);
    std::vector<glm::vec4> gpu_indices;
    gpu_indices.resize(dst_length);
    Jobs::RunParallelFor(dst_length, [&](const auto i) {
        if (i * 4 < src_length) {
          gpu_indices[i].x = UintBitsToFloat(src[i * 4]);
        }
        if (i * 4 + 1 < src_length) {
          gpu_indices[i].y = UintBitsToFloat(src[i * 4 + 1]);
        }
        if (i * 4 + 2 < src_length) {
          gpu_indices[i].z = UintBitsToFloat(src[i * 4 + 2]);
        }
        if (i * 4 + 3 < src_length) {
          gpu_indices[i].w = UintBitsToFloat(src[i * 4 + 3]);
        }
      
    });
    buffer->UploadVector(gpu_indices);
  };

  auto upload_triangles = [&](const std::shared_ptr<Buffer> &buffer, const std::vector<glm::uvec3>& src) {
    std::vector<glm::vec4> gpu_triangles;
    gpu_triangles.resize(src.size());
    Jobs::RunParallelFor(src.size(), [&](const auto i) {
        gpu_triangles[i].x = UintBitsToFloat(src[i].x);
        gpu_triangles[i].y = UintBitsToFloat(src[i].y);
        gpu_triangles[i].z = UintBitsToFloat(src[i].z);
        gpu_triangles[i].w = 0.f;
    });
    buffer->UploadVector(gpu_triangles);
  };

  auto upload_vec3 = [&](const std::shared_ptr<Buffer> &buffer, const std::vector<glm::vec3>& src) {
    std::vector<glm::vec4> gpu_vertices;
    gpu_vertices.resize(src.size());
    Jobs::RunParallelFor(src.size(), [&](const auto i) {
        gpu_vertices[i] = glm::vec4(src[i].x, src[i].y, src[i].z, 0.f);
      
    });
    buffer->UploadVector(gpu_vertices);
  };

  
#pragma endregion
  upload_bvh_nodes(scene_level_bvh_nodes_buffer, scene_level_bvh_nodes_);
  upload_indices(node_indices_buffer, node_indices_);
  
  upload_node_info_list(node_info_list_buffer, node_transforms_,
                        node_inverse_transforms_, node_level_bvh_node_offsets_,
                        node_level_bvh_node_sizes_, mesh_indices_offsets_,
                        mesh_indices_sizes_);
  upload_bvh_nodes(node_level_bvh_nodes_buffer, node_level_bvh_nodes_);
  upload_indices(mesh_indices_buffer, mesh_indices_);
  upload_mesh_info_list(mesh_info_list_buffer, mesh_level_bvh_node_offsets_, mesh_level_bvh_node_sizes_,
                        triangle_indices_offsets_,
                        triangle_indices_sizes_);
  upload_bvh_nodes(mesh_level_bvh_nodes_buffer, mesh_level_bvh_nodes_);
  upload_indices(triangle_indices_buffer, triangle_indices_);
  upload_indices(local_triangle_indices_buffer, local_triangle_indices_);
  upload_triangles(scene_triangles_buffer, triangles_);
  upload_vec3(scene_vertex_positions_buffer, vertex_positions_);

  scene_info_buffer->Upload(glm::vec4(UintBitsToFloat(static_cast<uint32_t>(scene_level_bvh_nodes_.size())), 0, 0, 0));
}

void RayTracer::AggregatedScene::Trace(const RayDescriptor& ray_descriptor,
                                       const std::function<void(const HitInfo& hit_info)>& closest_hit_func,
                                       const std::function<void()>& miss_func,
                                       const std::function<void(const HitInfo& hit_info)>& any_hit_func) const {
  HitInfo closest_hit_info{};
  closest_hit_info.has_hit = true;
  closest_hit_info.distance = FLT_MAX;
  bool has_hit = false;
  const auto flags = static_cast<unsigned>(ray_descriptor.flags);
  const bool enforce_any_hit = flags & static_cast<unsigned>(TraceFlags::EnforceAnyHit);
  const bool cull_back_face = flags & static_cast<unsigned>(TraceFlags::CullBackFace);
  const bool cull_front_face = flags & static_cast<unsigned>(TraceFlags::CullFrontFace);
  float test_distance = ray_descriptor.t_max;
  auto scene_space_ray_direction = glm::normalize(ray_descriptor.direction);
  const auto& scene_space_ray_origin = ray_descriptor.origin;
  const auto scene_space_inv_ray_direction = glm::vec3(
      1.f / scene_space_ray_direction.x, 1.f / scene_space_ray_direction.y, 1.f / scene_space_ray_direction.z);

  uint32_t node_group_index = 0;
  std::unordered_set<uint32_t> node_group_index_test;
  while (node_group_index < scene_level_bvh_nodes_.size()) {
    if (node_group_index_test.find(node_group_index) == node_group_index_test.end()) {
      node_group_index_test.emplace(node_group_index);
    } else {
      throw std::runtime_error("Duplicate node!");
    }
    const auto& node_group = scene_level_bvh_nodes_[node_group_index];
    if (!RayAabb(scene_space_ray_origin, scene_space_inv_ray_direction, node_group.aabb)) {
      node_group_index = node_group.alternate_node_index;
      continue;
    }
    for (uint32_t test_node_element_index = node_group.begin_next_level_element_index;
         test_node_element_index < node_group.end_next_level_element_index; ++test_node_element_index) {
      uint32_t mesh_group_index = 0;
      const auto node_index = node_indices_[test_node_element_index];
      const auto& node_global_transform = node_transforms_[node_index];
      const auto& node_inverse_global_transform = node_inverse_transforms_[node_index];
      const auto node_space_ray_origin = node_inverse_global_transform.TransformPoint(scene_space_ray_origin);
      auto node_space_ray_direction =
          glm::normalize(node_inverse_global_transform.TransformVector(scene_space_ray_direction));
      const auto node_space_inv_ray_direction = glm::vec3(
          1.f / node_space_ray_direction.x, 1.f / node_space_ray_direction.y, 1.f / node_space_ray_direction.z);
      const auto node_level_bvh_node_offset = node_level_bvh_node_offsets_[node_index];
      const auto node_level_bvh_node_size = node_level_bvh_node_sizes_[node_index];
      const auto mesh_indices_offset = mesh_indices_offsets_[node_index];
      std::unordered_set<uint32_t> mesh_group_index_test;
      while (mesh_group_index < node_level_bvh_node_size) {
        if (mesh_group_index_test.find(mesh_group_index) == mesh_group_index_test.end()) {
          mesh_group_index_test.emplace(mesh_group_index);
        } else {
          throw std::runtime_error("Duplicate mesh!");
        }
        const auto& mesh_group = node_level_bvh_nodes_[mesh_group_index + node_level_bvh_node_offset];
        if (!RayAabb(node_space_ray_origin, node_space_inv_ray_direction, mesh_group.aabb)) {
          mesh_group_index = mesh_group.alternate_node_index;
          continue;
        }
        for (uint32_t test_mesh_element_index = mesh_group.begin_next_level_element_index + mesh_indices_offset;
             test_mesh_element_index < mesh_group.end_next_level_element_index + mesh_indices_offset;
             ++test_mesh_element_index) {
          uint32_t triangle_group_index = 0;
          const auto mesh_index = mesh_indices_[test_mesh_element_index];
          const auto mesh_level_bvh_node_offset = mesh_level_bvh_node_offsets_[mesh_index];
          const auto mesh_level_bvh_node_size = mesh_level_bvh_node_sizes_[mesh_index];
          const auto triangle_indices_offset = triangle_indices_offsets_[mesh_index];
          std::unordered_set<uint32_t> triangle_group_index_test;
          while (triangle_group_index < mesh_level_bvh_node_size) {
            if (triangle_group_index_test.find(triangle_group_index) == triangle_group_index_test.end()) {
              triangle_group_index_test.emplace(triangle_group_index);
            } else {
              throw std::runtime_error("Duplicate triangle!");
            }
            const auto& triangle_group = mesh_level_bvh_nodes_[triangle_group_index + mesh_level_bvh_node_offset];
            if (!RayAabb(node_space_ray_origin, node_space_inv_ray_direction, triangle_group.aabb)) {
              triangle_group_index = triangle_group.alternate_node_index;
              continue;
            }
            for (uint32_t test_triangle_index = triangle_group.begin_next_level_element_index + triangle_indices_offset;
                 test_triangle_index < triangle_group.end_next_level_element_index + triangle_indices_offset;
                 ++test_triangle_index) {
              const auto triangle_index = triangle_indices_[test_triangle_index];
              const auto& triangle = triangles_[triangle_index];
              const auto& p0 = vertex_positions_[triangle.x];
              const auto& p1 = vertex_positions_[triangle.y];
              const auto& p2 = vertex_positions_[triangle.z];
              if (p0 == p1 && p1 == p2)
                continue;
              auto node_space_triangle_normal = glm::normalize(glm::cross(p1 - p0, p2 - p0));
              const auto normal_test = glm::dot(node_space_ray_direction, node_space_triangle_normal);
              if ((cull_back_face && normal_test > 0.f) || (cull_front_face && normal_test < 0.f) || normal_test == 0.f)
                continue;

              // node_space_hit_distance > 0 instead of node_space_hit_distance >= 0 to avoid self-intersection
              if (const auto node_space_hit_distance =
                      glm::dot(p0, node_space_triangle_normal) -
                      glm::dot(node_space_ray_origin, node_space_triangle_normal) / normal_test;
                  node_space_hit_distance > 0) {
                const auto node_space_hit = node_space_ray_origin + node_space_ray_direction * node_space_hit_distance;
                const auto scene_space_hit = node_global_transform.TransformPoint(node_space_hit);
                if (const auto scene_hit_distance = glm::distance(scene_space_ray_origin, scene_space_hit);
                    scene_hit_distance >= ray_descriptor.t_min && scene_hit_distance <= test_distance) {
                  if (const auto barycentric = Barycentric(node_space_hit, p0, p1, p2);
                      barycentric.x >= 0.f && barycentric.x <= 1.f && barycentric.y >= 0.f && barycentric.y <= 1.f &&
                      barycentric.z >= 0.f && barycentric.z <= 1.f) {
                    HitInfo any_hit_info{};
                    any_hit_info.has_hit = true;
                    any_hit_info.hit = scene_space_hit;
                    any_hit_info.normal = node_global_transform.TransformVector(node_space_triangle_normal);
                    any_hit_info.distance = scene_hit_distance;
                    any_hit_info.barycentric = barycentric;
                    any_hit_info.back_face = normal_test > 0.f;
                    any_hit_info.triangle_index = local_triangle_indices_[test_triangle_index];
                    any_hit_info.mesh_index = mesh_index;
                    any_hit_info.node_index = node_index;
                    any_hit_func(any_hit_info);
                    if (!enforce_any_hit) {
                      test_distance = scene_hit_distance;
                    }
                    if (any_hit_info.distance < closest_hit_info.distance) {
                      has_hit = true;
                      closest_hit_info = any_hit_info;
                    }
                  }
                }
              }
            }
            triangle_group_index++;
          }
        }
        mesh_group_index++;
      }
    }
    node_group_index++;
  }
  if (has_hit) {
    closest_hit_func(closest_hit_info);
  } else {
    miss_func();
  }
}

std::shared_ptr<DescriptorSetLayout> ray_tracer_descriptor_set_layout;
std::shared_ptr<Shader> trace_shader{};
std::shared_ptr<ComputePipeline> trace_pipeline{};

void RayTracer::AggregatedScene::TraceGpu(const std::vector<RayDescriptor>& rays, std::vector<HitInfo>& hit_infos,
                                          const TraceFlags flags) {
  if (!ray_tracer_descriptor_set_layout) {
    ray_tracer_descriptor_set_layout = std::make_shared<DescriptorSetLayout>();
    ray_tracer_descriptor_set_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ray_tracer_descriptor_set_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ray_tracer_descriptor_set_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ray_tracer_descriptor_set_layout->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ray_tracer_descriptor_set_layout->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ray_tracer_descriptor_set_layout->PushDescriptorBinding(5, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ray_tracer_descriptor_set_layout->PushDescriptorBinding(6, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ray_tracer_descriptor_set_layout->PushDescriptorBinding(7, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ray_tracer_descriptor_set_layout->PushDescriptorBinding(8, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ray_tracer_descriptor_set_layout->PushDescriptorBinding(9, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ray_tracer_descriptor_set_layout->PushDescriptorBinding(10, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ray_tracer_descriptor_set_layout->PushDescriptorBinding(11, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);

    ray_tracer_descriptor_set_layout->PushDescriptorBinding(12, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ray_tracer_descriptor_set_layout->PushDescriptorBinding(13, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);

    ray_tracer_descriptor_set_layout->Initialize();
  }

  if (!trace_shader) {
    trace_shader = ProjectManager::CreateTemporaryAsset<Shader>();
    trace_shader->Set(ShaderType::Compute,
                      std::filesystem::path("./DefaultResources") / "Shaders/Compute/Trace.comp");
  }

  if (!trace_pipeline) {
    trace_pipeline = std::make_shared<ComputePipeline>();
    trace_pipeline->descriptor_set_layouts.emplace_back(ray_tracer_descriptor_set_layout);
    auto& push_constant_range = trace_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(glm::vec4);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    trace_pipeline->compute_shader = trace_shader;
    trace_pipeline->Initialize();
  }
  InitializeBuffers();
  
  auto upload_rays = [&](const std::shared_ptr<Buffer>& buffer, const std::vector<RayDescriptor>& ray_descriptors) {
    std::vector<glm::vec4> gpu_vertices(ray_descriptors.size() * 2);
    Jobs::RunParallelFor(ray_descriptors.size(), [&](const auto i) {
      gpu_vertices[i * 2] = glm::vec4(ray_descriptors[i].origin.x, ray_descriptors[i].origin.y,
                                      ray_descriptors[i].origin.z, ray_descriptors[i].t_min);
      gpu_vertices[i * 2 + 1] = glm::vec4(ray_descriptors[i].direction.x, ray_descriptors[i].direction.y,
                                          ray_descriptors[i].direction.z, ray_descriptors[i].t_max);
    });
    buffer->UploadVector(gpu_vertices);
  };

  
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto rays_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  
  const auto ray_casting_results_buffer = std::make_shared<Buffer>(
      buffer_create_info, buffer_vma_allocation_create_info);

  struct GpuRayCastingResult {
    glm::vec4 hit{};
    glm::vec4 barycentric_back_face{};
    glm::vec4 normal_distance{};
    glm::vec4 node_mesh_triangle_indices{};
    GpuRayCastingResult() = default;
  };

  ray_casting_results_buffer->Resize(sizeof(GpuRayCastingResult) * rays.size());
  upload_rays(rays_buffer, rays);

  const auto ray_tracer_descriptor_set = std::make_shared<DescriptorSet>(ray_tracer_descriptor_set_layout);

  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(0, scene_level_bvh_nodes_buffer);

  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(1, node_indices_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(2, node_info_list_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(3, node_level_bvh_nodes_buffer);

  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(4, mesh_indices_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(5, mesh_info_list_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(6, mesh_level_bvh_nodes_buffer);

  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(7, triangle_indices_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(8, local_triangle_indices_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(9, scene_triangles_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(10, scene_vertex_positions_buffer);

  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(11, scene_info_buffer);

  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(12, rays_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(13, ray_casting_results_buffer);

  const glm::vec4 ray_cast_config =
      glm::vec4(static_cast<unsigned>(flags) | static_cast<unsigned>(TraceFlags::CullBackFace) ? 0.f : 1.f,
                static_cast<unsigned>(flags) | static_cast<unsigned>(TraceFlags::CullFrontFace) ? 0.f : 1.f,
                UintBitsToFloat(static_cast<uint32_t>(rays.size())), 0.f);
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    trace_pipeline->Bind(vk_command_buffer);
    trace_pipeline->BindDescriptorSet(vk_command_buffer, 0, ray_tracer_descriptor_set->GetVkDescriptorSet());
    trace_pipeline->PushConstant(vk_command_buffer, 0, ray_cast_config);
    vkCmdDispatch(vk_command_buffer, DivUp(static_cast<uint32_t>(rays.size()), 256), 1, 1);
  });

  std::vector<GpuRayCastingResult> gpu_ray_casting_results(rays.size());
  ray_casting_results_buffer->DownloadVector(gpu_ray_casting_results, rays.size());

  hit_infos.resize(rays.size());
  Jobs::RunParallelFor(rays.size(), [&](const auto i) {
    auto& hit_info = hit_infos[i];
    auto& gpu_ray_casting_result = gpu_ray_casting_results[i];
    if (gpu_ray_casting_result.hit.w == 0.f) {
      hit_info = {};
      return;
    }
    hit_info.has_hit = true;
    hit_info.hit = glm::vec3(gpu_ray_casting_result.hit);
    hit_info.barycentric = glm::vec3(gpu_ray_casting_result.barycentric_back_face);
    hit_info.back_face = gpu_ray_casting_result.barycentric_back_face.w != 0.f;
    hit_info.normal = glm::vec3(gpu_ray_casting_result.normal_distance);
    hit_info.distance = gpu_ray_casting_result.normal_distance.w;
    hit_info.node_index = FloatBitsToUint(gpu_ray_casting_result.node_mesh_triangle_indices.x);
    hit_info.mesh_index = FloatBitsToUint(gpu_ray_casting_result.node_mesh_triangle_indices.y);
    hit_info.triangle_index = FloatBitsToUint(gpu_ray_casting_result.node_mesh_triangle_indices.z);
  });
}

RayTracer::AggregatedScene RayTracer::Aggregate() const {
  AggregatedScene aggregated_scene;
  aggregated_scene.scene_level_bvh_nodes_ = flattened_bvh_node_group_.nodes;
  aggregated_scene.node_indices_ = flattened_bvh_node_group_.element_indices;

  aggregated_scene.node_transforms_.resize(node_instances_.size());
  aggregated_scene.node_inverse_transforms_.resize(node_instances_.size());
  aggregated_scene.node_level_bvh_node_offsets_.resize(node_instances_.size());
  aggregated_scene.node_level_bvh_node_sizes_.resize(node_instances_.size());
  aggregated_scene.mesh_indices_offsets_.resize(node_instances_.size());
  aggregated_scene.mesh_indices_sizes_.resize(node_instances_.size());
  for (uint32_t node_index = 0; node_index < node_instances_.size(); node_index++) {
    const auto& node_instance = node_instances_[node_index];
    aggregated_scene.node_transforms_[node_index] = node_instance.transformation;
    aggregated_scene.node_inverse_transforms_[node_index] = node_instance.inverse_transformation;
    aggregated_scene.node_level_bvh_node_offsets_[node_index] =
        static_cast<uint32_t>(aggregated_scene.node_level_bvh_nodes_.size());
    aggregated_scene.node_level_bvh_node_sizes_[node_index] =
        static_cast<uint32_t>(node_instance.flattened_bvh_mesh_group.nodes.size());
    aggregated_scene.mesh_indices_offsets_[node_index] = static_cast<uint32_t>(aggregated_scene.mesh_indices_.size());
    aggregated_scene.mesh_indices_sizes_[node_index] =
        static_cast<uint32_t>(node_instance.flattened_bvh_mesh_group.element_indices.size());
    aggregated_scene.node_level_bvh_nodes_.insert(aggregated_scene.node_level_bvh_nodes_.end(),
                                                 node_instance.flattened_bvh_mesh_group.nodes.begin(),
                                                 node_instance.flattened_bvh_mesh_group.nodes.end());
    aggregated_scene.mesh_indices_.insert(aggregated_scene.mesh_indices_.end(),
                                         node_instance.flattened_bvh_mesh_group.element_indices.begin(),
                                         node_instance.flattened_bvh_mesh_group.element_indices.end());
  }
  aggregated_scene.mesh_level_bvh_node_offsets_.resize(geometry_instances_.size());
  aggregated_scene.mesh_level_bvh_node_sizes_.resize(geometry_instances_.size());
  aggregated_scene.triangle_indices_offsets_.resize(geometry_instances_.size());
  aggregated_scene.triangle_indices_sizes_.resize(geometry_instances_.size());
  for (uint32_t mesh_index = 0; mesh_index < geometry_instances_.size(); mesh_index++) {
    const auto& mesh_instance = geometry_instances_[mesh_index];
    aggregated_scene.mesh_level_bvh_node_offsets_[mesh_index] =
        static_cast<uint32_t>(aggregated_scene.mesh_level_bvh_nodes_.size());
    aggregated_scene.mesh_level_bvh_node_sizes_[mesh_index] =
        static_cast<uint32_t>(mesh_instance.flattened_bvh_triangle_group.nodes.size());
    aggregated_scene.triangle_indices_offsets_[mesh_index] =
        static_cast<uint32_t>(aggregated_scene.triangle_indices_.size());
    aggregated_scene.triangle_indices_sizes_[mesh_index] =
        static_cast<uint32_t>(mesh_instance.flattened_bvh_triangle_group.element_indices.size());
    aggregated_scene.mesh_level_bvh_nodes_.insert(aggregated_scene.mesh_level_bvh_nodes_.end(),
                                                 mesh_instance.flattened_bvh_triangle_group.nodes.begin(),
                                                 mesh_instance.flattened_bvh_triangle_group.nodes.end());
    const auto& vertex_positions = mesh_instance.vertex_positions;
    const auto& triangles = mesh_instance.triangles;
    const auto vertex_offset = static_cast<uint32_t>(aggregated_scene.vertex_positions_.size());
    const auto triangle_offset = static_cast<uint32_t>(aggregated_scene.triangles_.size());
    aggregated_scene.vertex_positions_.insert(aggregated_scene.vertex_positions_.end(), vertex_positions.begin(),
                                             vertex_positions.end());
    aggregated_scene.triangles_.resize(triangle_offset + triangles.size());
    for (uint32_t i = 0; i < triangles.size(); i++) {
      aggregated_scene.triangles_[triangle_offset + i] = triangles[i] + vertex_offset;
    }
    const auto& local_triangle_indices = mesh_instance.flattened_bvh_triangle_group.element_indices;
    const uint32_t global_triangle_indices_offset = static_cast<uint32_t>(aggregated_scene.triangle_indices_.size());
    aggregated_scene.triangle_indices_.resize(global_triangle_indices_offset + local_triangle_indices.size());
    for (uint32_t i = 0; i < local_triangle_indices.size(); i++) {
      aggregated_scene.triangle_indices_[global_triangle_indices_offset + i] =
          local_triangle_indices[i] + triangle_offset;
    }

    aggregated_scene.local_triangle_indices_.insert(aggregated_scene.local_triangle_indices_.end(),
                                                   mesh_instance.flattened_bvh_triangle_group.element_indices.begin(),
                                                   mesh_instance.flattened_bvh_triangle_group.element_indices.end());
  }
  return aggregated_scene;
}

void RayTracer::GeometryInstance::Initialize(const std::shared_ptr<Mesh>& input_mesh) {
  Clear();
  const auto& input_vertices = input_mesh->UnsafeGetVertices();
  triangles = input_mesh->UnsafeGetTriangles();
  Bvh mesh_bvh{};
  const auto vertices_size = static_cast<uint32_t>(input_vertices.size());
  const auto triangles_size = static_cast<uint32_t>(triangles.size());
  mesh_bvh.aabb = input_mesh->GetBound();
  vertex_positions.resize(vertices_size);
  Jobs::RunParallelFor(vertices_size, [&](const size_t index) {
    vertex_positions[index] = input_vertices[index].position;
  });
  mesh_bvh.element_indices.resize(triangles_size);
  std::vector<Bound> element_aabbs(triangles_size);
  Jobs::RunParallelFor(triangles_size, [&](const size_t index) {
    const auto& triangle = triangles[index];
    mesh_bvh.element_indices[index] = static_cast<uint32_t>(index);

    auto& element_aabb = element_aabbs[index];
    element_aabb = {};
    for (int i = 0; i < 3; i++) {
      const auto& vertex_index = triangle[i];
      const auto& p = input_vertices[vertex_index].position;
      element_aabb.min = glm::min(element_aabb.min, p);
      element_aabb.max = glm::min(element_aabb.max, p);
    }
  });

  BinaryDivisionBvh(mesh_bvh, 0, element_aabbs);
  aabb = mesh_bvh.aabb;
  FlattenBvh(mesh_bvh, flattened_bvh_triangle_group, 0);
}

void RayTracer::GeometryInstance::Clear() noexcept {
  flattened_bvh_triangle_group.nodes.clear();
  flattened_bvh_triangle_group.element_indices.clear();
  aabb = {};
  triangles.clear();
  vertex_positions.clear();
}

void RayTracer::NodeInstance::Initialize(const std::shared_ptr<Scene>& input_scene, const Entity& input_entity,
                                            const std::vector<GeometryInstance>& mesh_instances,
                                            const std::map<Handle, uint32_t>& mesh_instances_map) {
  const auto mesh_renderer = input_scene->GetOrSetPrivateComponent<MeshRenderer>(input_entity).lock();
  const auto mesh = mesh_renderer->mesh.Get<Mesh>();
  const auto mesh_index = mesh_instances_map.at(mesh->GetHandle());
  const auto& mesh_instance = mesh_instances[mesh_index];
  transformation = input_scene->GetDataComponent<GlobalTransform>(input_entity);
  inverse_transformation.value = glm::inverse(transformation.value);
  Bvh node_bvh;
  node_bvh.element_indices.resize(1);
  std::vector<Bound> element_aabbs(1);

  node_bvh.element_indices[0] = mesh_index;
  const auto& mesh_aabb = mesh_instance.aabb;
  node_bvh.aabb = element_aabbs[0] = mesh_aabb;

  BinaryDivisionBvh(node_bvh, 0, element_aabbs);
  aabb = node_bvh.aabb;
  FlattenBvh(node_bvh, flattened_bvh_mesh_group, 0);
}

void RayTracer::NodeInstance::Clear() noexcept {
  aabb = {};
  transformation = {};
  inverse_transformation = {};
  flattened_bvh_mesh_group.nodes.clear();
  flattened_bvh_mesh_group.element_indices.clear();
}
