#include "RayTracer.hpp"

#include "Material.hpp"
#include "MeshRenderer.hpp"
#include "ProjectManager.hpp"
#include "Shader.hpp"

using namespace evo_engine;
void RayTracer::Initialize(
    const std::shared_ptr<RenderInstances>& render_instances,
    const std::function<void(uint32_t mesh_index, const std::shared_ptr<Mesh>& mesh)>& mesh_binding,
    const std::function<void(uint32_t node_index, const Entity& entity)>& node_binding) {
  Clear();
  uint32_t mesh_index = 0;
  std::map<Handle, uint32_t> mesh_instances_map;
  render_instances->deferred_render_instances.Dispatch([&](const RenderInstance& render_instance) {
    mesh_instances_map[render_instance.instance_index] = mesh_index;
    geometry_instances_.emplace_back();
    auto& mesh_instance = geometry_instances_.back();
    mesh_instance.Initialize(render_instance.mesh);
    mesh_binding(mesh_index, render_instance.mesh);
    mesh_index++;
  });
  uint32_t node_index = 0;
  render_instances->deferred_render_instances.Dispatch([&](const RenderInstance& render_instance) {
    const auto mesh = render_instance.mesh;
    node_instances_.emplace_back();
    auto& node_instance = node_instances_.back();
    node_instance.Initialize(render_instances, render_instance, geometry_instances_, mesh_instances_map);
    node_binding(node_index, render_instance.owner);
    node_index++;
  });

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
      const auto instance_index = node_instance.instance_index;
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
              const auto& p0 = mesh_instance.vertices[triangle.x].position;
              const auto& p1 = mesh_instance.vertices[triangle.y].position;
              const auto& p2 = mesh_instance.vertices[triangle.z].position;
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
                    any_hit_info.instance_index = instance_index;
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
  node_instances_[0].instance_index = 0;
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

RayTracer::BucketSplit RayTracer::SelectSplitFromBuckets(const uint32_t buckets[16], const BucketBound buckets_aabb[16],
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

void RayTracer::AggregatedScene::InitializeBuffers() {
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

  aggregate_scene_graph_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  aggregate_scene_geometry_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  aggregate_scene_info_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  struct GpuSceneInfo {
    glm::vec4 size_0;
    glm::vec4 size_1;
    glm::vec4 size_2;
  };

  GpuSceneInfo gpu_scene_info;
  gpu_scene_info.size_0.x = UintBitsToFloat(aggregate_scene_info.scene_level_bvh_nodes_size);
  gpu_scene_info.size_0.y = UintBitsToFloat(aggregate_scene_info.scene_level_bvh_nodes_offset);
  gpu_scene_info.size_0.z = UintBitsToFloat(aggregate_scene_info.node_indices_offset);
  gpu_scene_info.size_0.w = UintBitsToFloat(aggregate_scene_info.node_infos_offset);

  gpu_scene_info.size_1.x = UintBitsToFloat(aggregate_scene_info.node_level_bvh_nodes_offset);
  gpu_scene_info.size_1.y = UintBitsToFloat(aggregate_scene_info.mesh_indices_offset);
  gpu_scene_info.size_1.z = UintBitsToFloat(aggregate_scene_info.mesh_mappings_offset);
  gpu_scene_info.size_1.w = UintBitsToFloat(aggregate_scene_info.mesh_level_bvh_nodes_offset);

  gpu_scene_info.size_2.x = UintBitsToFloat(aggregate_scene_info.triangle_indices_offset);
  gpu_scene_info.size_2.y = UintBitsToFloat(aggregate_scene_info.triangles_offset);
  gpu_scene_info.size_2.z = UintBitsToFloat(aggregate_scene_info.vertices_offset);
  gpu_scene_info.size_2.w = UintBitsToFloat(aggregate_scene_info.local_triangle_indices_offset);

  aggregate_scene_graph_buffer->UploadVector(scene_graph_data);
  aggregate_scene_geometry_buffer->UploadVector(scene_geometry_data);
  aggregate_scene_info_buffer->Upload(gpu_scene_info);
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
  const auto get_bvh_node = [](const glm::vec4 buffer_0, const glm::vec4 buffer_1, const glm::vec4 buffer_2) {
    BvhNode bvh_node;
    bvh_node.aabb.min = buffer_0;
    bvh_node.aabb.max = buffer_1;
    bvh_node.alternate_node_index = glm::floatBitsToUint(buffer_2.x);
    bvh_node.begin_next_level_element_index = glm::floatBitsToUint(buffer_2.y);
    bvh_node.end_next_level_element_index = glm::floatBitsToUint(buffer_2.z);
    return bvh_node;
  };
  uint32_t node_group_index = 0;
  while (node_group_index < aggregate_scene_info.scene_level_bvh_nodes_size) {
    const auto node_group =
        get_bvh_node(scene_graph_data[aggregate_scene_info.scene_level_bvh_nodes_offset + node_group_index * 3],
                     scene_graph_data[aggregate_scene_info.scene_level_bvh_nodes_offset + node_group_index * 3 + 1],
                     scene_graph_data[aggregate_scene_info.scene_level_bvh_nodes_offset + node_group_index * 3 + 2]);
    if (!RayAabb(scene_space_ray_origin, scene_space_inv_ray_direction, node_group.aabb)) {
      node_group_index = node_group.alternate_node_index;
      continue;
    }
    for (uint32_t test_node_index = node_group.begin_next_level_element_index;
         test_node_index < node_group.end_next_level_element_index; ++test_node_index) {
      uint32_t mesh_group_index = 0;
      const auto node_index = glm::floatBitsToUint(
          scene_graph_data[aggregate_scene_info.node_indices_offset + test_node_index / 4][test_node_index % 4]);
      GlobalTransform node_global_transform;
      node_global_transform.value[0] = scene_graph_data[aggregate_scene_info.node_infos_offset + node_index * 9];
      node_global_transform.value[1] = scene_graph_data[aggregate_scene_info.node_infos_offset + node_index * 9 + 1];
      node_global_transform.value[2] = scene_graph_data[aggregate_scene_info.node_infos_offset + node_index * 9 + 2];
      node_global_transform.value[3] = scene_graph_data[aggregate_scene_info.node_infos_offset + node_index * 9 + 3];
      GlobalTransform node_inverse_global_transform;
      node_inverse_global_transform.value[0] =
          scene_graph_data[aggregate_scene_info.node_infos_offset + node_index * 9 + 4];
      node_inverse_global_transform.value[1] =
          scene_graph_data[aggregate_scene_info.node_infos_offset + node_index * 9 + 5];
      node_inverse_global_transform.value[2] =
          scene_graph_data[aggregate_scene_info.node_infos_offset + node_index * 9 + 6];
      node_inverse_global_transform.value[3] =
          scene_graph_data[aggregate_scene_info.node_infos_offset + node_index * 9 + 7];
      const auto node_space_ray_origin = node_inverse_global_transform.TransformPoint(scene_space_ray_origin);
      auto node_space_ray_direction =
          glm::normalize(node_inverse_global_transform.TransformVector(scene_space_ray_direction));
      const auto node_space_inv_ray_direction = glm::vec3(
          1.f / node_space_ray_direction.x, 1.f / node_space_ray_direction.y, 1.f / node_space_ray_direction.z);
      const auto node_level_bvh_node_offset =
          glm::floatBitsToUint(scene_graph_data[aggregate_scene_info.node_infos_offset + node_index * 9 + 8].x);
      const auto node_level_bvh_node_size =
          glm::floatBitsToUint(scene_graph_data[aggregate_scene_info.node_infos_offset + node_index * 9 + 8].y);
      const auto mesh_index_offset =
          glm::floatBitsToUint(scene_graph_data[aggregate_scene_info.node_infos_offset + node_index * 9 + 8].z);
      const auto instance_index =
          glm::floatBitsToUint(scene_graph_data[aggregate_scene_info.node_infos_offset + node_index * 9 + 8].w);
      while (mesh_group_index < node_level_bvh_node_size) {
        const auto mesh_group = get_bvh_node(scene_graph_data[aggregate_scene_info.node_level_bvh_nodes_offset +
                                                              (mesh_group_index + node_level_bvh_node_offset) * 3],
                                             scene_graph_data[aggregate_scene_info.node_level_bvh_nodes_offset +
                                                              (mesh_group_index + node_level_bvh_node_offset) * 3 + 1],
                                             scene_graph_data[aggregate_scene_info.node_level_bvh_nodes_offset +
                                                              (mesh_group_index + node_level_bvh_node_offset) * 3 + 2]);
        if (!RayAabb(node_space_ray_origin, node_space_inv_ray_direction, mesh_group.aabb)) {
          mesh_group_index = mesh_group.alternate_node_index;
          continue;
        }
        for (uint32_t test_mesh_index = mesh_group.begin_next_level_element_index + mesh_index_offset;
             test_mesh_index < mesh_group.end_next_level_element_index + mesh_index_offset; ++test_mesh_index) {
          uint32_t triangle_group_index = 0;
          const auto mesh_index =
              glm::floatBitsToUint(scene_graph_data[aggregate_scene_info.mesh_indices_offset + test_mesh_index / 4]
                                                   [test_mesh_index % 4]);  //>>>--> This doesn't work on dx.

          const auto mesh_level_bvh_node_offset =
              glm::floatBitsToUint(scene_geometry_data[aggregate_scene_info.mesh_mappings_offset + mesh_index].x);
          const auto mesh_level_bvh_node_size =
              glm::floatBitsToUint(scene_geometry_data[aggregate_scene_info.mesh_mappings_offset + mesh_index].y);
          const auto triangle_index_offset =
              glm::floatBitsToUint(scene_geometry_data[aggregate_scene_info.mesh_mappings_offset + mesh_index].z);
          while (triangle_group_index < mesh_level_bvh_node_size) {
            const auto triangle_group =
                get_bvh_node(scene_geometry_data[aggregate_scene_info.mesh_level_bvh_nodes_offset +
                                                 (triangle_group_index + mesh_level_bvh_node_offset) * 3],
                             scene_geometry_data[aggregate_scene_info.mesh_level_bvh_nodes_offset +
                                                 (triangle_group_index + mesh_level_bvh_node_offset) * 3 + 1],
                             scene_geometry_data[aggregate_scene_info.mesh_level_bvh_nodes_offset +
                                                 (triangle_group_index + mesh_level_bvh_node_offset) * 3 + 2]);
            if (!RayAabb(node_space_ray_origin, node_space_inv_ray_direction, triangle_group.aabb)) {
              triangle_group_index = triangle_group.alternate_node_index;
              continue;
            }
            for (uint32_t test_triangle_index = triangle_group.begin_next_level_element_index + triangle_index_offset;
                 test_triangle_index < triangle_group.end_next_level_element_index + triangle_index_offset;
                 ++test_triangle_index) {
              const auto triangle_index =
                  glm::floatBitsToUint(scene_geometry_data[aggregate_scene_info.triangle_indices_offset +
                                                           test_triangle_index / 4][test_triangle_index % 4]);
              const auto t_x =
                  glm::floatBitsToUint(scene_geometry_data[aggregate_scene_info.triangles_offset + triangle_index].x);
              const auto t_y =
                  glm::floatBitsToUint(scene_geometry_data[aggregate_scene_info.triangles_offset + triangle_index].y);
              const auto t_z =
                  glm::floatBitsToUint(scene_geometry_data[aggregate_scene_info.triangles_offset + triangle_index].z);
              glm::vec3 p0 = scene_geometry_data[aggregate_scene_info.vertices_offset + t_x * 5];
              glm::vec3 p1 = scene_geometry_data[aggregate_scene_info.vertices_offset + t_y * 5];
              glm::vec3 p2 = scene_geometry_data[aggregate_scene_info.vertices_offset + t_z * 5];
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
                    any_hit_info.triangle_index =
                        glm::floatBitsToUint(scene_geometry_data[aggregate_scene_info.local_triangle_indices_offset +
                                                                 test_triangle_index / 4][test_triangle_index % 4]);
                    any_hit_info.mesh_index = mesh_index;
                    any_hit_info.node_index = node_index;
                    any_hit_info.instance_index = instance_index;
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

std::shared_ptr<DescriptorSetLayout> trace_descriptor_set_layout;
std::shared_ptr<Shader> trace_shader{};
std::shared_ptr<ComputePipeline> trace_pipeline{};
struct AggregateSceneInternal {
  //=========================================================================================
  //| Scene level                                                                           |
  //=========================================================================================
  /**
   * @brief BVH nodes that helps locate the range of indices of node in [node_indices].
   */
  std::vector<RayTracer::BvhNode> scene_level_bvh_nodes;

  /**
   * @brief Indices of node, used for locating elements in [transforms], [inverse_transforms],
   * [node_level_bvh_node_offsets], and [mesh_indices_offsets].
   */
  std::vector<uint32_t> node_indices;

  //=========================================================================================
  //| Node level                                                                            |
  //=========================================================================================
  struct AggregatedSceneMapping {
    /**
     * @brief Offsets that help locating sublist of BVH nodes.
     */
    uint32_t bvh_node_offset;
    /**
     * @brief Sizes that help locating sublist of BVH nodes.
     */
    uint32_t bvh_node_size;
    /**
     * @brief Offsets that help locating index in sublist indices.
     */
    uint32_t indices_offset;
  };
  struct AggregatedSceneNodeInfo {
    /**
     * @brief Node's transformation matrix.
     */
    GlobalTransform transform;
    /**
     * @brief Node's inverse transformation matrix.
     */
    GlobalTransform inverse_transform;
    /**
     * @brief Entity's instance index.
     */
    uint32_t instance_index;
    AggregatedSceneMapping mapping;
  };

  std::vector<AggregatedSceneNodeInfo> node_infos;

  //=========================================================================================

  /**
   * @brief BVH nodes that helps locate the range of indices of mesh in [mesh_indices].
   */
  std::vector<RayTracer::BvhNode> node_level_bvh_nodes;

  /**
   * @brief Indices of mesh, used for locating elements in [mesh_level_bvh_node_offsets] and
   * [triangle_indices_offsets].
   */
  std::vector<uint32_t> mesh_indices;

  //=========================================================================================
  //| Mesh level                                                                            |
  //=========================================================================================
  std::vector<AggregatedSceneMapping> mesh_mappings;
  //=========================================================================================

  /**
   * @brief BVH nodes that helps locate the range of indices of triangle in [triangle_indices].
   */
  std::vector<RayTracer::BvhNode> mesh_level_bvh_nodes;

  /**
   * @brief Indices of triangle
   */
  std::vector<uint32_t> triangle_indices;

  /**
   * @brief The indices of triangle within corresponding mesh. Needed by constructing intersection info.
   */
  std::vector<uint32_t> local_triangle_indices;

  //=========================================================================================
  //| Primitive level                                                                       |
  //=========================================================================================
  /**
   * @brief Triangles of entire scene.
   */
  std::vector<glm::uvec3> triangles;
  /**
   * @brief Vertex positions of entire scene.
   */
  std::vector<Vertex> vertices;
};
void RayTracer::AggregatedScene::TraceGpu(const std::vector<RayDescriptor>& rays, std::vector<HitInfo>& hit_infos,
                                          const TraceFlags flags) {
  if (!trace_descriptor_set_layout) {
    trace_descriptor_set_layout = std::make_shared<DescriptorSetLayout>();
    trace_descriptor_set_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);
    trace_descriptor_set_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);
    trace_descriptor_set_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);

    trace_descriptor_set_layout->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);
    trace_descriptor_set_layout->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                            VK_SHADER_STAGE_COMPUTE_BIT, 0);

    trace_descriptor_set_layout->Initialize();
  }

  if (!trace_shader) {
    trace_shader = ProjectManager::CreateTemporaryAsset<Shader>();
    trace_shader->Set(ShaderType::Compute, std::filesystem::path("./DefaultResources") / "Shaders/Compute/Trace.comp");
  }

  if (!trace_pipeline) {
    trace_pipeline = std::make_shared<ComputePipeline>();
    trace_pipeline->descriptor_set_layouts.emplace_back(trace_descriptor_set_layout);
    auto& push_constant_range = trace_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(glm::vec4);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    trace_pipeline->compute_shader = trace_shader;
    trace_pipeline->Initialize();
  }
  if (!aggregate_scene_graph_buffer || !aggregate_scene_geometry_buffer || !aggregate_scene_info_buffer)
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

  const auto ray_casting_results_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  struct GpuRayCastingResult {
    glm::vec4 hit{};
    glm::vec4 barycentric_back_face{};
    glm::vec4 normal_distance{};
    glm::vec4 node_mesh_triangle_indices{};
    GpuRayCastingResult() = default;
  };

  ray_casting_results_buffer->Resize(sizeof(GpuRayCastingResult) * rays.size());
  upload_rays(rays_buffer, rays);

  const auto ray_tracer_descriptor_set = std::make_shared<DescriptorSet>(trace_descriptor_set_layout);

  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(0, aggregate_scene_graph_buffer);

  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(1, aggregate_scene_geometry_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(2, aggregate_scene_info_buffer);

  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(3, rays_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(4, ray_casting_results_buffer);

  const glm::vec4 ray_cast_config =
      glm::vec4(static_cast<unsigned>(flags) | static_cast<unsigned>(TraceFlags::CullBackFace) ? 0.f : 1.f,
                static_cast<unsigned>(flags) | static_cast<unsigned>(TraceFlags::CullFrontFace) ? 0.f : 1.f,
                UintBitsToFloat(static_cast<uint32_t>(rays.size())), 0.f);
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    trace_pipeline->Bind(vk_command_buffer);
    trace_pipeline->BindDescriptorSet(vk_command_buffer, 0, ray_tracer_descriptor_set->GetVkDescriptorSet());
    trace_pipeline->PushConstant(vk_command_buffer, 0, ray_cast_config);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(static_cast<uint32_t>(rays.size()), 256), 1, 1);
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

    hit_info.instance_index = FloatBitsToUint(gpu_ray_casting_result.node_mesh_triangle_indices.x);
    hit_info.node_index = FloatBitsToUint(gpu_ray_casting_result.node_mesh_triangle_indices.y);
    hit_info.mesh_index = FloatBitsToUint(gpu_ray_casting_result.node_mesh_triangle_indices.z);
    hit_info.triangle_index = FloatBitsToUint(gpu_ray_casting_result.node_mesh_triangle_indices.w);
  });
}

RayTracer::AggregatedScene RayTracer::Aggregate() const {
  AggregateSceneInternal aggregated_scene_internal;
  aggregated_scene_internal.scene_level_bvh_nodes = flattened_bvh_node_group_.nodes;
  aggregated_scene_internal.node_indices = flattened_bvh_node_group_.element_indices;

  aggregated_scene_internal.node_infos.resize(node_instances_.size());
  for (uint32_t node_index = 0; node_index < node_instances_.size(); node_index++) {
    const auto& node_instance = node_instances_[node_index];
    aggregated_scene_internal.node_infos[node_index].transform = node_instance.transformation;
    aggregated_scene_internal.node_infos[node_index].instance_index = node_instance.instance_index;
    aggregated_scene_internal.node_infos[node_index].inverse_transform = node_instance.inverse_transformation;
    aggregated_scene_internal.node_infos[node_index].mapping.bvh_node_offset =
        static_cast<uint32_t>(aggregated_scene_internal.node_level_bvh_nodes.size());
    aggregated_scene_internal.node_infos[node_index].mapping.bvh_node_size =
        static_cast<uint32_t>(node_instance.flattened_bvh_mesh_group.nodes.size());
    aggregated_scene_internal.node_infos[node_index].mapping.indices_offset =
        static_cast<uint32_t>(aggregated_scene_internal.mesh_indices.size());

    aggregated_scene_internal.node_level_bvh_nodes.insert(aggregated_scene_internal.node_level_bvh_nodes.end(),
                                                          node_instance.flattened_bvh_mesh_group.nodes.begin(),
                                                          node_instance.flattened_bvh_mesh_group.nodes.end());
    aggregated_scene_internal.mesh_indices.insert(aggregated_scene_internal.mesh_indices.end(),
                                                  node_instance.flattened_bvh_mesh_group.element_indices.begin(),
                                                  node_instance.flattened_bvh_mesh_group.element_indices.end());
  }
  aggregated_scene_internal.mesh_mappings.resize(geometry_instances_.size());
  aggregated_scene_internal.mesh_mappings.resize(geometry_instances_.size());
  aggregated_scene_internal.mesh_mappings.resize(geometry_instances_.size());
  aggregated_scene_internal.mesh_mappings.resize(geometry_instances_.size());
  for (uint32_t mesh_index = 0; mesh_index < geometry_instances_.size(); mesh_index++) {
    const auto& mesh_instance = geometry_instances_[mesh_index];
    aggregated_scene_internal.mesh_mappings[mesh_index].bvh_node_offset =
        static_cast<uint32_t>(aggregated_scene_internal.mesh_level_bvh_nodes.size());
    aggregated_scene_internal.mesh_mappings[mesh_index].bvh_node_size =
        static_cast<uint32_t>(mesh_instance.flattened_bvh_triangle_group.nodes.size());
    aggregated_scene_internal.mesh_mappings[mesh_index].indices_offset =
        static_cast<uint32_t>(aggregated_scene_internal.triangle_indices.size());
    aggregated_scene_internal.mesh_level_bvh_nodes.insert(aggregated_scene_internal.mesh_level_bvh_nodes.end(),
                                                          mesh_instance.flattened_bvh_triangle_group.nodes.begin(),
                                                          mesh_instance.flattened_bvh_triangle_group.nodes.end());
    const auto& vertex = mesh_instance.vertices;
    const auto& triangles = mesh_instance.triangles;
    const auto vertex_offset = static_cast<uint32_t>(aggregated_scene_internal.vertices.size());
    const auto triangle_offset = static_cast<uint32_t>(aggregated_scene_internal.triangles.size());
    aggregated_scene_internal.vertices.insert(aggregated_scene_internal.vertices.end(), vertex.begin(), vertex.end());
    aggregated_scene_internal.triangles.resize(triangle_offset + triangles.size());
    for (uint32_t i = 0; i < triangles.size(); i++) {
      aggregated_scene_internal.triangles[triangle_offset + i] = triangles[i] + vertex_offset;
    }
    const auto& local_triangle_indices = mesh_instance.flattened_bvh_triangle_group.element_indices;
    const uint32_t global_triangle_indices_offset =
        static_cast<uint32_t>(aggregated_scene_internal.triangle_indices.size());
    aggregated_scene_internal.triangle_indices.resize(global_triangle_indices_offset + local_triangle_indices.size());
    for (uint32_t i = 0; i < local_triangle_indices.size(); i++) {
      aggregated_scene_internal.triangle_indices[global_triangle_indices_offset + i] =
          local_triangle_indices[i] + triangle_offset;
    }

    aggregated_scene_internal.local_triangle_indices.insert(
        aggregated_scene_internal.local_triangle_indices.end(),
        mesh_instance.flattened_bvh_triangle_group.element_indices.begin(),
        mesh_instance.flattened_bvh_triangle_group.element_indices.end());
  }

#pragma region Upload functions

  auto upload_bvh_nodes = [&](std::vector<glm::vec4>& destination, const std::vector<BvhNode>& bvh_nodes) {
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
    const uint32_t offset = destination.size();
    destination.resize(destination.size() + gpu_bvh_nodes.size() * 3);
    memcpy(&destination[offset], gpu_bvh_nodes.data(), gpu_bvh_nodes.size() * sizeof(GpuBvhNode));
    return offset;
  };

  auto upload_node_info_list =
      [&](std::vector<glm::vec4>& destination,
          const std::vector<AggregateSceneInternal::AggregatedSceneNodeInfo>& aggregated_scene_node_infos) {
        struct GpuNodeInfo {
          glm::mat4 transform{};
          glm::mat4 inverse_transform{};
          glm::vec4 offsets{};
          GpuNodeInfo() = default;

          explicit GpuNodeInfo(const glm::mat4& src_transform, const glm::mat4& src_inverse_transform,
                               const uint32_t node_level_bvh_node_offset, const uint32_t node_level_bvh_node_size,
                               const uint32_t mesh_indices_offset, const uint32_t instance_index) {
            transform = src_transform;
            inverse_transform = src_inverse_transform;
            offsets[0] = UintBitsToFloat(node_level_bvh_node_offset);
            offsets[1] = UintBitsToFloat(node_level_bvh_node_size);
            offsets[2] = UintBitsToFloat(mesh_indices_offset);
            offsets[3] = UintBitsToFloat(instance_index);
          }
        };

        auto node_infos = std::vector(aggregated_scene_node_infos.size(), GpuNodeInfo());
        Jobs::RunParallelFor(node_infos.size(), [&](const auto i) {
          node_infos[i] = GpuNodeInfo(
              aggregated_scene_node_infos[i].transform.value, aggregated_scene_node_infos[i].inverse_transform.value,
              aggregated_scene_node_infos[i].mapping.bvh_node_offset,
              aggregated_scene_node_infos[i].mapping.bvh_node_size,
              aggregated_scene_node_infos[i].mapping.indices_offset, aggregated_scene_node_infos[i].instance_index);
        });
        const uint32_t offset = destination.size();
        destination.resize(destination.size() + node_infos.size() * 9);
        memcpy(&destination[offset], node_infos.data(), node_infos.size() * sizeof(GpuNodeInfo));
        return offset;
      };

  auto upload_mesh_info_list = [&](std::vector<glm::vec4>& destination,
                                   const std::vector<AggregateSceneInternal::AggregatedSceneMapping>& mappings) {
    struct GpuMeshInfo {
      glm::vec4 offsets{};
      GpuMeshInfo() = default;

      explicit GpuMeshInfo(const uint32_t mesh_level_bvh_node_offset, const uint32_t mesh_level_bvh_node_size,
                           const uint32_t triangle_indices_offset) {
        offsets[0] = UintBitsToFloat(mesh_level_bvh_node_offset);
        offsets[1] = UintBitsToFloat(mesh_level_bvh_node_size);
        offsets[2] = UintBitsToFloat(triangle_indices_offset);
        offsets[3] = 0.0f;
      }
    };

    auto mesh_infos = std::vector(mappings.size(), GpuMeshInfo());
    Jobs::RunParallelFor(mappings.size(), [&](const auto i) {
      mesh_infos[i] = GpuMeshInfo(mappings[i].bvh_node_offset, mappings[i].bvh_node_size, mappings[i].indices_offset);
    });
    const uint32_t offset = destination.size();
    destination.resize(destination.size() + mesh_infos.size());
    memcpy(&destination[offset], mesh_infos.data(), mesh_infos.size() * sizeof(GpuMeshInfo));
    return offset;
  };

  auto upload_indices = [&](std::vector<glm::vec4>& destination, const std::vector<uint32_t>& src) {
    const auto src_length = src.size();
    const auto dst_length = Platform::DivUp(static_cast<uint32_t>(src_length), 4);
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
    const uint32_t offset = destination.size();
    destination.insert(destination.end(), gpu_indices.begin(), gpu_indices.end());
    return offset;
  };

  auto upload_triangles = [&](std::vector<glm::vec4>& destination, const std::vector<glm::uvec3>& src) {
    std::vector<glm::vec4> gpu_triangles;
    gpu_triangles.resize(src.size());
    Jobs::RunParallelFor(src.size(), [&](const auto i) {
      gpu_triangles[i].x = UintBitsToFloat(src[i].x);
      gpu_triangles[i].y = UintBitsToFloat(src[i].y);
      gpu_triangles[i].z = UintBitsToFloat(src[i].z);
      gpu_triangles[i].w = 0.f;
    });
    const uint32_t offset = destination.size();
    destination.insert(destination.end(), gpu_triangles.begin(), gpu_triangles.end());
    return offset;
  };

  auto upload_vertices = [&](std::vector<glm::vec4>& destination, const std::vector<Vertex>& src) {
    std::vector<glm::vec4> gpu_vertices;
    gpu_vertices.resize(src.size() * 5);
    Jobs::RunParallelFor(src.size(), [&](const auto i) {
      const auto& vertex = src[i];
      gpu_vertices[i * 5] = glm::vec4(vertex.position.x, vertex.position.y, vertex.position.z, vertex.vertex_info1);
      gpu_vertices[i * 5 + 1] = glm::vec4(vertex.normal.x, vertex.normal.y, vertex.normal.z, vertex.vertex_info2);
      gpu_vertices[i * 5 + 2] = glm::vec4(vertex.tangent.x, vertex.tangent.y, vertex.tangent.z, vertex.vertex_info3);
      gpu_vertices[i * 5 + 3] = vertex.color;
      gpu_vertices[i * 5 + 4] =
          glm::vec4(vertex.tex_coord.x, vertex.tex_coord.y, vertex.vertex_info4.x, vertex.vertex_info4.y);
    });
    const uint32_t offset = destination.size();
    destination.insert(destination.end(), gpu_vertices.begin(), gpu_vertices.end());
    return offset;
  };

#pragma endregion
  AggregatedScene aggregated_scene;

  aggregated_scene.scene_geometry_data.clear();
  aggregated_scene.scene_graph_data.clear();

  aggregated_scene.aggregate_scene_info.scene_level_bvh_nodes_size =
      static_cast<uint32_t>(aggregated_scene_internal.scene_level_bvh_nodes.size());

  aggregated_scene.aggregate_scene_info.scene_level_bvh_nodes_offset =
      upload_bvh_nodes(aggregated_scene.scene_graph_data, aggregated_scene_internal.scene_level_bvh_nodes);
  aggregated_scene.aggregate_scene_info.node_indices_offset =
      upload_indices(aggregated_scene.scene_graph_data, aggregated_scene_internal.node_indices);

  aggregated_scene.aggregate_scene_info.node_infos_offset =
      upload_node_info_list(aggregated_scene.scene_graph_data, aggregated_scene_internal.node_infos);
  aggregated_scene.aggregate_scene_info.node_level_bvh_nodes_offset =
      upload_bvh_nodes(aggregated_scene.scene_graph_data, aggregated_scene_internal.node_level_bvh_nodes);
  aggregated_scene.aggregate_scene_info.mesh_indices_offset =
      upload_indices(aggregated_scene.scene_graph_data, aggregated_scene_internal.mesh_indices);

  aggregated_scene.aggregate_scene_info.mesh_mappings_offset =
      upload_mesh_info_list(aggregated_scene.scene_geometry_data, aggregated_scene_internal.mesh_mappings);
  aggregated_scene.aggregate_scene_info.mesh_level_bvh_nodes_offset =
      upload_bvh_nodes(aggregated_scene.scene_geometry_data, aggregated_scene_internal.mesh_level_bvh_nodes);

  aggregated_scene.aggregate_scene_info.triangle_indices_offset =
      upload_indices(aggregated_scene.scene_geometry_data, aggregated_scene_internal.triangle_indices);
  aggregated_scene.aggregate_scene_info.triangles_offset =
      upload_triangles(aggregated_scene.scene_geometry_data, aggregated_scene_internal.triangles);
  aggregated_scene.aggregate_scene_info.vertices_offset =
      upload_vertices(aggregated_scene.scene_geometry_data, aggregated_scene_internal.vertices);

  aggregated_scene.aggregate_scene_info.local_triangle_indices_offset =
      upload_indices(aggregated_scene.scene_geometry_data, aggregated_scene_internal.local_triangle_indices);

  return aggregated_scene;
}

Entity RayTracer::GetEntity(const uint32_t node_index) const {
  return node_instances_[node_index].entity;
}

void RayTracer::GeometryInstance::Initialize(const std::shared_ptr<Mesh>& input_mesh) {
  Clear();
  const auto& input_vertices = input_mesh->UnsafeGetVertices();
  triangles = input_mesh->UnsafeGetTriangles();
  Bvh mesh_bvh{};
  const auto vertices_size = static_cast<uint32_t>(input_vertices.size());
  const auto triangles_size = static_cast<uint32_t>(triangles.size());
  mesh_bvh.aabb = input_mesh->GetBound();
  vertices.resize(vertices_size);
  Jobs::RunParallelFor(vertices_size, [&](const size_t index) {
    vertices[index] = input_vertices[index];
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
  vertices.clear();
}

void RayTracer::NodeInstance::Initialize(const std::shared_ptr<RenderInstances>& render_instances,
                                         const RenderInstance& render_instance,
                                         const std::vector<GeometryInstance>& mesh_instances,
                                         const std::map<Handle, uint32_t>& mesh_instances_map) {
  const auto mesh_index = mesh_instances_map.at(render_instance.instance_index);
  const auto& mesh_instance = mesh_instances[mesh_index];
  transformation = render_instances->target_scene->GetDataComponent<GlobalTransform>(render_instance.owner);
  instance_index = render_instance.instance_index;
  inverse_transformation.value = glm::inverse(transformation.value);
  entity = render_instance.owner;
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
  instance_index = 0;
  flattened_bvh_mesh_group.nodes.clear();
  flattened_bvh_mesh_group.element_indices.clear();
}
