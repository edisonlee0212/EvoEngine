#pragma once
#include "Mesh.hpp"
namespace evo_engine {
/**
 * @class CpuRayTracer
 * @brief This class provides the cpu ray tracing service.
 */
template <typename MeshRecord, typename NodeRecord>
class CpuRayTracer {
 public:
  enum class TraceFlags {
    // No special flag set.
    None = 0,
    // Enforce any hit programs for all geometries. This flag is mutually exclusive with DisableAnyHit.
    EnforceAnyHit = 1 << 1,
    CullBackFace = 1 << 2,
    CullFrontFace = 1 << 3
  };

  struct RayDescriptor {
    /**
     * @brief Starting point of the ray.
     */
    glm::vec3 origin{};
    /**
     * @brief Direction of the ray. Doesn't have to be normalized. Zero length vector will cause ray being discarded.
     */
    glm::vec3 direction{};
    /**
     * @brief Minimum ray distance.
     */
    float t_min = 0.f;
    /**
     * @brief Maximum ray distance.
     */
    float t_max = FLT_MAX;
    /**
     * @brief Configurations of the ray.
     */
    TraceFlags flags = TraceFlags::None;
  };

  struct HitInfo {
    /**
     * @brief Position of ray triangle intersection.
     */
    glm::vec3 hit{};
    /**
     * @brief Barycentric coordinate of the intersection on the triangle being hit.
     */
    glm::vec3 barycentric{};
    /**
     * @brief If the ray hits the back side of the triangle.
     */
    bool back_face = false;
    /**
     * @brief Normal direction of the triangle.
     */
    glm::vec3 normal{};
    /**
     * @brief Distance from intersection to ray origin.
     */
    float distance = 0.f;
    /**
     * @brief Index of the intersected triangle from the original mesh.
     */
    uint32_t triangle_index = 0;
    /**
     * @brief Index of the intersected mesh.
     */
    uint32_t mesh_index = 0;
    /**
     * @brief Index of the intersected node.
     */
    uint32_t node_index = 0;
  };

  CpuRayTracer() = default;

  ~CpuRayTracer() noexcept = default;

  /**
   * @brief Initialize ray tracer with single mesh.
   * @param input_mesh Target mesh.
   */
  void Initialize(const std::shared_ptr<Mesh>& input_mesh);

  /**
   * @brief Initialize ray tracer for entire scene.
   * @param input_scene The scene for initialization.
   * @param mesh_binding Action to set up mesh record for each mesh.
   * @param node_binding Action to set up node record for each node.
   */
  void Initialize(
      const std::shared_ptr<Scene>& input_scene,
      const std::function<void(const std::shared_ptr<Mesh>& mesh, MeshRecord& mesh_instance_data)>& mesh_binding,
      const std::function<void(const Entity& entity, NodeRecord& node_instance_data)>& node_binding);

  /**
   * @brief Trace a ray within the scene. Function is thread-safe.
   * @param ray_descriptor Configuration for the ray.
   * @param closest_hit_func Action to take for closest hit point.
   * @param miss_func Action to take for escaping ray.
   * @param any_hit_func Action to take for any hit point(s) along the ray shooting path. You should never assume this
   * will cover all possible intersections unless EnforceAnyHit flag is set to on.
   */
  void Trace(const RayDescriptor& ray_descriptor, const std::function<void(const HitInfo& hit_info)>& closest_hit_func,
             const std::function<void()>& miss_func,
             const std::function<void(const HitInfo& hit_info)>& any_hit_func) const;

  /**
   * @brief Clear all data in the ray tracer.
   */
  void Clear() noexcept;

  /**
   * @brief Access the mesh record with given mesh index.
   * @param mesh_index The index of mesh record. Can be retrieved from HitInfo.
   * @return Corresponding mesh record.
   */
  [[nodiscard]] const MeshRecord& PeekMeshRecord(uint32_t mesh_index) const;

  /**
   * @brief Access the node record with given mesh index.
   * @param node_index The index of node record. Can be retrieved from HitInfo.
   * @return Corresponding node record.
   */
  [[nodiscard]] const NodeRecord& PeekNodeRecord(uint32_t node_index) const;
#pragma region Internal
 private:
  struct BvhNode {
    Bound aabb{};
    uint32_t element_start_index = 0;
    uint32_t element_end_index = 0;
    uint32_t next_node_skip = 0;

    uint32_t bvh_level = 0;
  };

  struct FlattenedBvh {
    Bound aabb;
    std::vector<BvhNode> nodes;
    std::vector<uint32_t> element_indices;

    void Clear();

    void CollectBound(uint32_t start_level, uint32_t end_level, std::vector<Bound>& aabbs) const;
  };
  struct MeshInstance {
    FlattenedBvh flattened_bvh_triangle_group;
    std::vector<glm::vec3> vertex_position_list;

    void Initialize(const std::shared_ptr<Mesh>& input_mesh);

    void CollectBound(uint32_t start_level, uint32_t end_level, std::vector<Bound>& aabbs) const;

    void Clear() noexcept;
  };

  struct NodeInstance {
    GlobalTransform global_transformation{};
    GlobalTransform inverse_global_transformation{};
    FlattenedBvh flattened_bvh_mesh_group;

    void Initialize(const std::shared_ptr<Scene>& input_scene, const Entity& input_entity,
                    const std::vector<MeshInstance>& mesh_instances,
                    const std::map<Handle, uint32_t>& mesh_instances_map);

    void CollectBound(uint32_t start_level, uint32_t end_level, std::vector<Bound>& aabbs) const;

    void Clear() noexcept;
  };

  enum class Axis { X, Y, Z };
  struct SplitResult {
    Axis axis;
    float split;
  };

  struct BucketSplit {
    size_t split_idx;
    float cost;
  };

  struct BucketBound {
    glm::vec3 min_v{FLT_MAX, FLT_MAX, FLT_MAX};
    glm::vec3 max_v{-FLT_MAX, -FLT_MAX, -FLT_MAX};

    void AddPoint(const glm::vec3& p);

    void Combine(const BucketBound& other);

    [[nodiscard]] float ComputeCost(int triangle_count) const;
  };

  static BucketSplit SelectSplitFromBuckets(const uint32_t buckets[16], const BucketBound buckets_aabb[16],
                                            size_t triangle_count);

  struct Bvh {
    Bound aabb{};
    uint32_t subtree_element_size = 0;
    std::vector<Bvh> children;
    std::vector<uint32_t> element_indices;
  };

  static SplitResult FindBestSplit(const Bvh& parent, const std::vector<Bound>& aabbs);

  static void BinaryDivisionBvh(Bvh& parent, uint32_t current_tree_depth, const std::vector<Bound>& aabbs);

  static void FlattenBvh(const Bvh& current_bvh, FlattenedBvh& flattened_bvh, uint32_t level);

  static void FlattenMeshBvh(const Bvh& current_bvh, const std::vector<Vertex>& input_vertices,
                             const std::vector<glm::uvec3>& input_triangles, FlattenedBvh& flattened_bvh,
                             std::vector<glm::vec3>& vertex_position_list);

  static glm::vec3 Barycentric(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);

  FlattenedBvh flattened_bvh_node_group_;
  std::vector<MeshInstance> mesh_instances_;
  std::vector<NodeInstance> node_instances_;
  std::vector<MeshRecord> mesh_records_{};
  std::vector<NodeRecord> node_records_{};
#pragma endregion
};
#pragma region Implementation
template <typename MeshRecord, typename NodeRecord>
void CpuRayTracer<MeshRecord, NodeRecord>::Initialize(
    const std::shared_ptr<Scene>& input_scene,
    const std::function<void(const std::shared_ptr<Mesh>& mesh, MeshRecord& mesh_instance_data)>& mesh_binding,
    const std::function<void(const Entity& entity, NodeRecord& node_instance_data)>& node_binding) {
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

    mesh_instances_.emplace_back();
    auto& mesh_instance = mesh_instances_.back();
    mesh_instance.Initialize(mesh);
    mesh_index++;
  }

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
    node_instance.Initialize(input_scene, entity, mesh_instances_, mesh_index_map);
  }

  Bvh scene_bvh;
  scene_bvh.element_indices.resize(node_instances_.size());
  std::vector<Bound> element_aabbs(node_instances_.size());
  scene_bvh.aabb = {};
  for (uint32_t node_index = 0; node_index < node_instances_.size(); node_index++) {
    scene_bvh.element_indices[node_index] = node_index;
    const auto& node_instance = node_instances_[node_index];
    auto node_aabb = node_instance.flattened_bvh_mesh_group.aabb;
    node_aabb.ApplyTransform(node_instance.global_transformation.value);
    element_aabbs[node_index] = node_aabb;
    scene_bvh.aabb.min = glm::min(scene_bvh.aabb.min, node_aabb.min);
    scene_bvh.aabb.max = glm::max(scene_bvh.aabb.max, node_aabb.max);
  }
  mesh_records_.resize(mesh_instances_.size());
  node_records_.resize(node_instances_.size());
  size_t data_index = 0;
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
    mesh_binding(mesh, mesh_records_[data_index]);
    node_binding(entity, node_records_[data_index]);
    data_index++;
  }
  BinaryDivisionBvh(scene_bvh, 0, element_aabbs);
  flattened_bvh_node_group_.aabb = scene_bvh.aabb;
  FlattenBvh(scene_bvh, flattened_bvh_node_group_, 0);
}

template <typename MeshRecord, typename NodeRecord>
void CpuRayTracer<MeshRecord, NodeRecord>::Trace(
    const RayDescriptor& ray_descriptor, const std::function<void(const HitInfo& hit_info)>& closest_hit_func,
    const std::function<void()>& miss_func, const std::function<void(const HitInfo& hit_info)>& any_hit_func) const {
  HitInfo closest_hit_info{};
  closest_hit_info.distance = FLT_MAX;
  bool has_hit = false;
  const auto flags = static_cast<unsigned>(ray_descriptor.flags);
  const bool enforce_any_hit = flags & static_cast<unsigned>(TraceFlags::EnforceAnyHit);
  const bool cull_back_face = flags & static_cast<unsigned>(TraceFlags::CullBackFace);
  const bool cull_front_face = flags & static_cast<unsigned>(TraceFlags::CullFrontFace);
  float test_distance = ray_descriptor.t_max;
  auto ray_aabb = [](const glm::vec3& r_o, const glm::vec3& r_inv_d, const Bound& aabb) {
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
  };

  const auto scene_space_ray_direction = glm::normalize(ray_descriptor.direction);
  const auto& scene_space_ray_origin = ray_descriptor.origin;
  const auto scene_space_inv_ray_direction = glm::vec3(
      1.f / scene_space_ray_direction.x, 1.f / scene_space_ray_direction.y, 1.f / scene_space_ray_direction.z);

  uint32_t node_group_index = 0;
  while (node_group_index < flattened_bvh_node_group_.nodes.size()) {
    const auto& node_group = flattened_bvh_node_group_.nodes.at(node_group_index);
    if (!ray_aabb(ray_descriptor.origin, scene_space_inv_ray_direction, node_group.aabb)) {
      node_group_index = node_group.next_node_skip;
      continue;
    }
    for (size_t test_node_index = node_group.element_start_index; test_node_index < node_group.element_end_index;
         ++test_node_index) {
      uint32_t mesh_group_index = 0;
      const auto& node_instance = node_instances_.at(flattened_bvh_node_group_.element_indices[test_node_index]);
      const auto& node_global_transform = node_instance.global_transformation;
      const auto& node_inverse_global_transform = node_instance.inverse_global_transformation;
      const auto node_space_ray_origin = node_inverse_global_transform.TransformPoint(scene_space_ray_origin);
      const auto node_space_ray_direction =
          glm::normalize(node_inverse_global_transform.TransformVector(scene_space_ray_direction));
      const auto node_space_inv_ray_direction = glm::vec3(
          1.f / node_space_ray_direction.x, 1.f / node_space_ray_direction.y, 1.f / node_space_ray_direction.z);

      while (mesh_group_index < node_instance.flattened_bvh_mesh_group.nodes.size()) {
        const auto& mesh_group = node_instance.flattened_bvh_mesh_group.nodes.at(mesh_group_index);
        if (!ray_aabb(node_space_ray_origin, node_space_inv_ray_direction, mesh_group.aabb)) {
          mesh_group_index = mesh_group.next_node_skip;
          continue;
        }
        for (size_t test_mesh_index = mesh_group.element_start_index; test_mesh_index < mesh_group.element_end_index;
             ++test_mesh_index) {
          uint32_t triangle_group_index = 0;
          const auto& mesh_instance =
              mesh_instances_.at(node_instance.flattened_bvh_mesh_group.element_indices[test_mesh_index]);
          while (triangle_group_index < mesh_instance.flattened_bvh_triangle_group.nodes.size()) {
            const auto& triangle_group = mesh_instance.flattened_bvh_triangle_group.nodes.at(triangle_group_index);
            if (!ray_aabb(node_space_ray_origin, node_space_inv_ray_direction, triangle_group.aabb)) {
              triangle_group_index = triangle_group.next_node_skip;
              continue;
            }
            for (size_t test_triangle_index = triangle_group.element_start_index;
                 test_triangle_index < triangle_group.element_end_index; ++test_triangle_index) {
              const auto& p0 = mesh_instance.vertex_position_list[test_triangle_index * 3];
              const auto& p1 = mesh_instance.vertex_position_list[test_triangle_index * 3 + 1];
              const auto& p2 = mesh_instance.vertex_position_list[test_triangle_index * 3 + 2];
              if (p0 == p1 && p1 == p2)
                continue;
              const auto node_space_triangle_normal = glm::normalize(glm::cross(p1 - p0, p2 - p0));
              const auto normal_test = glm::dot(node_space_ray_direction, node_space_triangle_normal);
              if ((cull_back_face && normal_test > 0.f) || (cull_front_face && normal_test < 0.f) || normal_test == 0.f)
                continue;

              const auto node_space_hit_distance = (glm::dot(p0, node_space_triangle_normal) -
                                                    glm::dot(node_space_ray_origin, node_space_triangle_normal)) /
                                                   normal_test;
              const auto node_space_hit = node_space_ray_origin + node_space_ray_direction * node_space_hit_distance;
              const auto scene_space_hit = node_global_transform.TransformPoint(node_space_hit);
              const auto scene_hit_distance = glm::distance(scene_space_ray_origin, scene_space_hit);
              if (node_space_hit_distance >= 0 && scene_hit_distance >= ray_descriptor.t_min &&
                  scene_hit_distance <= test_distance) {
                if (const auto barycentric = Barycentric(node_space_hit, p0, p1, p2);
                    barycentric.x >= 0.f && barycentric.x <= 1.f && barycentric.y >= 0.f && barycentric.y <= 1.f &&
                    barycentric.z >= 0.f && barycentric.z <= 1.f) {
                  HitInfo any_hit_info;
                  any_hit_info.hit = scene_space_hit;
                  any_hit_info.normal = node_global_transform.TransformVector(node_space_triangle_normal);
                  any_hit_info.distance = scene_hit_distance;
                  any_hit_info.barycentric = barycentric;
                  any_hit_info.back_face = normal_test > 0.f;
                  any_hit_info.triangle_index =
                      mesh_instance.flattened_bvh_triangle_group.element_indices[test_triangle_index];
                  any_hit_info.mesh_index = node_instance.flattened_bvh_mesh_group.element_indices[test_mesh_index];
                  any_hit_info.node_index = flattened_bvh_node_group_.element_indices[test_node_index];
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

template <typename MeshRecord, typename NodeRecord>
void CpuRayTracer<MeshRecord, NodeRecord>::Clear() noexcept {
  flattened_bvh_node_group_.Clear();
  mesh_instances_.clear();
  node_instances_.clear();

  mesh_records_.clear();
  node_records_.clear();
}

template <typename MeshRecord, typename NodeRecord>
const MeshRecord& CpuRayTracer<MeshRecord, NodeRecord>::PeekMeshRecord(const uint32_t mesh_index) const {
  return mesh_records_[mesh_index];
}

template <typename MeshRecord, typename NodeRecord>
const NodeRecord& CpuRayTracer<MeshRecord, NodeRecord>::PeekNodeRecord(const uint32_t node_index) const {
  return node_records_[node_index];
}

template <typename MeshRecord, typename NodeRecord>
void CpuRayTracer<MeshRecord, NodeRecord>::Initialize(const std::shared_ptr<Mesh>& input_mesh) {
  Clear();
  assert(std::is_pod<MeshRecord>() && std::is_pod<NodeRecord>());
  mesh_instances_.resize(1);
  node_instances_.resize(1);
  mesh_instances_[0].Initialize(input_mesh);
  flattened_bvh_node_group_.aabb = node_instances_[0].flattened_bvh_mesh_group.aabb =
      mesh_instances_[0].flattened_bvh_triangle_group.aabb;
  node_instances_[0].flattened_bvh_mesh_group.element_indices = {0};
  node_instances_[0].flattened_bvh_mesh_group.nodes.resize(1);
  node_instances_[0].global_transformation = GlobalTransform();
  auto& node = node_instances_[0].flattened_bvh_mesh_group.nodes[0];
  node.element_start_index = 0;
  node.element_end_index = 1;
  node.next_node_skip = 1;
  node.aabb = flattened_bvh_node_group_.aabb;
}

template <typename MeshRecord, typename NodeRecord>
void CpuRayTracer<MeshRecord, NodeRecord>::FlattenedBvh::CollectBound(const uint32_t start_level,
                                                                      const uint32_t end_level,
                                                                      std::vector<Bound>& aabbs) const {
  for (const auto& node : nodes) {
    if (node.bvh_level >= start_level && node.bvh_level < end_level) {
      aabbs.emplace_back(node.aabb);
    }
  }
}
template <typename MeshRecord, typename NodeRecord>
void CpuRayTracer<MeshRecord, NodeRecord>::BucketBound::AddPoint(const glm::vec3& p) {
  min_v = glm::min(min_v, p);
  max_v = glm::max(max_v, p);
}
template <typename MeshRecord, typename NodeRecord>
void CpuRayTracer<MeshRecord, NodeRecord>::BucketBound::Combine(const BucketBound& other) {
  min_v = glm::min(min_v, other.min_v);
  max_v = glm::max(max_v, other.max_v);
}
template <typename MeshRecord, typename NodeRecord>
float CpuRayTracer<MeshRecord, NodeRecord>::BucketBound::ComputeCost(const int triangle_count) const {
  const auto size = max_v - min_v;
  const float surface_area = 2.0f * (size.x * size.y + size.x * size.z + size.x * size.y);
  return static_cast<float>(triangle_count) * surface_area;
}
template <typename MeshRecord, typename NodeRecord>
typename CpuRayTracer<MeshRecord, NodeRecord>::BucketSplit CpuRayTracer<MeshRecord, NodeRecord>::SelectSplitFromBuckets(
    const uint32_t buckets[16], const BucketBound buckets_aabb[16], const size_t triangle_count) {
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

template <typename MeshRecord, typename NodeRecord>
typename CpuRayTracer<MeshRecord, NodeRecord>::SplitResult CpuRayTracer<MeshRecord, NodeRecord>::FindBestSplit(
    const Bvh& parent, const std::vector<Bound>& aabbs) {
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
template <typename MeshRecord, typename NodeRecord>
void CpuRayTracer<MeshRecord, NodeRecord>::BinaryDivisionBvh(Bvh& parent, const uint32_t current_tree_depth,
                                                             const std::vector<Bound>& aabbs) {
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
template <typename MeshRecord, typename NodeRecord>
void CpuRayTracer<MeshRecord, NodeRecord>::FlattenBvh(const Bvh& current_bvh, FlattenedBvh& flattened_bvh,
                                                      const uint32_t level) {
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
  node.bvh_level = level;
  node.aabb = current_bvh.aabb;
  node.element_start_index = static_cast<uint32_t>(flattened_bvh.element_indices.size());
  flattened_bvh.element_indices.insert(flattened_bvh.element_indices.end(), current_bvh.element_indices.begin(),
                                       current_bvh.element_indices.end());
  node.element_end_index = static_cast<uint32_t>(flattened_bvh.element_indices.size());
  const size_t current_scene_index = flattened_bvh.nodes.size() - 1;
  if (!current_bvh.children.empty()) {
    FlattenBvh(current_bvh.children[0], flattened_bvh, level + 1);
    FlattenBvh(current_bvh.children[1], flattened_bvh, level + 1);
  }
  flattened_bvh.nodes[current_scene_index].next_node_skip = static_cast<uint32_t>(flattened_bvh.nodes.size());
}
template <typename MeshRecord, typename NodeRecord>
glm::vec3 CpuRayTracer<MeshRecord, NodeRecord>::Barycentric(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b,
                                                            const glm::vec3& c) {
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
template <typename MeshRecord, typename NodeRecord>
void CpuRayTracer<MeshRecord, NodeRecord>::MeshInstance::Initialize(const std::shared_ptr<Mesh>& input_mesh) {
  const auto& input_vertices = input_mesh->UnsafeGetVertices();
  const auto& input_triangles = input_mesh->UnsafeGetTriangles();
  Clear();
  Bvh mesh_bvh{};
  const auto input_triangle_size = static_cast<uint32_t>(input_triangles.size());
  mesh_bvh.aabb = input_mesh->GetBound();

  mesh_bvh.element_indices.resize(input_triangle_size);

  std::vector<Bound> element_aabbs(input_triangle_size);
  Jobs::RunParallelFor(input_triangle_size, [&](const size_t index) {
    const auto& triangle = input_triangles[index];
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
  flattened_bvh_triangle_group.aabb = mesh_bvh.aabb;
  FlattenMeshBvh(mesh_bvh, input_vertices, input_triangles, flattened_bvh_triangle_group, vertex_position_list);
}
template <typename MeshRecord, typename NodeRecord>
void CpuRayTracer<MeshRecord, NodeRecord>::FlattenMeshBvh(const Bvh& current_bvh,
                                                          const std::vector<Vertex>& input_vertices,
                                                          const std::vector<glm::uvec3>& input_triangles,
                                                          FlattenedBvh& flattened_bvh,
                                                          std::vector<glm::vec3>& vertex_position_list) {
  if (current_bvh.children.empty() && current_bvh.element_indices.empty()) {
    // Children node without triangles? Skip it
    return;
  }
  if (!current_bvh.children.empty()) {
    // If one of the children does not contain any triangles
    // we can skip this node completely as it is an extra AABB test
    // for nothing
    if (current_bvh.children[0].subtree_element_size > 0 && current_bvh.children[1].subtree_element_size == 0) {
      FlattenMeshBvh(current_bvh.children[0], input_vertices, input_triangles, flattened_bvh, vertex_position_list);
      return;
    }
    if (current_bvh.children[1].subtree_element_size > 0 && current_bvh.children[0].subtree_element_size == 0) {
      FlattenMeshBvh(current_bvh.children[1], input_vertices, input_triangles, flattened_bvh, vertex_position_list);
      return;
    }
  }
  auto& node = flattened_bvh.nodes.emplace_back();
  node.aabb = current_bvh.aabb;
  node.element_start_index = static_cast<uint32_t>(flattened_bvh.element_indices.size());
  for (const auto& triangle_index : current_bvh.element_indices) {
    const auto& triangle = input_triangles[triangle_index];
    const auto& p0 = input_vertices[triangle.x].position;
    const auto& p1 = input_vertices[triangle.y].position;
    const auto& p2 = input_vertices[triangle.z].position;
    vertex_position_list.emplace_back(p0);
    vertex_position_list.emplace_back(p1);
    vertex_position_list.emplace_back(p2);
  }
  flattened_bvh.element_indices.insert(flattened_bvh.element_indices.end(), current_bvh.element_indices.begin(),
                                       current_bvh.element_indices.end());
  node.element_end_index = static_cast<uint32_t>(flattened_bvh.element_indices.size());
  const size_t current_node_index = flattened_bvh.nodes.size() - 1;
  if (!current_bvh.children.empty()) {
    FlattenMeshBvh(current_bvh.children[0], input_vertices, input_triangles, flattened_bvh, vertex_position_list);
    FlattenMeshBvh(current_bvh.children[1], input_vertices, input_triangles, flattened_bvh, vertex_position_list);
  }
  flattened_bvh.nodes[current_node_index].next_node_skip = static_cast<uint32_t>(flattened_bvh.nodes.size());
}

template <typename MeshRecord, typename NodeRecord>
void CpuRayTracer<MeshRecord, NodeRecord>::MeshInstance::Clear() noexcept {
  flattened_bvh_triangle_group.Clear();
  vertex_position_list.clear();
}
template <typename MeshRecord, typename NodeRecord>
void CpuRayTracer<MeshRecord, NodeRecord>::NodeInstance::Initialize(
    const std::shared_ptr<Scene>& input_scene, const Entity& input_entity,
    const std::vector<MeshInstance>& mesh_instances, const std::map<Handle, uint32_t>& mesh_instances_map) {
  const auto mesh_renderer = input_scene->GetOrSetPrivateComponent<MeshRenderer>(input_entity).lock();
  const auto mesh = mesh_renderer->mesh.Get<Mesh>();
  const auto mesh_index = mesh_instances_map.at(mesh->GetHandle());
  const auto& mesh_instance = mesh_instances[mesh_index];
  global_transformation = input_scene->GetDataComponent<GlobalTransform>(input_entity);
  inverse_global_transformation.value = glm::inverse(global_transformation.value);
  Bvh node_bvh;
  node_bvh.element_indices.resize(1);
  std::vector<Bound> element_aabbs(1);

  node_bvh.element_indices[0] = mesh_index;
  const auto& mesh_aabb = mesh_instance.flattened_bvh_triangle_group.aabb;
  node_bvh.aabb = element_aabbs[0] = mesh_aabb;

  BinaryDivisionBvh(node_bvh, 0, element_aabbs);
  flattened_bvh_mesh_group.aabb = node_bvh.aabb;
  FlattenBvh(node_bvh, flattened_bvh_mesh_group, 0);
}
template <typename MeshRecord, typename NodeRecord>
void CpuRayTracer<MeshRecord, NodeRecord>::MeshInstance::CollectBound(const uint32_t start_level,
                                                                      const uint32_t end_level,
                                                                      std::vector<Bound>& aabbs) const {
  flattened_bvh_triangle_group.CollectBound(start_level, end_level, aabbs);
}
template <typename MeshRecord, typename NodeRecord>
void CpuRayTracer<MeshRecord, NodeRecord>::NodeInstance::CollectBound(const uint32_t start_level,
                                                                      const uint32_t end_level,
                                                                      std::vector<Bound>& aabbs) const {
  flattened_bvh_mesh_group.CollectBound(start_level, end_level, aabbs);
}
template <typename MeshRecord, typename NodeRecord>
void CpuRayTracer<MeshRecord, NodeRecord>::NodeInstance::Clear() noexcept {
  flattened_bvh_mesh_group.Clear();
}
template <typename MeshRecord, typename NodeRecord>
void CpuRayTracer<MeshRecord, NodeRecord>::FlattenedBvh::Clear() {
  aabb = {};
  nodes.clear();
  element_indices.clear();
}
#pragma endregion
}  // namespace evo_engine
