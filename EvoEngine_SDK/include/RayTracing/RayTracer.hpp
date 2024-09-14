#pragma once
#include "Mesh.hpp"
#include "RenderInstances.hpp"
#include "Scene.hpp"
namespace evo_engine {
/**
 * @class RayTracer
 * @brief This class provides the cpu ray tracing service.
 */
class RayTracer final {
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
    bool has_hit = false;
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
     * @brief Index of the intersected node. Can be used to locate entity.
     */
    uint32_t node_index = 0;
    /**
     * @brief Index of the intersected instance. Can be used to locate material.
     */
    uint32_t instance_index = 0;
  };

  /**
   * @brief Initialize ray tracer with single mesh.
   * @param input_mesh Target mesh.
   */
  void Initialize(const std::shared_ptr<Mesh>& input_mesh);

  /**
   * @brief Initialize ray tracer for entire scene.
   * @param render_instances The render instances for initialization.
   * @param mesh_binding Action to set up mesh record for each mesh.
   * @param node_binding Action to set up node record for each node.
   */
  void Initialize(const std::shared_ptr<RenderInstances>& render_instances,
                  const std::function<void(uint32_t mesh_index, const std::shared_ptr<Mesh>& mesh)>& mesh_binding,
                  const std::function<void(uint32_t node_index, const Entity& entity)>& node_binding);
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

  struct BvhNode {
    /**
     * @brief Bounding box of current BVH node.
     */
    Bound aabb{};
    /**
     * @brief Begin of range of elements of next level.
     */
    uint32_t begin_next_level_element_index = 0;
    /**
     * @brief End of range of elements of next level.
     */
    uint32_t end_next_level_element_index = 0;
    /**
     * @brief Index of alternate BVH node if current node is skipped.
     */
    uint32_t alternate_node_index = 0;
  };
  struct AggregatedScene {
    /**
     * @brief Trace a ray within the scene. Function is thread-safe.
     * @param ray_descriptor Configuration for the ray.
     * @param closest_hit_func Action to take for closest hit point.
     * @param miss_func Action to take for escaping ray.
     * @param any_hit_func Action to take for any hit point(s) along the ray shooting path. You should never assume this
     * will cover all possible intersections unless EnforceAnyHit flag is set to on.
     */
    void Trace(const RayDescriptor& ray_descriptor,
               const std::function<void(const HitInfo& hit_info)>& closest_hit_func,
               const std::function<void()>& miss_func,
               const std::function<void(const HitInfo& hit_info)>& any_hit_func) const;

    void TraceGpu(const std::vector<RayDescriptor>& rays, std::vector<HitInfo>& hit_infos, TraceFlags flags);

    //=========================================================================================
    //| GPU Related                                                                           |
    //=========================================================================================
    void InitializeBuffers();
    struct AggregateSceneInfo {
      uint32_t scene_level_bvh_nodes_size = 0;    // 0_x
      uint32_t scene_level_bvh_nodes_offset = 0;  // 0_y
      uint32_t node_indices_offset = 0;           // 0_z
      uint32_t node_infos_offset = 0;             // 0_w
      uint32_t node_level_bvh_nodes_offset = 0;   // 1_x
      uint32_t mesh_indices_offset = 0;           // 1_y

      uint32_t mesh_mappings_offset = 0;         // 1_z
      uint32_t mesh_level_bvh_nodes_offset = 0;  // 1_w

      uint32_t triangle_indices_offset = 0;        // 2_x
      uint32_t triangles_offset = 0;               // 2_y
      uint32_t vertices_offset = 0;        // 2_z
      uint32_t local_triangle_indices_offset = 0;  // 2_w
    };

    AggregateSceneInfo aggregate_scene_info{};
    std::vector<glm::vec4> scene_graph_data{};
    std::vector<glm::vec4> scene_geometry_data{};

    std::shared_ptr<Buffer> aggregate_scene_graph_buffer{};
    std::shared_ptr<Buffer> aggregate_scene_geometry_buffer{};
    std::shared_ptr<Buffer> aggregate_scene_info_buffer{};
  };

  /**
   * @brief Aggregate entire multi-level (scene -> node -> mesh -> primitive) BVH data structure into single linear
   * aggregated scene data structure for GPU acceleration.
   * @return Aggregated scene data.
   */
  [[nodiscard]] AggregatedScene Aggregate() const;
  [[nodiscard]] Entity GetEntity(uint32_t node_index) const;
 private:
  struct FlattenedBvh {
    std::vector<BvhNode> nodes{};
    std::vector<uint32_t> element_indices{};
  };

  struct GeometryInstance {
    Bound aabb{};
    FlattenedBvh flattened_bvh_triangle_group{};
    std::vector<Vertex> vertices{};
    std::vector<glm::uvec3> triangles{};
    void Initialize(const std::shared_ptr<Mesh>& input_mesh);
    void Clear() noexcept;
  };

  struct NodeInstance {
    Bound aabb{};
    uint32_t instance_index = 0;
    Entity entity{};
    GlobalTransform transformation{};
    GlobalTransform inverse_transformation{};
    FlattenedBvh flattened_bvh_mesh_group;
    void Initialize(const std::shared_ptr<RenderInstances>& render_instances, const RenderInstance& render_instance,
                    const std::vector<GeometryInstance>& mesh_instances,
                    const std::map<Handle, uint32_t>& mesh_instances_map);
    void Clear() noexcept;
  };

  enum class Axis { X, Y, Z };
  struct SplitResult {
    Axis axis{};
    float split{};
  };

  struct BucketSplit {
    size_t split_idx{};
    float cost{};
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
    std::vector<Bvh> children{};
    std::vector<uint32_t> element_indices{};
  };

  static SplitResult FindBestSplit(const Bvh& parent, const std::vector<Bound>& aabbs);

  static void BinaryDivisionBvh(Bvh& parent, uint32_t current_tree_depth, const std::vector<Bound>& aabbs);

  static void FlattenBvh(const Bvh& current_bvh, FlattenedBvh& flattened_bvh, uint32_t level);

  static glm::vec3 Barycentric(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);
  Bound aabb_{};
  FlattenedBvh flattened_bvh_node_group_{};
  std::vector<GeometryInstance> geometry_instances_{};
  std::vector<NodeInstance> node_instances_{};
};

}  // namespace evo_engine
