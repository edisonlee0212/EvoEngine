#pragma once
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
     * @brief Index of the intersected node.
     */
    uint32_t node_index = 0;
  };

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
  void Initialize(const std::shared_ptr<Scene>& input_scene,
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

    std::shared_ptr<Buffer> scene_level_bvh_nodes_buffer;

    std::shared_ptr<Buffer> node_indices_buffer;
    std::shared_ptr<Buffer> node_info_list_buffer;
    std::shared_ptr<Buffer> node_level_bvh_nodes_buffer;

    std::shared_ptr<Buffer> mesh_indices_buffer;
    std::shared_ptr<Buffer> mesh_info_list_buffer;
    std::shared_ptr<Buffer> mesh_level_bvh_nodes_buffer;

    std::shared_ptr<Buffer> triangle_indices_buffer;
    std::shared_ptr<Buffer> local_triangle_indices_buffer;
    std::shared_ptr<Buffer> scene_triangles_buffer;
    std::shared_ptr<Buffer> scene_vertex_positions_buffer;

    

  private:
    friend class RayTracer;
    //=========================================================================================
    //| CGScene level                                                                         |
    //=========================================================================================
    /**
     * @brief BVH nodes that helps locate the range of indices of node(CGNode) in [node_indices].
     */
    std::vector<BvhNode> scene_level_bvh_nodes_;

    /**
     * @brief Indices of node(CGNode), used for locating elements in [transforms], [inverse_transforms],
     * [node_level_bvh_node_offsets], and [mesh_indices_offsets].
     */
    std::vector<uint32_t> node_indices_;

    //=========================================================================================
    //| CGNode level                                                                          |
    //=========================================================================================
    /**
     * @brief Node(CGNode)'s transformation matrix.
     */
    std::vector<GlobalTransform> node_transforms_;

    /**
     * @brief Node(CGNode)'s inverse transformation matrix.
     */
    std::vector<GlobalTransform> node_inverse_transforms_;

    /**
     * @brief Offsets that help locating sublist of BVH nodes for current node(CGNode) in [node_level_bvh_nodes].
     */
    std::vector<uint32_t> node_level_bvh_node_offsets_;

    /**
     * @brief Sizes that help locating sublist of BVH nodes for current node(CGNode) in [node_level_bvh_nodes].
     */
    std::vector<uint32_t> node_level_bvh_node_sizes_;

    /**
     * @brief Offsets that help locating sublist of mesh indices for current node(CGNode) in [mesh_indices].
     */
    std::vector<uint32_t> mesh_indices_offsets_;

    /**
     * @brief Sizes that help locating sublist of mesh indices for current node(CGNode) in [mesh_indices].
     */
    std::vector<uint32_t> mesh_indices_sizes_;

    //=========================================================================================

    /**
     * @brief BVH nodes that helps locate the range of indices of mesh(CGMesh) in [mesh_indices].
     */
    std::vector<BvhNode> node_level_bvh_nodes_;

    /**
     * @brief Indices of mesh(CGMesh), used for locating elements in [mesh_level_bvh_node_offsets] and
     * [triangle_indices_offsets].
     */
    std::vector<uint32_t> mesh_indices_;

    //=========================================================================================
    //| CGMesh level                                                                          |
    //=========================================================================================
    /**
     * @brief Offsets that helps locate the range of indices of triangle(CGVecU3) in [mesh_level_bvh_nodes].
     */
    std::vector<uint32_t> mesh_level_bvh_node_offsets_;
    /**
     * @brief Sizes that helps locate the range of indices of triangle(CGVecU3) in [mesh_level_bvh_nodes].
     */
    std::vector<uint32_t> mesh_level_bvh_node_sizes_;
    /**
     * @brief Offsets that help locating sublist of mesh indices for current node(CGNode) in [triangle_indices].
     */
    std::vector<uint32_t> triangle_indices_offsets_;
    /**
     * @brief Sizes that help locating sublist of mesh indices for current node(CGNode) in [triangle_indices].
     */
    std::vector<uint32_t> triangle_indices_sizes_;

    //=========================================================================================

    /**
     * @brief BVH nodes that helps locate the range of indices of triangle(CGVecU3) in [triangle_indices].
     */
    std::vector<BvhNode> mesh_level_bvh_nodes_;

    /**
     * @brief Indices of triangle(CGVecU3)
     */
    std::vector<uint32_t> triangle_indices_;

    /**
     * @brief The indices of triangle within corresponding mesh. Needed by constructing intersection info.
     */
    std::vector<uint32_t> local_triangle_indices_;

    //=========================================================================================
    //| Primitive level                                                                       |
    //=========================================================================================
    /**
     * @brief Triangles of entire scene.
     */
    std::vector<glm::uvec3> triangles_;
    /**
     * @brief Vertex positions of entire scene.
     */
    std::vector<glm::vec3> vertex_positions_;

  };

  /**
   * @brief Aggregate entire multi-level (scene -> node -> mesh -> primitive) BVH data structure into single linear
   * aggregated scene data structure for GPU acceleration.
   * @return Aggregated scene data.
   */
  [[nodiscard]] AggregatedScene Aggregate() const;
 private:
  struct FlattenedBvh {
    std::vector<BvhNode> nodes;
    std::vector<uint32_t> element_indices;
  };

  struct GeometryInstance {
    Bound aabb;
    FlattenedBvh flattened_bvh_triangle_group;
    std::vector<glm::vec3> vertex_positions;
    std::vector<glm::uvec3> triangles;
    void Initialize(const std::shared_ptr<Mesh>& input_mesh);
    void Clear() noexcept;
  };

  struct NodeInstance {
    Bound aabb;
    GlobalTransform transformation{};
    GlobalTransform inverse_transformation{};
    FlattenedBvh flattened_bvh_mesh_group;

    void Initialize(const std::shared_ptr<Scene>& input_scene, const Entity& input_entity,
                    const std::vector<GeometryInstance>& mesh_instances,
                    const std::map<Handle, uint32_t>& mesh_instances_map);
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

  static glm::vec3 Barycentric(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);
  Bound aabb_{};
  FlattenedBvh flattened_bvh_node_group_{};
  std::vector<GeometryInstance> geometry_instances_{};
  std::vector<NodeInstance> node_instances_{};
};

}  // namespace evo_engine
