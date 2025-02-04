
#pragma once
#include "Mesh.hpp"
#include "RenderInstanceStorage.hpp"
#include "Scene.hpp"

#include "PointCloudSample.hpp"

namespace evo_engine {
/**
 * @class CpuRayTracer
 * @brief This class provides the CPU-based ray tracing service for 3D rendering.
 */
class CpuRayTracer final {
 public:
  /**
   * @enum TraceFlags
   * @brief Flags used to configure ray tracing behavior.
   */
  enum class TraceFlags {
    /**
     * @brief No special flag set.
     */
    Default = 0,
    /**
     * @brief Enforce any hit programs for all geometries. Mutually exclusive with DisableAnyHit.
     */
    EnforceAnyHit = 1 << 1,
    /**
     * @brief Cull (ignore) back-facing triangles during ray intersection tests.
     */
    CullBackFace = 1 << 2,
    /**
     * @brief Cull (ignore) front-facing triangles during ray intersection tests.
     */
    CullFrontFace = 1 << 3
  };

  /**
   * @struct RayDescriptor
   * @brief Describes a ray to be traced, including its origin, direction, and attributes.
   */
  struct RayDescriptor {
    /**
     * @brief Starting point of the ray.
     */
    glm::vec3 origin{};
    /**
     * @brief Direction of the ray. Does not need to be normalized. Rays with zero-length direction are discarded.
     */
    glm::vec3 direction{};
    /**
     * @brief Minimum ray distance for intersection tests.
     */
    float t_min = 0.f;
    /**
     * @brief Maximum ray distance for intersection tests.
     */
    float t_max = FLT_MAX;
    /**
     * @brief Configurations of the ray, determined using flags.
     */
    TraceFlags flags = TraceFlags::Default;
  };

  /**
   * @struct HitInfo
   * @brief Contains detailed information about a hit result from ray tracing.
   */
  struct HitInfo {
    /**
     * @brief Indicates whether the ray hit any geometry.
     */
    bool has_hit = false;
    /**
     * @brief Position of the ray-triangle intersection in 3D space.
     */
    glm::vec3 hit{};
    /**
     * @brief Barycentric coordinates of the intersection on the triangle.
     */
    glm::vec3 barycentric{};
    /**
     * @brief Set to true if the ray hit the back face of the triangle.
     */
    bool back_face = false;
    /**
     * @brief Surface normal of the hit triangle at the intersection point.
     */
    glm::vec3 normal{};
    /**
     * @brief Distance between the ray origin and the intersection point.
     */
    float distance = 0.f;
    /**
     * @brief Index of the intersected triangle in the originating mesh.
     */
    uint32_t triangle_index = 0;
    /**
     * @brief Index of the intersected mesh in the scene.
     */
    uint32_t mesh_index = 0;
    /**
     * @brief Index of the intersected node. Useful for locating an entity in the scene graph.
     */
    uint32_t node_index = 0;
    /**
     * @brief Index of the intersected instance. Can be used to link to material or instance-specific data.
     */
    uint32_t instance_index = 0;
  };

  /**
   * @brief Initialize the ray tracer with a single mesh.
   * @param input_mesh A shared pointer to the target mesh for the ray tracer.
   */
  void Initialize(const std::shared_ptr<Mesh>& input_mesh);

  /**
   * @brief Initialize the ray tracer for an entire scene.
   * @param render_instances A storage pointer containing render instances to initialize the scene.
   * @param mesh_binding A callable to set up each mesh's record. Takes `mesh_index` and the corresponding mesh.
   * @param node_binding A callable to set up records for each node. Takes `node_index` and the corresponding entity.
   */
  void Initialize(const std::shared_ptr<RenderInstanceStorage>& render_instances,
                  const std::function<void(uint32_t mesh_index, const std::shared_ptr<Mesh>& mesh)>& mesh_binding,
                  const std::function<void(uint32_t node_index, const Entity& entity)>& node_binding);

  /**
   * @brief Trace a ray within the scene. This function is thread-safe.
   * @param ray_descriptor The description of the ray to be traced.
   * @param closest_hit_func A function to handle the closest hit information.
   * @param miss_func A function to handle when the ray does not hit any geometry.
   * @param any_hit_func A function to handle intermediate hit points along the ray's path.
   *        Note: This function may not handle all possible intersections unless the EnforceAnyHit flag is enabled.
   */
  void Trace(const RayDescriptor& ray_descriptor, const std::function<void(const HitInfo& hit_info)>& closest_hit_func,
             const std::function<void()>& miss_func,
             const std::function<void(const HitInfo& hit_info)>& any_hit_func) const;

  /**
   * @brief Trace a ray within the scene and return only the closest hit information. This function is thread-safe.
   * @param ray_descriptor The description of the ray to be traced.
   * @param closest_hit_info The output structure to store the closest hit information.
   */
  void Trace(const RayDescriptor& ray_descriptor, HitInfo& closest_hit_info) const;

  /**
   * @brief Sample points from the point cloud within the scene.
   * @param samples A vector to store the sampled points.
   */
  void SamplePointCloud(std::vector<PointCloudSample>& samples) const;

  /**
   * @brief Clear all the data stored in the ray tracer.
   */
  void Clear() noexcept;

  /**
   * @struct BvhNode
   * @brief Represents a single node in the Bounding Volume Hierarchy (BVH) tree.
   */
  struct BvhNode {
    /**
     * @brief Axis-Aligned Bounding Box (AABB) of the current BVH node.
     */
    Bound aabb{};
    /**
     * @brief Index for the beginning of the range of elements at the next level.
     */
    uint32_t begin_next_level_element_index = 0;
    /**
     * @brief Index for the end of the range of elements at the next level.
     */
    uint32_t end_next_level_element_index = 0;
    /**
     * @brief Index of an alternate BVH node if the current node is skipped.
     */
    uint32_t alternate_node_index = 0;
  };

  /**
   * @struct AggregatedScene
   * @brief Holds aggregated data for the entire scene, optimized for GPU acceleration.
   */
  struct AggregatedScene {
    /**
     * @brief Trace a ray within the scene. This function is thread-safe.
     * @param ray_descriptor Configuration for the ray.
     * @param closest_hit_func Action to handle the closest hit point.
     * @param miss_func Action to handle the case where the ray escapes the scene.
     * @param any_hit_func Action to handle any hit points along the ray path.
     */
    void Trace(const RayDescriptor& ray_descriptor,
               const std::function<void(const HitInfo& hit_info)>& closest_hit_func,
               const std::function<void()>& miss_func,
               const std::function<void(const HitInfo& hit_info)>& any_hit_func) const;

    /**
     * @brief Trace multiple rays using GPU acceleration.
     * @param rays Input vector of rays to be traced.
     * @param hit_infos Output vector of hit information for each ray.
     * @param flags Configuration flags affecting the tracing behavior.
     */
    void TraceGpu(const std::vector<RayDescriptor>& rays, std::vector<HitInfo>& hit_infos, TraceFlags flags);

    /**
     * @brief Sample the point cloud using GPU acceleration.
     * @param cpu_ray_tracer Reference to the CPU-based ray tracer for sampling points.
     * @param samples Output vector of samples from the point cloud.
     */
    void SamplePointCloudGpu(const CpuRayTracer& cpu_ray_tracer, std::vector<PointCloudSample>& samples);

    /**
     * @brief Initialize the GPU buffers required for scene data.
     */
    void InitializeBuffers();

    /**
     * @struct AggregateSceneInfo
     * @brief Contains metadata and offsets for aggregated scene data.
     */
    struct AggregateSceneInfo {
      uint32_t scene_level_bvh_nodes_size = 0;    ///< Size of scene-level BVH nodes.
      uint32_t scene_level_bvh_nodes_offset = 0;  ///< Offset to scene-level BVH nodes.
      uint32_t node_indices_offset = 0;           ///< Offset to node indices.
      uint32_t node_infos_offset = 0;             ///< Offset to node information.
      uint32_t node_level_bvh_nodes_offset = 0;   ///< Offset to node-level BVH nodes.
      uint32_t mesh_indices_offset = 0;           ///< Offset to mesh indices.

      uint32_t mesh_mappings_offset = 0;         ///< Offset to mesh mappings.
      uint32_t mesh_level_bvh_nodes_offset = 0;  ///< Offset to mesh-level BVH nodes.

      uint32_t triangle_indices_offset = 0;        ///< Offset to triangle indices.
      uint32_t triangles_offset = 0;               ///< Offset to triangles.
      uint32_t vertices_offset = 0;                ///< Offset to vertices.
      uint32_t local_triangle_indices_offset = 0;  ///< Offset to local triangle indices.
    };

    /**
     * @brief Metadata and offsets for the aggregated scene.
     */
    AggregateSceneInfo aggregate_scene_info{};

    /**
     * @brief Data related to the scene graph, stored as a vector of vec4.
     */
    std::vector<glm::vec4> scene_graph_data{};

    /**
     * @brief Data related to scene geometry, stored as a vector of vec4.
     */
    std::vector<glm::vec4> scene_geometry_data{};

    /**
     * @brief GPU buffer for aggregated scene graph data.
     */
    std::shared_ptr<Buffer> aggregate_scene_graph_buffer{};

    /**
     * @brief GPU buffer for aggregated scene geometry data.
     */
    std::shared_ptr<Buffer> aggregate_scene_geometry_buffer{};

    /**
     * @brief GPU buffer for aggregated scene metadata and offsets.
     */
    std::shared_ptr<Buffer> aggregate_scene_info_buffer{};
  };

  /**
   * @brief Aggregate the entire multi-level BVH data structure into a single linear aggregated scene for GPU use.
   * @return Aggregated scene data optimized for GPU acceleration.
   */
  [[nodiscard]] AggregatedScene Aggregate() const;

  /**
   * @brief Retrieve an entity from the scene based on the node index.
   * @param node_index Index of the node associated with the entity.
   * @return The corresponding entity.
   */
  [[nodiscard]] Entity GetEntity(uint32_t node_index) const;

  /**
   * @brief Retrieve the renderer handle associated with a specific node.
   * @param node_index Index of the node.
   * @return The renderer handle for the node.
   */
  [[nodiscard]] Handle GetRendererHandle(uint32_t node_index) const;

 private:
  /**
   * @struct FlattenedBvh
   * @brief Represents a flattened version of the Bounding Volume Hierarchy (BVH) for efficient traversal and
   * storage.
   */
  struct FlattenedBvh {
    /**
     * @brief Flattened list of BVH nodes.
     */
    std::vector<BvhNode> nodes{};
    /**
     * @brief Indices of elements (e.g., triangles or meshes) associated with the BVH.
     */
    std::vector<uint32_t> element_indices{};
  };

  /**
   * @struct GeometryInstance
   * @brief Represents an instance of geometry with its BVH and associated data.
   */
  struct GeometryInstance {
    /**
     * @brief Axis-Aligned Bounding Box (AABB) of the geometry.
     */
    Bound aabb{};
    /**
     * @brief Flattened BVH of the associated triangles of the geometry.
     */
    FlattenedBvh flattened_bvh_triangle_group{};
    /**
     * @brief List of vertices in the geometry.
     */
    std::vector<Vertex> vertices{};
    /**
     * @brief List of triangles in the geometry, represented by indices of vertices.
     */
    std::vector<glm::uvec3> triangles{};

    /**
     * @brief Initialize the geometry instance using the given mesh.
     * @param input_mesh A shared pointer to the input mesh.
     */
    void Initialize(const std::shared_ptr<Mesh>& input_mesh);

    /**
     * @brief Clear all data stored in the geometry instance.
     */
    void Clear() noexcept;
  };

  /**
   * @struct NodeInstance
   * @brief Represents an instance of a scene node, containing its data and relationships.
   */
  struct NodeInstance {
    /**
     * @brief Axis-Aligned Bounding Box (AABB) of the node.
     */
    Bound aabb{};
    /**
     * @brief Index of the instance in the scene.
     */
    uint32_t instance_index = 0;
    /**
     * @brief The entity associated with the node.
     */
    Entity entity{};
    /**
     * @brief The renderer handle associated with the node.
     */
    Handle renderer_handle = 0;
    /**
     * @brief Transformation matrix for the node.
     */
    GlobalTransform transformation{};
    /**
     * @brief Inverse transformation matrix for the node.
     */
    GlobalTransform inverse_transformation{};
    /**
     * @brief Flattened BVH of the meshes associated with the node.
     */
    FlattenedBvh flattened_bvh_mesh_group{};

    /**
     * @brief Initialize the node instance using render instances and meshes.
     * @param render_instances A shared pointer to render instance storage.
     * @param render_instance A shared pointer to the specific mesh render instance.
     * @param mesh_instances A vector of geometry instances representing the meshes.
     * @param mesh_instances_map A mapping from renderer handles to mesh instance indices.
     */
    void Initialize(const std::shared_ptr<RenderInstanceStorage>& render_instances,
                    const std::shared_ptr<RenderInstanceStorage::MeshRenderInstance>& render_instance,
                    const std::vector<GeometryInstance>& mesh_instances,
                    const std::map<Handle, uint32_t>& mesh_instances_map);

    /**
     * @brief Clear all data stored in the node instance.
     */
    void Clear() noexcept;
  };

  /**
   * @enum Axis
   * @brief Represents the three axes X, Y, and Z, used for BVH splitting.
   */
  enum class Axis { X, Y, Z };

  /**
   * @struct SplitResult
   * @brief Contains the result of a BVH splitting operation.
   */
  struct SplitResult {
    /**
     * @brief Axis along which the splitting occurred.
     */
    Axis axis{};
    /**
     * @brief Position of the split along the selected axis.
     */
    float split{};
  };

  /**
   * @struct BucketSplit
   * @brief Represents the result of splitting geometry into buckets.
   */
  struct BucketSplit {
    /**
     * @brief Index of the bucket where the split occurs.
     */
    size_t split_idx{};
    /**
     * @brief Cost of the split, used to minimize intersection tests.
     */
    float cost{};
  };

  /**
   * @struct BucketBound
   * @brief Represents the bounding volume of a bucket used in BVH splitting.
   */
  struct BucketBound {
    /**
     * @brief Minimum corner of the bounding box.
     */
    glm::vec3 min_v{FLT_MAX, FLT_MAX, FLT_MAX};
    /**
     * @brief Maximum corner of the bounding box.
     */
    glm::vec3 max_v{-FLT_MAX, -FLT_MAX, -FLT_MAX};

    /**
     * @brief Add a point to the bounding volume, expanding it as necessary.
     * @param p The point to add.
     */
    void AddPoint(const glm::vec3& p);

    /**
     * @brief Combine this bounding volume with another, creating a more encompassing bounding box.
     * @param other The other bounding volume to combine with.
     */
    void Combine(const BucketBound& other);

    /**
     * @brief Compute the cost of this bounding volume, based on its surface area and number of triangles.
     * @param triangle_count The number of triangles inside this bounding box.
     * @return The computed cost.
     */
    [[nodiscard]] float ComputeCost(int triangle_count) const;
  };

  /**
   * @brief Select the best split for BVH construction from precomputed buckets.
   * @param buckets Array containing triangle counts for each bucket.
   * @param buckets_aabb Array containing bounding volumes for each bucket.
   * @param triangle_count Total number of triangles being considered for splitting.
   * @return The best split found, including the bucket index and cost.
   */
  static BucketSplit SelectSplitFromBuckets(const uint32_t buckets[16], const BucketBound buckets_aabb[16],
                                            size_t triangle_count);

  /**
   * @struct Bvh
   * @brief Represents a hierarchical bounding volume structure for efficient ray tracing and geometry queries.
   */
  struct Bvh {
    /**
     * @brief Axis-Aligned Bounding Box (AABB) of the BVH node.
     */
    Bound aabb{};
    /**
     * @brief Number of elements (e.g., triangles) within this subtree.
     */
    uint32_t subtree_element_size = 0;
    /**
     * @brief Child BVH nodes.
     */
    std::vector<Bvh> children{};
    /**
     * @brief Indices of elements (e.g., triangles or meshes) within this BVH node.
     */
    std::vector<uint32_t> element_indices{};
  };

  /**
   * @brief Find the best split for a given parent BVH node during BVH construction.
   * @param parent The parent BVH node.
   * @param aabbs A list of bounding boxes corresponding to elements (e.g., triangles) within the parent node.
   * @return The best split result, containing the axis and split location.
   */
  static SplitResult FindBestSplit(const Bvh& parent, const std::vector<Bound>& aabbs);

  /**
   * @brief Perform binary division to construct a BVH from a given parent node.
   * @param parent The parent BVH node to divide.
   * @param current_tree_depth The current depth of the tree.
   * @param aabbs A list of bounding boxes corresponding to elements (e.g., triangles) being divided.
   */
  static void BinaryDivisionBvh(Bvh& parent, uint32_t current_tree_depth, const std::vector<Bound>& aabbs);

  /**
   * @brief Flatten a hierarchical BVH structure into a compact representation for efficient traversal.
   * @param current_bvh The current BVH to flatten.
   * @param flattened_bvh The output flattened BVH structure.
   * @param level The current depth level of the BVH being flattened.
   */
  static void FlattenBvh(const Bvh& current_bvh, FlattenedBvh& flattened_bvh, uint32_t level);

  /**
   * @brief Calculate barycentric coordinates for a point within a triangle.
   * @param p The point to calculate coordinates for.
   * @param a The first vertex of the triangle.
   * @param b The second vertex of the triangle.
   * @param c The third vertex of the triangle.
   * @return The barycentric coordinates of point p relative to the triangle (a, b, c).
   */
  static glm::vec3 Barycentric(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);

  /**
   * @brief Axis-Aligned Bounding Box (AABB) of the entire scene.
   */
  Bound aabb_{};

  /**
   * @brief Flattened BVH structure for scene nodes.
   */
  FlattenedBvh flattened_bvh_node_group_{};

  /**
   * @brief List of geometry instances comprising the scene.
   */
  std::vector<GeometryInstance> geometry_instances_{};

  /**
   * @brief List of node instances comprising the scene.
   */
  std::vector<NodeInstance> node_instances_{};
};

}  // namespace evo_engine
