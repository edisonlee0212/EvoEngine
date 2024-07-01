#pragma once

namespace mesh_tracing {

class MeshTracer {
 public:
  struct Aabb {
    glm::vec3 center{};
    glm::vec3 size{};
  };
  struct FlattenedBvhNode {
    glm::vec3 aabb_min{};
    glm::vec3 aabb_max{};
    uint32_t f_bvh_vertices_start_index = 0;
    uint32_t f_bvh_vertices_end_index = 0;
    uint32_t next_f_bvh_node_index = 0;
  };
  struct Bvh {
    Aabb aabb{};
    uint32_t subtree_triangle_size = 0;
    std::vector<Bvh> children;
    struct TriangleInfo {
      uint32_t triangle_index;
      uint32_t instance_index;
    };
    std::vector<TriangleInfo> triangle_info_list;
    void Initialize(const std::vector<glm::vec3>& input_vertices, const std::vector<glm::uvec3>& input_triangles,
                    uint32_t max_triangle_per_leaf, uint32_t max_tree_depth);

    void Clear();
    void CollectAabb(uint32_t min_tree_depth, uint32_t max_tree_depth, std::vector<Aabb>& aabbs) const;
  };
  Bvh bvh{};
  Aabb aabb{};

  std::vector<FlattenedBvhNode> f_bvh_nodes;
  std::vector<uint32_t> f_bvh_triangle_index;
  std::vector<uint32_t> f_bvh_instance_index;
  std::vector<glm::vec3> f_bvh_vertices;

  /**
   * @brief Builds a BVH (bounding volume hierarchy) for a mesh.
   * @param input_vertices Vertices of the mesh.
   * @param input_triangles Triangles of the mesh.
   * @param max_triangle_per_leaf Maximum number of triangles in a leaf node.
   * @param max_tree_depth Maximum depth of the tree (useful for stack based algorithms)
   */
  void Initialize(const std::vector<glm::vec3>& input_vertices, const std::vector<glm::uvec3>& input_triangles,
                  uint32_t max_triangle_per_leaf, uint32_t max_tree_depth);

  void Clear();
  static glm::vec3 Barycentric(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);
  enum class TraceFlags {
    // No special flag set.
    None = 0,
    // Enforce any hit programs for all geometries. This flag is mutually exclusive with DisableAnyHit.
    EnforceAnyHit = 1 << 1,
  };
  struct TraceParameters {
    glm::vec3 origin = glm::vec3(0.f);
    glm::vec3 direction = glm::vec3(0.f);
    float t_min = 0.f;
    float t_max = FLT_MAX;
    TraceFlags flags = TraceFlags::None;
  };

  struct HitInfo {
    uint32_t vertex_index = 0;
    uint32_t triangle_index = 0;
    uint32_t instance_index = 0;
    glm::vec3 hit = glm::vec3(0.f);
    glm::vec2 barycentric = glm::vec2(0.f);
    bool back_face = false;
  };

  void Trace(const TraceParameters& trace_parameters, const std::function<void(HitInfo hit_info)>& closest_hit_func,
             const std::function<void()>& miss_func, const std::function<void(HitInfo hit_info)>& any_hit_func) const;
};
}  // namespace mesh_tracing