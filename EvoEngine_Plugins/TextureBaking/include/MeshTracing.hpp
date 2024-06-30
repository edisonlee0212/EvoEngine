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
    uint32_t start = 0;
    uint32_t end = 0;
    uint32_t jump = 0;
  };
  struct Bvh {
    Aabb aabb{};
    uint32_t subtree_triangle_size = 0;
    std::vector<Bvh> children;
    std::vector<uint32_t> triangle_indices;
    void Initialize(const std::vector<glm::vec3>& input_vertices, const std::vector<glm::uvec3>& input_triangles,
                    uint32_t max_triangle_per_leaf, uint32_t max_tree_depth);

    void Clear();
    void CollectAabb(uint32_t min_tree_depth, uint32_t max_tree_depth, std::vector<Aabb>& aabbs) const;
  };
  Bvh bvh{};
  Aabb aabb{};

  std::vector<FlattenedBvhNode> flattened_bvh_nodes;
  std::vector<uint32_t> flattened_bvh_original_face_id;
  std::vector<glm::vec3> flattened_bvh_positions;

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
};
}  // namespace mesh_tracing