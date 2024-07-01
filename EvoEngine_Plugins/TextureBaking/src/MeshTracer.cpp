#include "MeshTracer.hpp"

using namespace mesh_tracing;
using namespace evo_engine;
class Timer {
 public:
  Timer() {
    Begin();
  }
  void Begin() {
    begin_ = std::chrono::high_resolution_clock::now();
  }

  [[nodiscard]] float ElapsedSeconds() const {
    return static_cast<float>(
               std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - begin_)
                   .count()) /
           1000.0f;
  }

 private:
  std::chrono::high_resolution_clock::time_point begin_;
};
#pragma region Bvh
void MeshTracer::Bvh::Initialize(const std::vector<glm::vec3>& input_vertices,
                                 const std::vector<glm::uvec3>& input_triangles, const uint32_t max_triangle_per_leaf,
                                 const uint32_t max_tree_depth) {
  const Timer timer{};
  glm::vec3 min_bound{FLT_MAX};
  glm::vec3 max_bound{-FLT_MAX};
  for (const auto& vertex : input_vertices) {
    min_bound = glm::min(vertex, min_bound);
    max_bound = glm::max(vertex, max_bound);
  }
  aabb.center = (max_bound + min_bound) * .5f;
  aabb.size = (max_bound - min_bound) * .5f;
  const auto count = input_triangles.size();
  triangle_info_list.resize(count);
  Jobs::RunParallelFor(count, [&](const size_t i) {
    triangle_info_list.at(i) = {static_cast<uint32_t>(i), 0};
  });
  enum class Axis { X, Y, Z };
  struct SplitResult {
    Axis axis;
    float split;
  };
  struct BucketSplit {
    size_t split_idx;
    float cost;
  };
  struct BucketAabb {
    glm::vec3 min_v = glm::vec3(FLT_MAX);
    glm::vec3 max_v = glm::vec3(-FLT_MAX);
    void AddPoint(const glm::vec3& p) {
      min_v = glm::min(p, min_v);
      max_v = glm::max(p, max_v);
    }
    void Combine(const BucketAabb& other) {
      min_v = glm::min(min_v, other.min_v);
      max_v = glm::max(max_v, other.max_v);
    }
    [[nodiscard]] float ComputeCost(const int triangle_count) const {
      const auto size = max_v - min_v;
      const float surface_area = 2.0f * (size.x * size.y + size.x * size.z + size.x * size.y);
      return static_cast<float>(triangle_count) * surface_area;
    }
  };
  auto select_split_from_buckets = [&](const uint32_t buckets[16], const BucketAabb buckets_aabb[16],
                                       const size_t triangle_count) {
    // Pass to compute AABB on the right side of the split
    BucketAabb aabb_right_side[15];
    aabb_right_side[14] = buckets_aabb[15];
    for (int i = 13; i >= 0; --i) {
      aabb_right_side[i] = aabb_right_side[i + 1];
      aabb_right_side[i].Combine(buckets_aabb[i + 1]);
    }

    float best_cost = FLT_MAX;
    int best_split_idx = 0;

    BucketAabb aabb_left;
    int count_left = 0;
    int count_right = static_cast<int>(triangle_count);
    for (int i = 0; i < 15; ++i) {
      const auto c = static_cast<int>(buckets[i]);
      count_left += c;
      count_right -= c;
      aabb_left.Combine(buckets_aabb[i]);
      BucketAabb aabb_right = aabb_right_side[i];
      const float cost_left = aabb_left.ComputeCost(count_left);
      const float cost_right = aabb_right.ComputeCost(count_right);
      if (const float cost = cost_left + cost_right; cost < best_cost) {
        best_cost = cost;
        best_split_idx = i;
      }
    }

    return BucketSplit{static_cast<size_t>(best_split_idx), best_cost};
  };
  auto find_best_split = [&](const Bvh& parent) {
    SplitResult ret;

    BucketAabb centroids_aabb;
    for (const auto& triangle_info : parent.triangle_info_list) {
      const auto triangle = input_triangles[triangle_info.triangle_index];
      const auto p0 = input_vertices[triangle.x];
      const auto p1 = input_vertices[triangle.y];
      const auto p2 = input_vertices[triangle.z];
      const auto centroid = (p0 + p1 + p2) / 3.0f;
      centroids_aabb.AddPoint(centroid);
    }

    uint32_t buckets_x[16] = {};
    uint32_t buckets_y[16] = {};
    uint32_t buckets_z[16] = {};
    BucketAabb buckets_aabb_x[16];
    BucketAabb buckets_aabb_y[16];
    BucketAabb buckets_aabb_z[16];

    for (const auto& triangle_info : parent.triangle_info_list) {
      const auto triangle = input_triangles[triangle_info.triangle_index];
      const auto p0 = input_vertices[triangle.x];
      const auto p1 = input_vertices[triangle.y];
      const auto p2 = input_vertices[triangle.z];
      const auto centroid = (p0 + p1 + p2) / 3.0f;

      const auto ijk = (centroid - centroids_aabb.min_v) / (centroids_aabb.max_v - centroids_aabb.min_v) * 15.99f;
      const size_t i = std::isnan(ijk.x) ? static_cast<size_t>(0) : static_cast<size_t>(ijk.x);
      const size_t j = std::isnan(ijk.y) ? static_cast<size_t>(0) : static_cast<size_t>(ijk.y);
      const size_t k = std::isnan(ijk.z) ? static_cast<size_t>(0) : static_cast<size_t>(ijk.z);
      assert(i < 16 && j < 16 && k < 16);

      ++buckets_x[i];
      ++buckets_y[j];
      ++buckets_z[k];

      buckets_aabb_x[i].AddPoint(p0);
      buckets_aabb_x[i].AddPoint(p1);
      buckets_aabb_x[i].AddPoint(p2);
      buckets_aabb_y[j].AddPoint(p0);
      buckets_aabb_y[j].AddPoint(p1);
      buckets_aabb_y[j].AddPoint(p2);
      buckets_aabb_z[k].AddPoint(p0);
      buckets_aabb_z[k].AddPoint(p1);
      buckets_aabb_z[k].AddPoint(p2);
    }

    const uint32_t triangle_count = static_cast<uint32_t>(parent.triangle_info_list.size());
    const auto split_x = select_split_from_buckets(buckets_x, buckets_aabb_x, triangle_count);
    const auto split_y = select_split_from_buckets(buckets_y, buckets_aabb_y, triangle_count);
    const auto split_z = select_split_from_buckets(buckets_z, buckets_aabb_z, triangle_count);

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
  };
  std::function<void(Bvh & parent, uint32_t current_tree_depth)> binary_division_bvh;
  binary_division_bvh = [&](Bvh& parent, uint32_t current_tree_depth) {
    if (parent.triangle_info_list.size() <= max_triangle_per_leaf || current_tree_depth >= max_tree_depth) {
      parent.subtree_triangle_size = static_cast<uint32_t>(parent.triangle_info_list.size());
      return;
    }
    parent.children.resize(2);
    glm::vec3 aabb_min_l(FLT_MAX);
    glm::vec3 aabb_max_l(-FLT_MAX);
    glm::vec3 aabb_min_r(FLT_MAX);
    glm::vec3 aabb_max_r(-FLT_MAX);
    const SplitResult split = find_best_split(parent);
    for (const auto& triangle_info : parent.triangle_info_list) {
      const auto& triangle = input_triangles[triangle_info.triangle_index];
      const auto p0 = input_vertices[triangle.x];
      const auto p1 = input_vertices[triangle.y];
      const auto p2 = input_vertices[triangle.z];

      if (const auto c = (p0 + p1 + p2) * (1.0f / 3.0f); (split.axis == Axis::X && c.x <= split.split) ||
                                                         (split.axis == Axis::Y && c.y <= split.split) ||
                                                         (split.axis == Axis::Z && c.z <= split.split)) {
        parent.children[0].triangle_info_list.push_back(triangle_info);
        aabb_min_l = min(aabb_min_l, min(p0, min(p1, p2)));
        aabb_max_l = max(aabb_max_l, max(p0, max(p1, p2)));
      } else {
        parent.children[1].triangle_info_list.push_back(triangle_info);
        aabb_min_r = min(aabb_min_r, min(p0, min(p1, p2)));
        aabb_max_r = max(aabb_max_r, max(p0, max(p1, p2)));
      }
    }

    if (parent.children[0].triangle_info_list.empty() || parent.children[1].triangle_info_list.empty()) {
      // Unable to subdivide... We brake here
      parent.children.clear();
      parent.subtree_triangle_size = static_cast<uint32_t>(parent.triangle_info_list.size());
      return;
    }

    parent.triangle_info_list.clear();

    parent.children[0].aabb = Aabb{(aabb_min_l + aabb_max_l) * 0.5f, (aabb_max_l - aabb_min_l) * 0.5f};
    parent.children[1].aabb = Aabb{(aabb_min_r + aabb_max_r) * 0.5f, (aabb_max_r - aabb_min_r) * 0.5f};

    binary_division_bvh(parent.children[0], current_tree_depth + 1);
    binary_division_bvh(parent.children[1], current_tree_depth + 1);

    // Update AABB

    aabb_min_l = parent.children[0].aabb.center - parent.children[0].aabb.size;
    aabb_max_l = parent.children[0].aabb.center + parent.children[0].aabb.size;
    aabb_min_r = parent.children[1].aabb.center - parent.children[1].aabb.size;
    aabb_max_r = parent.children[1].aabb.center + parent.children[1].aabb.size;
    const auto aabb_min = glm::min(aabb_min_l, aabb_min_r);
    const auto aabb_max = glm::max(aabb_max_l, aabb_max_r);
    parent.aabb.center = (aabb_max + aabb_min) * 0.5f;
    parent.aabb.size = (aabb_max - aabb_min) * 0.5f;

    parent.subtree_triangle_size = parent.children[0].subtree_triangle_size + parent.children[1].subtree_triangle_size;
  };

  binary_division_bvh(*this, 0);

  const auto elapsed_time = timer.ElapsedSeconds();

  EVOENGINE_LOG(std::string("Bvh initialization finished in ") + std::to_string(elapsed_time) + " seconds")
}

void MeshTracer::Bvh::Clear() {
  aabb = {};
  subtree_triangle_size = 0;
  children.clear();
  triangle_info_list.clear();
}

void MeshTracer::Bvh::CollectAabb(const uint32_t min_tree_depth, const uint32_t max_tree_depth,
                                  std::vector<Aabb>& aabbs) const {
  std::function<void(const Bvh& current_bvh, uint32_t current_depth)> collect_aabb;
  collect_aabb = [&](const Bvh& current_bvh, const uint32_t current_depth) {
    if (current_depth < max_tree_depth) {
      if (current_depth >= min_tree_depth)
        aabbs.emplace_back(current_bvh.aabb);
      for (const auto& child : current_bvh.children) {
        collect_aabb(child, current_depth + 1);
      }
    }
  };
  collect_aabb(*this, 0);
}
#pragma endregion

void MeshTracer::Initialize(const std::vector<glm::vec3>& input_vertices,
                            const std::vector<glm::uvec3>& input_triangles, const uint32_t max_triangle_per_leaf,
                            const uint32_t max_tree_depth) {
  Clear();
  bvh.Initialize(input_vertices, input_triangles, max_triangle_per_leaf, max_tree_depth);
  aabb = bvh.aabb;
  std::function<void(const Bvh& current_bvh)> construct_flatten_bvh_with_mesh;
  construct_flatten_bvh_with_mesh = [&](const Bvh& current_bvh) {
    if (current_bvh.children.empty() && current_bvh.triangle_info_list.empty()) {
      // Children node without triangles? Skip it
      return;
    }
    if (!current_bvh.children.empty()) {
      // If one of the children does not contain any triangles
      // we can skip this node completely as it is an extra AABB test
      // for nothing
      if (current_bvh.children[0].subtree_triangle_size > 0 && current_bvh.children[1].subtree_triangle_size == 0) {
        construct_flatten_bvh_with_mesh(current_bvh.children[0]);
        return;
      }
      if (current_bvh.children[1].subtree_triangle_size > 0 && current_bvh.children[0].subtree_triangle_size == 0) {
        construct_flatten_bvh_with_mesh(current_bvh.children[1]);
        return;
      }
    }
    f_bvh_nodes.emplace_back();
    FlattenedBvhNode& d = f_bvh_nodes.back();
    d.aabb_min = current_bvh.aabb.center - current_bvh.aabb.size;
    d.aabb_max = current_bvh.aabb.center + current_bvh.aabb.size;
    d.f_bvh_vertices_start_index = static_cast<uint32_t>(f_bvh_vertices.size());
    for (const auto& triangle_info : current_bvh.triangle_info_list) {
      const auto& triangle = input_triangles.at(triangle_info.triangle_index);
      const auto& p0 = input_vertices.at(triangle.x);
      const auto& p1 = input_vertices.at(triangle.y);
      const auto& p2 = input_vertices.at(triangle.z);
      f_bvh_triangle_index.emplace_back(triangle_info.triangle_index);
      f_bvh_instance_index.emplace_back(triangle_info.instance_index);
      f_bvh_vertices.emplace_back(p0);
      f_bvh_vertices.emplace_back(p1);
      f_bvh_vertices.emplace_back(p2);
    }
    d.f_bvh_vertices_end_index = static_cast<uint32_t>(f_bvh_vertices.size());
    const size_t current_node_index = f_bvh_nodes.size() - 1;
    if (!current_bvh.children.empty()) {
      construct_flatten_bvh_with_mesh(current_bvh.children[0]);
      construct_flatten_bvh_with_mesh(current_bvh.children[1]);
    }
    f_bvh_nodes[current_node_index].next_f_bvh_node_index = static_cast<uint32_t>(f_bvh_nodes.size());
  };
  construct_flatten_bvh_with_mesh(bvh);
}

void MeshTracer::Clear() {
  f_bvh_nodes.clear();
  f_bvh_triangle_index.clear();
  f_bvh_instance_index.clear();
  f_bvh_vertices.clear();
  aabb = {};
  bvh.Clear();
}

glm::vec3 MeshTracer::Barycentric(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
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

void MeshTracer::Trace(const TraceParameters& trace_parameters, const std::function<void(HitInfo)>& closest_hit_func,
                       const std::function<void()>& miss_func, const std::function<void(HitInfo)>& any_hit_func) const {
  uint32_t current_f_bvh_index = 0;
  float current_distance = trace_parameters.t_max;
  const auto ray_direction = glm::normalize(trace_parameters.direction);
  HitInfo closest_hit_info{};
  bool has_hit = false;
  unsigned flags = static_cast<unsigned>(trace_parameters.flags);
  const bool enforce_any_hit = flags & static_cast<unsigned>(TraceFlags::EnforceAnyHit);
  while (current_f_bvh_index < f_bvh_nodes.size()) {
    const auto& f_bvh_node = f_bvh_nodes.at(current_f_bvh_index);
#pragma region RayAABB
    glm::vec3 t1 = (f_bvh_node.aabb_min - trace_parameters.origin) / ray_direction;
    glm::vec3 t2 = (f_bvh_node.aabb_max - trace_parameters.origin) / ray_direction;
    const glm::vec3 current_t_min = glm::min(t1, t2);
    const glm::vec3 current_t_max = glm::max(t1, t2);
    const float min_d = glm::max(current_t_min.x, glm::max(current_t_min.y, current_t_min.z));
    const float max_d = glm::min(current_t_max.x, glm::min(current_t_max.y, current_t_max.z));
#pragma endregion
    if (const float ray_aabb_distance = max_d >= 0 && min_d <= max_d ? min_d : FLT_MAX;
        ray_aabb_distance <= (enforce_any_hit ? trace_parameters.t_max : current_distance)) {
#pragma region Test all triangles
      for (uint32_t test_vertex_index = f_bvh_node.f_bvh_vertices_start_index;
           test_vertex_index < f_bvh_node.f_bvh_vertices_end_index; test_vertex_index += 3) {
        const auto& a = f_bvh_vertices.at(test_vertex_index);
        const auto& b = f_bvh_vertices.at(test_vertex_index + 1);
        const auto& c = f_bvh_vertices.at(test_vertex_index + 2);
        const auto test_triangle_normal = glm::normalize(glm::cross(b - a, c - a));
        const auto normal_test = glm::dot(ray_direction, test_triangle_normal);
        // Skip this triangle if it's parallel with the ray.
        if (normal_test == 0.f)
          continue;
        const auto hit_distance =
            (glm::dot(a, test_triangle_normal) - glm::dot(trace_parameters.origin, test_triangle_normal)) / normal_test;
        if (hit_distance >= trace_parameters.t_min &&
            hit_distance < (enforce_any_hit ? trace_parameters.t_max : current_distance)) {
          const auto hit = trace_parameters.origin + ray_direction * hit_distance;
          if (const auto barycentric = Barycentric(hit, a, b, c); barycentric.x >= 0.f && barycentric.x <= 1.f &&
                                                                  barycentric.y >= 0.f && barycentric.y <= 1.f &&
                                                                  barycentric.z >= 0.f && barycentric.z <= 1.f) {
            has_hit = true;
            HitInfo current_hit_info{};
            current_hit_info.vertex_index = test_vertex_index;
            current_hit_info.triangle_index = f_bvh_triangle_index.at(test_vertex_index / 3);
            current_hit_info.instance_index = f_bvh_instance_index.at(test_vertex_index / 3);
            current_hit_info.hit = hit;
            current_hit_info.barycentric = barycentric;
            current_hit_info.back_face = normal_test > 0.f;
            any_hit_func(current_hit_info);
            if (!enforce_any_hit || hit_distance < current_distance) {
              closest_hit_info = current_hit_info;
            }
          }
        }
      }
#pragma endregion
      current_f_bvh_index++;
    } else {
      current_f_bvh_index = f_bvh_node.next_f_bvh_node_index;
    }
  }
  if (has_hit) {
    closest_hit_func(closest_hit_info);
  } else {
    miss_func();
  }
}