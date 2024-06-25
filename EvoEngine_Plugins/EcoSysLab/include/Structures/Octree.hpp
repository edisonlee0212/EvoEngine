#pragma once

#include "MarchingCubes.hpp"
#include "glm/gtx/quaternion.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
typedef int OctreeNodeHandle;
typedef int OctreeOctreeNodeDataHandle;
class OctreeNode {
  float radius_ = 0.0f;
  unsigned level_ = 0;
  glm::vec3 center_ = glm::vec3(0.0f);
  OctreeNodeHandle children_[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
  OctreeOctreeNodeDataHandle data_handle_ = -1;
  template <typename Nd>
  friend class Octree;
  bool recycled_ = true;

 public:
  [[nodiscard]] float GetRadius() const {
    return radius_;
  }
  [[nodiscard]] unsigned GetLevel() const {
    return level_;
  }
  [[nodiscard]] glm::vec3 GetCenter() const {
    return center_;
  }
  [[nodiscard]] OctreeOctreeNodeDataHandle GetOctreeNodeDataHandle() const {
    return data_handle_;
  }
  [[nodiscard]] bool IsRecycled() const {
    return recycled_;
  }
  /*
  int m_leftUpBack = -1;
  int m_leftUpFront = -1;
  int m_leftDownBack = -1;
  int m_leftDownFront = -1;
  int m_rightUpBack = -1;
  int m_rightUpFront = -1;
  int m_rightDownBack = -1;
  int m_rightDownFront = -1;
  */
};

template <typename OctreeNodeData>
class Octree {
  std::vector<OctreeNode> octree_nodes_ = {};
  std::queue<size_t> node_pool_ = {};
  std::vector<OctreeNodeData> node_data_ = {};
  std::queue<size_t> node_data_pool_ = {};
  OctreeNodeHandle Allocate(float radius, unsigned level, const glm::vec3& center);
  void Recycle(OctreeNodeHandle node_handle);
  float chunk_radius_ = 16;
  unsigned max_subdivision_level_ = 10;
  float minimum_node_radius_ = 0.015625f;
  glm::vec3 center_;

 public:
  Octree();
  [[nodiscard]] float GetMinRadius() const;
  Octree(float radius, unsigned max_subdivision_level, const glm::vec3& center);
  void IterateLeaves(const std::function<void(const OctreeNode& octree_node)>& func) const;
  [[nodiscard]] bool Occupied(const glm::vec3& position) const;
  void Reset(float radius, unsigned max_subdivision_level, const glm::vec3& center);
  [[nodiscard]] OctreeNodeHandle GetNodeHandle(const glm::vec3& position) const;
  [[nodiscard]] const OctreeNode& RefNode(OctreeNodeHandle node_handle) const;

  // void Expand(OctreeNodeHandle nodeHandle);
  // void Collapse(OctreeNodeHandle nodeHandle);

  void Occupy(const glm::vec3& position, const std::function<void(OctreeNode&)>& occupied_nodes);
  void Occupy(const glm::vec3& position, const glm::quat& rotation, float length, float radius,
              const std::function<void(OctreeNode&)>& occupied_nodes);
  void Occupy(const glm::vec3& min, const glm::vec3& max,
              const std::function<bool(const glm::vec3& box_center)>& collision_handle,
              const std::function<void(OctreeNode&)>& occupied_nodes);
  [[nodiscard]] OctreeNodeData& RefOctreeNodeData(OctreeOctreeNodeDataHandle node_data_handle);
  [[nodiscard]] const OctreeNodeData& PeekOctreeNodeData(OctreeOctreeNodeDataHandle node_data_handle) const;
  [[nodiscard]] OctreeNodeData& RefOctreeNodeData(const OctreeNode& octree_node);
  [[nodiscard]] const OctreeNodeData& PeekOctreeNodeData(const OctreeNode& octree_node) const;
  void GetVoxels(std::vector<glm::mat4>& voxels) const;
  void TriangulateField(std::vector<Vertex>& vertices, std::vector<unsigned>& indices, bool remove_duplicate) const;
};

template <typename OctreeNodeData>
OctreeNodeHandle Octree<OctreeNodeData>::Allocate(const float radius, const unsigned level, const glm::vec3& center) {
  OctreeNodeHandle new_node_handle;
  if (node_pool_.empty()) {
    new_node_handle = octree_nodes_.size();
    octree_nodes_.emplace_back();
  } else {
    new_node_handle = node_pool_.front();
    node_pool_.pop();
  }

  auto& node = octree_nodes_.at(new_node_handle);
  node.radius_ = radius;
  node.level_ = level;
  node.center_ = center;
  node.recycled_ = false;
  if (node_data_pool_.empty()) {
    node.data_handle_ = node_data_.size();
    node_data_.emplace_back();
  } else {
    node.data_handle_ = node_data_pool_.front();
    node_data_pool_.pop();
  }
  node_data_.at(node.data_handle_) = {};
  return octree_nodes_.size() - 1;
}

template <typename OctreeNodeData>
void Octree<OctreeNodeData>::Recycle(const OctreeNodeHandle node_handle) {
  node_data_pool_.push(node_handle);
  auto& node = octree_nodes_[node_handle];
  node.radius_ = 0;
  node.level_ = 0;
  node.center_ = {};
  node.recycled_ = true;

  node_data_pool_.push(node.data_handle_);
  node.data_handle_ = -1;
}

template <typename OctreeNodeData>
Octree<OctreeNodeData>::Octree() {
  Reset(16, 10, glm::vec3(0.0f));
}
template <typename OctreeNodeData>
float Octree<OctreeNodeData>::GetMinRadius() const {
  return minimum_node_radius_;
}
template <typename OctreeNodeData>
Octree<OctreeNodeData>::Octree(const float radius, const unsigned max_subdivision_level, const glm::vec3& center) {
  Reset(radius, max_subdivision_level, center);
}

template <typename OctreeNodeData>
bool Octree<OctreeNodeData>::Occupied(const glm::vec3& position) const {
  float current_radius = chunk_radius_;
  glm::vec3 center = center_;
  int octree_node_index = 0;
  for (int subdivision = 0; subdivision < max_subdivision_level_; subdivision++) {
    current_radius /= 2.f;
    const auto& octree_node = octree_nodes_[octree_node_index];
    const int index =
        4 * (position.x > center.x ? 0 : 1) + 2 * (position.y > center.y ? 0 : 1) + (position.z > center.z ? 0 : 1);
    if (octree_node.children_[index] == -1) {
      return false;
    }
    octree_node_index = octree_node.children_[index];
    center.x += position.x > center.x ? current_radius : -current_radius;
    center.y += position.y > center.y ? current_radius : -current_radius;
    center.z += position.z > center.z ? current_radius : -current_radius;
  }
  return true;
}
template <typename OctreeNodeData>
void Octree<OctreeNodeData>::Reset(float radius, unsigned max_subdivision_level, const glm::vec3& center) {
  chunk_radius_ = minimum_node_radius_ = radius;
  max_subdivision_level_ = max_subdivision_level;
  center_ = center;
  octree_nodes_.clear();
  for (int subdivision = 0; subdivision < max_subdivision_level_; subdivision++) {
    minimum_node_radius_ /= 2.f;
  }
  Allocate(chunk_radius_, -1, center);
}
template <typename OctreeNodeData>
OctreeNodeHandle Octree<OctreeNodeData>::GetNodeHandle(const glm::vec3& position) const {
  float current_radius = chunk_radius_;
  glm::vec3 center = center_;
  OctreeNodeHandle octree_node_index = 0;
  for (int subdivision = 0; subdivision < max_subdivision_level_; subdivision++) {
    current_radius /= 2.f;
    const auto& octree_node = octree_nodes_[octree_node_index];
    const int index =
        4 * (position.x > center.x ? 0 : 1) + 2 * (position.y > center.y ? 0 : 1) + (position.z > center.z ? 0 : 1);
    if (octree_node.children_[index] == -1) {
      return -1;
    }
    octree_node_index = octree_node.children_[index];
    center.x += position.x > center.x ? current_radius : -current_radius;
    center.y += position.y > center.y ? current_radius : -current_radius;
    center.z += position.z > center.z ? current_radius : -current_radius;
  }
  return octree_node_index;
}
template <typename OctreeNodeData>
const OctreeNode& Octree<OctreeNodeData>::RefNode(const OctreeNodeHandle node_handle) const {
  return octree_nodes_[node_handle];
}

template <typename OctreeNodeData>
void Octree<OctreeNodeData>::Occupy(const glm::vec3& position, const std::function<void(OctreeNode&)>& occupied_nodes) {
  float current_radius = chunk_radius_;
  glm::vec3 center = center_;
  int octree_node_index = 0;
  for (int subdivision = 0; subdivision < max_subdivision_level_; subdivision++) {
    current_radius /= 2.f;
    const auto& octree_node = octree_nodes_[octree_node_index];
    const int index =
        4 * (position.x > center.x ? 0 : 1) + 2 * (position.y > center.y ? 0 : 1) + (position.z > center.z ? 0 : 1);
    center.x += position.x > center.x ? current_radius : -current_radius;
    center.y += position.y > center.y ? current_radius : -current_radius;
    center.z += position.z > center.z ? current_radius : -current_radius;
    if (octree_node.children_[index] == -1) {
      const auto new_index = Allocate(current_radius, subdivision, center);
      octree_nodes_[octree_node_index].children_[index] = new_index;
      octree_node_index = new_index;
    } else
      octree_node_index = octree_node.children_[index];
  }
  occupied_nodes(octree_nodes_[octree_node_index]);
}

template <typename OctreeNodeData>
void Octree<OctreeNodeData>::Occupy(const glm::vec3& position, const glm::quat& rotation, float length, float radius,
                                    const std::function<void(OctreeNode&)>& occupied_nodes) {
  const float max_radius = glm::max(length, radius);
  Occupy(
      glm::vec3(position - glm::vec3(max_radius)), glm::vec3(position + glm::vec3(max_radius)),
      [&](const glm::vec3& box_center) {
        const auto relative_pos = glm::rotate(glm::inverse(rotation), box_center - position);
        return glm::abs(relative_pos.z) <= length && glm::length(glm::vec2(relative_pos.x, relative_pos.y)) <= radius;
      },
      occupied_nodes);
}

template <typename OctreeNodeData>
void Octree<OctreeNodeData>::Occupy(const glm::vec3& min, const glm::vec3& max,
                                    const std::function<bool(const glm::vec3& box_center)>& collision_handle,
                                    const std::function<void(OctreeNode&)>& occupied_nodes) {
  for (float x = min.x - minimum_node_radius_; x < max.x + minimum_node_radius_; x += minimum_node_radius_) {
    for (float y = min.y - minimum_node_radius_; y < max.y + minimum_node_radius_; y += minimum_node_radius_) {
      for (float z = min.z - minimum_node_radius_; z < max.z + minimum_node_radius_; z += minimum_node_radius_) {
        if (collision_handle(glm::vec3(x, y, z))) {
          Occupy(glm::vec3(x, y, z), occupied_nodes);
        }
      }
    }
  }
}

template <typename OctreeNodeData>
OctreeNodeData& Octree<OctreeNodeData>::RefOctreeNodeData(int node_data_handle) {
  assert(node_data_handle > 0 && node_data_handle < node_data_.size());
  return node_data_[node_data_handle];
}

template <typename OctreeNodeData>
const OctreeNodeData& Octree<OctreeNodeData>::PeekOctreeNodeData(int node_data_handle) const {
  assert(node_data_handle > 0 && node_data_handle < node_data_.size());
  return node_data_[node_data_handle];
}

template <typename OctreeNodeData>
OctreeNodeData& Octree<OctreeNodeData>::RefOctreeNodeData(const OctreeNode& octree_node) {
  assert(!octree_node.recycled_);
  return node_data_[octree_node.data_handle_];
}

template <typename OctreeNodeData>
const OctreeNodeData& Octree<OctreeNodeData>::PeekOctreeNodeData(const OctreeNode& octree_node) const {
  assert(!octree_node.recycled_);
  return node_data_[octree_node.data_handle_];
}

template <typename OctreeNodeData>
void Octree<OctreeNodeData>::IterateLeaves(const std::function<void(const OctreeNode& octree_node)>& func) const {
  for (const auto& node : octree_nodes_) {
    if (node.level_ == max_subdivision_level_ - 1) {
      func(node);
    }
  }
}
template <typename OctreeNodeData>
void Octree<OctreeNodeData>::GetVoxels(std::vector<glm::mat4>& voxels) const {
  voxels.clear();
  IterateLeaves([&](const OctreeNode& octree_node) {
    voxels.push_back(glm::translate(octree_node.center_) * glm::scale(glm::vec3(minimum_node_radius_)));
  });
}
template <typename OctreeNodeData>
void Octree<OctreeNodeData>::TriangulateField(std::vector<Vertex>& vertices, std::vector<unsigned>& indices,
                                              const bool remove_duplicate) const {
  std::vector<TestingCell> testing_cells;
  IterateLeaves([&](const OctreeNode& octree_node) {
    TestingCell testing_cell;
    testing_cell.m_position = octree_node.center_;
    testing_cells.push_back(testing_cell);
  });

  MarchingCubes::TriangulateField(
      center_,
      [&](const glm::vec3& sample_point) {
        return Occupied(sample_point) ? 1.0f : 0.0f;
      },
      0.5f, minimum_node_radius_, testing_cells, vertices, indices, remove_duplicate);
}

}  // namespace eco_sys_lab