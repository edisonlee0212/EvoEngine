#include <gtest/gtest.h>

#include <cmath>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "BasicFoliageDescriptor.hpp"
#include "GpuTreeMeshData.hpp"
#include "Jobs.hpp"
#include "TreeMeshGenerator.hpp"
#include "TreeSkinnedMeshGenerator.hpp"

using namespace eco_sys_lab_package;

namespace {
using MeshGenerator = CylindricalMeshGenerator<BaseSkeletonData, BaseFlowData, BaseNodeData>;
using SkinnedMeshGenerator = CylindricalSkinnedMeshGenerator<BaseSkeletonData, BaseFlowData, BaseNodeData>;

void Generate(const BaseSkeleton& skeleton, std::vector<Vertex>& vertices, std::vector<unsigned>& indices) {
  MeshGenerator::Generate(
      skeleton, vertices, indices, {},
      [](glm::vec3&, const glm::vec3&, float, float) {
      },
      [](glm::vec2&, float, float) {
      });
}

template <typename VertexType>
void ExpectValidMesh(const std::vector<VertexType>& vertices, const std::vector<unsigned>& indices) {
  ASSERT_FALSE(vertices.empty());
  ASSERT_FALSE(indices.empty());
  for (const auto& vertex : vertices) {
    EXPECT_TRUE(std::isfinite(vertex.position.x));
    EXPECT_TRUE(std::isfinite(vertex.position.y));
    EXPECT_TRUE(std::isfinite(vertex.position.z));
  }
  for (const auto index : indices)
    EXPECT_LT(index, vertices.size());
}
}  // namespace

TEST(TreeSkeletonMeshing, LeafSizeDoesNotDependOnTreeExtent) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  BasicFoliageDescriptor foliage;
  foliage.leaf_count = 1;
  foliage.gravitropism = 0.0f;
  SkeletonNodeInfo node;
  node.thickness = 0.01f;
  node.root_distance = 1.0f;
  node.end_distance = 0.0f;
  std::vector<glm::mat4> small_tree, large_tree;
  foliage.GenerateFoliageMatrices(small_tree, node, 1.0f);
  foliage.GenerateFoliageMatrices(large_tree, node, 10.0f);
  ASSERT_EQ(small_tree.size(), 1);
  ASSERT_EQ(large_tree.size(), 1);
  for (const auto& matrices : {small_tree, large_tree}) {
    EXPECT_NEAR(glm::length(glm::vec3(matrices[0][0])), foliage.leaf_size.x, 1e-5f);
    EXPECT_NEAR(glm::length(glm::vec3(matrices[0][2])), foliage.leaf_size.y, 1e-5f);
  }
}

TEST(TreeSkeletonMeshing, SkipsZeroLengthParentWithoutLosingChild) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  evo_engine::Jobs::Initialize(2);
  BaseSkeleton skeleton;
  const auto child = skeleton.Extend(0, false);
  skeleton.RefNode(child).info.length = 1.0f;
  skeleton.SortLists();
  skeleton.CalculateFlows();
  skeleton.CalculateDistanceVolumeLevel();

  std::vector<Vertex> vertices;
  std::vector<unsigned> indices;
  Generate(skeleton, vertices, indices);
  ExpectValidMesh(vertices, indices);

  vertices.clear();
  indices.clear();
  MeshGenerator::GeneratePartially(
      {0, child}, skeleton, vertices, indices, {},
      [](glm::vec3&, const glm::vec3&, float, float) {
      },
      [](glm::vec2&, float, float) {
      });
  ExpectValidMesh(vertices, indices);

  std::vector<SkinnedVertex> skinned_vertices;
  std::vector<glm::mat4> offsets;
  indices.clear();
  SkinnedMeshGenerator::Generate(
      skeleton, skinned_vertices, indices, offsets, {},
      [](glm::vec3&, const glm::vec3&, float, float) {
      },
      [](glm::vec2&, float, float) {
      });
  ExpectValidMesh(skinned_vertices, indices);
}

TEST(TreeSkeletonMeshing, StitchesAcrossZeroLengthInteriorNode) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  evo_engine::Jobs::Initialize(2);
  BaseSkeleton skeleton;
  skeleton.RefNode(0).info.length = 1.0f;
  const auto empty = skeleton.Extend(0, false);
  const auto child = skeleton.Extend(empty, false);
  skeleton.RefNode(empty).info.global_position = glm::vec3(0, 0, -1);
  skeleton.RefNode(child).info.length = 1.0f;
  skeleton.RefNode(child).info.global_position = glm::vec3(0, 0, -1);
  skeleton.SortLists();
  skeleton.CalculateFlows();
  skeleton.CalculateDistanceVolumeLevel();

  std::vector<Vertex> vertices;
  std::vector<unsigned> indices;
  Generate(skeleton, vertices, indices);
  ExpectValidMesh(vertices, indices);
  bool joined = false;
  for (size_t i = 0; i < indices.size(); i += 3) {
    bool root_vertex = false;
    bool child_vertex = false;
    for (size_t j = 0; j < 3; ++j) {
      root_vertex |= vertices[indices[i + j]].vertex_info1 == 1;
      child_vertex |= vertices[indices[i + j]].vertex_info1 == child + 1;
    }
    joined |= root_vertex && child_vertex;
  }
  EXPECT_TRUE(joined);
}

TEST(TreeSkeletonMeshing, DetachedBranchUsesItsOwnStartRing) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  evo_engine::Jobs::Initialize(2);
  BaseSkeleton skeleton;
  skeleton.RefNode(0).info.length = 1.0f;
  skeleton.RefNode(0).info.thickness = 0.02f;
  const auto apical = skeleton.Extend(0, false);
  const auto branch = skeleton.Extend(0, true);
  skeleton.RefNode(apical).info.length = 1.0f;
  skeleton.RefNode(apical).info.global_position = glm::vec3(0, 0, -1);
  skeleton.RefNode(branch).info.length = 1.0f;
  skeleton.RefNode(branch).info.thickness = 0.2f;
  skeleton.RefNode(branch).info.global_position = glm::vec3(0, 0, -1);
  skeleton.SortLists();
  skeleton.CalculateFlows();
  skeleton.CalculateDistanceVolumeLevel();

  std::vector<Vertex> vertices;
  std::vector<unsigned> indices;
  Generate(skeleton, vertices, indices);
  ExpectValidMesh(vertices, indices);
  bool found_branch = false;
  for (size_t i = 0; i < indices.size(); i += 3) {
    bool touches_branch = false;
    for (size_t j = 0; j < 3; ++j)
      touches_branch |= vertices[indices[i + j]].vertex_info1 == branch + 1;
    if (touches_branch) {
      found_branch = true;
      for (size_t j = 0; j < 3; ++j)
        EXPECT_EQ(vertices[indices[i + j]].vertex_info1, branch + 1);
    }
  }
  EXPECT_TRUE(found_branch);
}

TEST(TreeSkeletonMeshing, RingDirectionRemainsFiniteWhenReferenceIsParallelToAxis) {
  const RingSegment ring(0, 1, {}, {}, glm::vec3(0, 1, 0), {}, 1, 1, 0, 1);
  const auto start = ring.GetDirection(glm::vec3(0, 1, 0), 90, true);
  const auto end = ring.GetDirection({}, 90, false);
  for (const auto& direction : {start, end}) {
    EXPECT_TRUE(std::isfinite(direction.x));
    EXPECT_TRUE(std::isfinite(direction.y));
    EXPECT_TRUE(std::isfinite(direction.z));
    EXPECT_NEAR(glm::length(direction), 1.0f, 1e-5f);
  }
}

TEST(TreeSkeletonMeshing, PacksForkParentsAndRebasesTreeRanges) {
  BaseSkeleton skeleton;
  skeleton.RefNode(0).info.length = 1.0f;
  skeleton.RefNode(0).info.thickness = 0.2f;
  const auto first = skeleton.Extend(0, false);
  const auto second = skeleton.Extend(0, true);
  skeleton.RefNode(first).info.thickness = 0.05f;
  skeleton.RefNode(second).info.thickness = 0.02f;
  skeleton.RefNode(first).info.leaves = 3.0f;
  skeleton.RefNode(first).info.end_distance = 0.1f;
  skeleton.SortLists();

  std::vector<GpuTreeNode> nodes;
  AppendGpuTreeNodes(skeleton, {}, nullptr, 3, true, nodes);
  ASSERT_EQ(nodes.size(), 3);
  EXPECT_EQ(nodes[0].links.x, UINT32_MAX);
  EXPECT_EQ(nodes[1].links.x, 0);
  EXPECT_EQ(nodes[2].links.x, 0);
  EXPECT_EQ(nodes[1].links.y, 3);
  EXPECT_FLOAT_EQ(nodes[0].position_thickness.w, 0.2f);
  EXPECT_FLOAT_EQ(nodes[1].foliage.x, 3.0f);
  EXPECT_FLOAT_EQ(nodes[1].foliage.y, 0.1f);
  EXPECT_FLOAT_EQ(nodes[1].foliage.z, 0.05f);
  EXPECT_EQ(nodes[1].foliage_meta.x, static_cast<uint32_t>(first));
  EXPECT_EQ(nodes[1].foliage_meta.y, 1u);
  EXPECT_EQ(nodes[1].links.w, UINT32_MAX);

  AppendGpuTreeNodes(skeleton, {}, nullptr, 7, false, nodes);
  ASSERT_EQ(nodes.size(), 6);
  EXPECT_EQ(nodes[4].links.x, 3);
  EXPECT_EQ(nodes[5].links.x, 3);
  EXPECT_EQ(nodes[4].links.y, 7);
  EXPECT_EQ(nodes[4].foliage_meta.y, 0u);
}

TEST(TreeSkeletonMeshing, ReusesPackedNodesAtDifferentTreeOffsets) {
  BaseSkeleton skeleton;
  skeleton.Extend(0, false);
  skeleton.SortLists();
  std::vector<GpuTreeNode> packed;
  AppendGpuTreeNodes(skeleton, {}, nullptr, 0, true, packed);

  std::vector<GpuTreeNode> nodes;
  AppendPackedGpuTreeNodes(packed, 3, nodes);
  AppendPackedGpuTreeNodes(packed, 7, nodes);

  ASSERT_EQ(nodes.size(), 4);
  EXPECT_EQ(nodes[0].links.x, UINT32_MAX);
  EXPECT_EQ(nodes[1].links.x, 0u);
  EXPECT_EQ(nodes[2].links.x, UINT32_MAX);
  EXPECT_EQ(nodes[3].links.x, 2u);
  EXPECT_EQ(nodes[0].links.y, 3u);
  EXPECT_EQ(nodes[3].links.y, 7u);
  EXPECT_EQ(packed[1].links.x, 0u);
  EXPECT_EQ(packed[1].links.y, 0u);
}
