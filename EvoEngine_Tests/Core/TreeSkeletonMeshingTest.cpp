#include <gtest/gtest.h>

#include <cmath>

#include "../../EvoEngine_Packages/EcoSysLab/Editor/include/PlantVisualOrganBuffers.hpp"
#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "BasicFoliageDescriptor.hpp"
#include "GpuTreeDebugData.hpp"
#include "GpuTreeMeshData.hpp"
#include "Jobs.hpp"
#include "TreeMeshGenerator.hpp"
#include "TreeSkinnedMeshGenerator.hpp"

using namespace eco_sys_lab_package;

TEST(TreeDebugPicking, DecodesOriginalNodeIdentityAndPositionForBothSkeletons) {
  uint32_t factor;
  const float expected = 0.375f;
  std::memcpy(&factor, &expected, sizeof(factor));
  const auto shoot = DecodeGpuTreeDebugPick({8, 1, factor, 0}, 10, 2);
  ASSERT_TRUE(shoot);
  EXPECT_EQ(shoot->node, 7u);
  EXPECT_TRUE(shoot->shoot);
  EXPECT_FLOAT_EQ(shoot->length_factor, expected);
  const auto root = DecodeGpuTreeDebugPick({2, 0, factor, 0}, 1, 3);
  ASSERT_TRUE(root);
  EXPECT_EQ(root->node, 1u);
  EXPECT_FALSE(root->shoot);
  EXPECT_FLOAT_EQ(root->length_factor, expected);
}

TEST(TreeDebugPicking, RejectsBackgroundInvalidHandlesAndNonFinitePositions) {
  EXPECT_FALSE(DecodeGpuTreeDebugPick({}, 10, 10));
  EXPECT_FALSE(DecodeGpuTreeDebugPick({11, 1, 0, 0}, 10, 20));
  EXPECT_FALSE(DecodeGpuTreeDebugPick({11, 0, 0, 0}, 20, 10));
  EXPECT_FALSE(DecodeGpuTreeDebugPick({1, 2, 0, 0}, 10, 10));
  for (float value : {-0.1f, 1.1f, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    EXPECT_FALSE(DecodeGpuTreeDebugPick({1, 1, bits, 0}, 10, 10));
  }
}

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

TEST(PlantVisualOrgans, PreservesOrganOrderTransformsColorsAndVisibility) {
  ShootSkeleton skeleton;
  skeleton.Extend(0, false);
  skeleton.SortLists();
  for (const auto handle : skeleton.PeekSortedNodeList()) {
    auto& data = skeleton.RefNode(handle).data;
    data.leaves.resize(3);
    data.flowers.resize(3);
    data.fruits.resize(3);
    const auto initialize = [&](auto& organs) {
      for (size_t i = 0; i < organs.size(); ++i) {
        auto& organ = organs[i];
        organ.status = i == 1 ? OrganStatus::Inactive : (i == 0 ? OrganStatus::Dormant : OrganStatus::Flushed);
        organ.position = {handle * 3.0f, static_cast<float>(i), -2};
        organ.rotation = glm::quat(glm::vec3(0.2f, 0.4f, -0.3f));
        organ.scale = {0.5f, 2.0f, 1.0f};
        organ.health = 0.25f;
        organ.maturity = 0.75f;
      }
    };
    initialize(data.leaves);
    initialize(data.flowers);
    initialize(data.fruits);
  }
  const auto transform = glm::translate(glm::vec3(4, -3, 2)) * glm::mat4_cast(glm::quat(glm::vec3(0.3f, -0.2f, 0.1f))) *
                         glm::scale(glm::vec3(2, 1, 3));
  PlantVisualOrganBuffers buffers;
  for (uint32_t mask = 0; mask < 8; ++mask) {
    buffers.Build(skeleton, transform, mask & 1, mask & 2, mask & 4);
    const std::vector<ParticleInfo>* lists[] = {&buffers.foliage, &buffers.flowers, &buffers.fruits};
    const float scales[] = {0.5f, 1.0f, 0.25f};
    const glm::vec4 colors[] = {
        {glm::mix(glm::vec3(152, 203, 0) / 255.0f, glm::vec3(159, 100, 66) / 255.0f, 0.75f), 0.5f},
        {glm::mix(glm::vec3(1), glm::vec3(255, 192, 203) / 255.0f, 0.75f), 0.75f},
        {glm::mix(glm::vec3(152, 255, 152) / 255.0f, glm::vec3(255, 165, 0) / 255.0f, 0.75f), 0.75f}};
    for (uint32_t type = 0; type < 3; ++type) {
      ASSERT_EQ(lists[type]->size(), mask & (1u << type) ? 4u : 0u);
      for (size_t i = 0; i < lists[type]->size(); ++i) {
        const auto& organ = skeleton.PeekNode(static_cast<int>(i / 2)).data.leaves[(i % 2) * 2];
        const auto& result = lists[type]->at(i);
        const glm::vec4 local(0.2f, -0.4f, 0.7f, 1);
        const glm::vec4 expected =
            transform * glm::vec4(organ.position + organ.rotation * (organ.scale * scales[type] * glm::vec3(local)), 1);
        EXPECT_LT(glm::length(result.instance_matrix.value * local - expected), 1e-5f);
        EXPECT_LT(glm::length(result.instance_color - colors[type]), 1e-6f);
      }
    }
  }
}

TEST(PlantVisualOrgans, ReusesBuffersAndCombinesTreesInInputOrder) {
  ShootSkeleton skeleton;
  skeleton.SortLists();
  auto& organs = skeleton.RefNode(0).data.leaves;
  organs.resize(8);
  for (auto& organ : organs)
    organ.status = OrganStatus::Flushed;
  PlantVisualOrganBuffers first, second, combined;
  first.Build(skeleton, glm::mat4(1), true, false, false);
  second.Build(skeleton, glm::translate(glm::vec3(10, 0, 0)), true, false, false);
  const auto* first_storage = first.foliage.data();
  const auto capacity = first.foliage.capacity();
  combined.Append(first);
  combined.Append(second);
  ASSERT_EQ(combined.foliage.size(), 16u);
  EXPECT_FLOAT_EQ(combined.foliage.front().instance_matrix.value[3].x, 0);
  EXPECT_FLOAT_EQ(combined.foliage[8].instance_matrix.value[3].x, 10);
  const auto* combined_storage = combined.foliage.data();
  combined.Clear();
  combined.Append(second);
  combined.Append(first);
  EXPECT_EQ(combined.foliage.data(), combined_storage);
  EXPECT_FLOAT_EQ(combined.foliage.front().instance_matrix.value[3].x, 10);
  first.Build(skeleton, glm::mat4(1), false, false, false);
  EXPECT_TRUE(first.foliage.empty());
  EXPECT_EQ(first.foliage.capacity(), capacity);
  organs.resize(2);
  first.Build(skeleton, glm::mat4(1), true, false, false);
  EXPECT_EQ(first.foliage.size(), 2u);
  EXPECT_EQ(first.foliage.data(), first_storage);
  first.Clear();
  EXPECT_TRUE(first.foliage.empty());
  EXPECT_EQ(first.foliage.capacity(), capacity);
}
