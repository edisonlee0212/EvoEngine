#include <gtest/gtest.h>
#include <algorithm>
#include <glm/gtx/quaternion.hpp>
#include "Application.hpp"
#include "GpuTreeSubtreePhysics.hpp"
#include "TreeClusterHierarchy.hpp"

using namespace eco_sys_lab_package;
namespace {
using Rest = DynamicTreeSkeleton::GpuRestState;
Rest Branches() {
  Rest rest;
  const std::vector<glm::vec3> points{{0, 0, 0},    {0, 5, 0},    {0, 10, 0},      {1, 5, 0},       {2, 5, 0},
                                      {1, 5.3f, 0}, {1, 5.6f, 0}, {1.2f, 5.3f, 0}, {1.4f, 5.3f, 0}, {-3, 10, 0},
                                      {-3, 11, 0},  {-3, 12, 0},  {0, 10, 0.3f},   {0, 10, 0.6f}};
  const std::vector<glm::uvec2> links{{0, 1}, {1, 2}, {1, 3},  {3, 4},   {3, 5},  {5, 6},  {5, 7},
                                      {7, 8}, {2, 9}, {9, 10}, {10, 11}, {2, 12}, {12, 13}};
  const std::vector<float> radii{0.5f,   0.4f, 0.1f,  0.08f, 0.02f, 0.015f, 0.01f,
                                 0.008f, 0.2f, 0.03f, 0.02f, 0.02f, 0.02f};
  rest.branch_orders = {0, 0, 1, 1, 2, 2, 3, 3, 1, 2, 2, 1, 1};
  for (const auto& point : points)
    rest.vertices.push_back({{point, 1}, {}, {point, 0}, {}});
  for (uint32_t i = 0; i < links.size(); ++i) {
    const auto link = links[i];
    const auto delta = points[link.y] - points[link.x];
    const auto q = glm::rotation(glm::vec3(1, 0, 0), glm::normalize(delta));
    rest.rods.push_back({{link.x, link.y, i, i == 0 ? 1u : 0u},
                         {glm::length(delta), 1, 1, radii[i]},
                         {q.x, q.y, q.z, q.w},
                         {q.x, q.y, q.z, q.w},
                         {}});
  }
  return rest;
}
TreeClusterRule Rule(TreeClusterPolicy policy) {
  TreeClusterRule rule;
  rule.policy = policy;
  rule.maximum_extent_ratio = 0.1f;
  return rule;
}
std::vector<uint32_t> ClusterRoots(const TreeClusterSelection& selection) {
  std::vector<uint32_t> roots;
  for (const auto& element : selection.elements)
    if (element.IsCluster())
      roots.push_back(element.root_rod);
  std::sort(roots.begin(), roots.end());
  return roots;
}
void CheckCoverage(const TreeClusterHierarchy& hierarchy, const TreeClusterSelection& selection) {
  ASSERT_EQ(selection.rod_elements.size(), hierarchy.candidates.size());
  std::vector<uint32_t> owners(hierarchy.candidates.size(), UINT32_MAX);
  for (uint32_t i = 0; i < selection.elements.size(); ++i) {
    const auto& element = selection.elements[i];
    const auto& candidate = hierarchy.candidates[element.root_rod];
    ASSERT_EQ(element.begin, candidate.begin);
    ASSERT_EQ(element.end, element.IsCluster() ? candidate.end : candidate.begin + 1);
    if (candidate.parent == UINT32_MAX)
      EXPECT_EQ(element.parent, UINT32_MAX);
    else {
      EXPECT_EQ(element.parent, selection.rod_elements[candidate.parent]);
      EXPECT_LT(element.parent, i);
    }
    for (uint32_t j = element.begin; j < element.end; ++j) {
      const auto rod = hierarchy.preorder[j];
      EXPECT_EQ(owners[rod], UINT32_MAX);
      owners[rod] = i;
    }
  }
  EXPECT_EQ(owners, selection.rod_elements);
  EXPECT_EQ(std::count(owners.begin(), owners.end(), UINT32_MAX), 0);
}
}  // namespace

TEST(TreeClusterHierarchy, ExportsBotanicalOrderAcrossContinuationAndSkippedNodes) {
  Application application;
  ApplicationContextScope scope(application);
  BaseSkeleton skeleton;
  const auto continuation = skeleton.Extend(0, false);
  const auto lateral = skeleton.Extend(0, true);
  const auto lateral_continuation = skeleton.Extend(lateral, false);
  const auto skipped = skeleton.Extend(lateral, true);
  const auto child = skeleton.Extend(skipped, false);
  skeleton.SortLists();
  for (const auto node : skeleton.PeekSortedNodeList()) {
    auto& info = skeleton.RefNode(node).info;
    info.length = node == skipped ? 0 : 1;
    info.thickness = 0.1f;
    info.global_rotation = glm::rotation(glm::vec3(0, 0, -1), glm::vec3(0, 1, 0));
    info.order = 99;  // Cached derived fields may not have been refreshed yet.
  }
  Tree tree;
  tree.shoot_model.Initialize(skeleton);
  DynamicTreeSkeleton physics;
  ASSERT_TRUE(physics.InitializePhysics(tree));
  const auto rest = physics.ExportGpuRestState();
  ASSERT_EQ(rest.branch_orders.size(), rest.rods.size());
  EXPECT_EQ(rest.branch_orders[rest.node_rods[0]], 0u);
  EXPECT_EQ(rest.branch_orders[rest.node_rods[continuation]], 0u);
  EXPECT_EQ(rest.branch_orders[rest.node_rods[lateral]], 1u);
  EXPECT_EQ(rest.branch_orders[rest.node_rods[lateral_continuation]], 1u);
  EXPECT_EQ(rest.node_rods[skipped], UINT32_MAX);
  EXPECT_EQ(rest.branch_orders[rest.node_rods[child]], 2u);
}

TEST(TreeClusterHierarchy, PoliciesChooseLargestNonOverlappingSubtrees) {
  const auto rest = Branches();
  const auto hierarchy = BuildTreeClusterHierarchy(rest).value();
  const auto size = Rule(TreeClusterPolicy::Size), order = Rule(TreeClusterPolicy::BranchOrder),
             combined = Rule(TreeClusterPolicy::Combined);
  const auto a = SelectTreeClusters(hierarchy, {size}), b = SelectTreeClusters(hierarchy, {order}),
             c = SelectTreeClusters(hierarchy, {combined}), d = SelectTreeClusters(hierarchy, {size, order});
  EXPECT_EQ(ClusterRoots(a), (std::vector<uint32_t>{4, 11}));
  EXPECT_EQ(ClusterRoots(b), (std::vector<uint32_t>{4, 9}));
  EXPECT_EQ(ClusterRoots(c), (std::vector<uint32_t>{4}));
  EXPECT_EQ(ClusterRoots(d), (std::vector<uint32_t>{4, 9, 11}));
  for (const auto& selection : {a, b, c, d})
    CheckCoverage(hierarchy, selection);
  EXPECT_EQ(d.elements[d.rod_elements[4]].policy_mask,
            (1u << static_cast<uint32_t>(TreeClusterPolicy::Size)) |
                (1u << static_cast<uint32_t>(TreeClusterPolicy::BranchOrder)));
  EXPECT_EQ(d.rod_elements[4], d.rod_elements[6]);
  auto generous = size;
  generous.maximum_extent_ratio = 0.5f;
  generous.maximum_attachment_radius_ratio = 0.5f;
  const auto larger = SelectTreeClusters(hierarchy, {order, generous});
  EXPECT_EQ(larger.rod_elements[2], larger.rod_elements[4]);
  EXPECT_EQ(larger.elements[larger.rod_elements[4]].root_rod, 2u);
  CheckCoverage(hierarchy, larger);
  EXPECT_EQ(ClusterRoots(d), ClusterRoots(SelectTreeClusters(hierarchy, {order, size, order})));
}

TEST(TreeClusterHierarchy, CandidatesFeedExistingBodyRepresentation) {
  const auto rest = Branches();
  const auto hierarchy = BuildTreeClusterHierarchy(rest).value();
  const auto selection = SelectTreeClusters(hierarchy, {Rule(TreeClusterPolicy::BranchOrder)});
  for (const auto& element : selection.elements) {
    if (!element.IsCluster())
      continue;
    const auto body = BuildTreeSubtreeBody(rest, element.root_rod);
    ASSERT_TRUE(body);
    const auto& candidate = hierarchy.candidates[element.root_rod];
    EXPECT_EQ(body->rods.size(), element.end - element.begin);
    EXPECT_NEAR(body->mass, candidate.mass, 1e-5f);
    std::vector<uint32_t> expected(hierarchy.preorder.begin() + element.begin,
                                   hierarchy.preorder.begin() + element.end),
        actual;
    for (const auto& rod : body->rods)
      actual.push_back(rod.indices.x);
    std::sort(expected.begin(), expected.end());
    std::sort(actual.begin(), actual.end());
    EXPECT_EQ(actual, expected);
  }
  EXPECT_EQ(hierarchy.candidates[4].end - hierarchy.candidates[4].begin, 4u);
  EXPECT_NEAR(hierarchy.candidates[4].total_length, 1.0f, 1e-5f);
  EXPECT_FLOAT_EQ(hierarchy.candidates[4].mass, 4);
}

TEST(TreeClusterHierarchy, MissingOrderDoesNotGuessAndLimitsRemainIndependent) {
  auto rest = Branches();
  rest.branch_orders.clear();
  const auto hierarchy = BuildTreeClusterHierarchy(rest).value();
  EXPECT_FALSE(hierarchy.has_branch_orders);
  EXPECT_TRUE(ClusterRoots(SelectTreeClusters(hierarchy, {Rule(TreeClusterPolicy::BranchOrder)})).empty());
  EXPECT_TRUE(ClusterRoots(SelectTreeClusters(hierarchy, {Rule(TreeClusterPolicy::Combined)})).empty());
  EXPECT_EQ(ClusterRoots(SelectTreeClusters(hierarchy, {Rule(TreeClusterPolicy::Size)})),
            (std::vector<uint32_t>{4, 11}));
  const auto unchanged = SelectTreeClusters(hierarchy, {});
  EXPECT_EQ(unchanged.elements.size(), rest.rods.size());
  CheckCoverage(hierarchy, unchanged);
  auto size = Rule(TreeClusterPolicy::Size);
  size.maximum_attachment_radius_ratio = 0.01f;
  EXPECT_TRUE(ClusterRoots(SelectTreeClusters(hierarchy, {size})).empty());
  auto order = Rule(TreeClusterPolicy::BranchOrder);
  order.minimum_rods = 3;
  EXPECT_EQ(ClusterRoots(SelectTreeClusters(BuildTreeClusterHierarchy(Branches()).value(), {order})),
            (std::vector<uint32_t>{4}));
}

TEST(TreeClusterHierarchy, PinnedDescendantsPreventContainingClusters) {
  auto rest = Branches();
  rest.rods[7].links.w = 1;
  const auto hierarchy = BuildTreeClusterHierarchy(rest).value();
  const auto selection = SelectTreeClusters(hierarchy, {Rule(TreeClusterPolicy::BranchOrder)});
  EXPECT_EQ(ClusterRoots(selection), (std::vector<uint32_t>{9}));
  EXPECT_FALSE(hierarchy.candidates[4].collapsible);
  CheckCoverage(hierarchy, selection);
}

TEST(TreeClusterHierarchy, SizeRulesAreInvariantUnderRigidTransformsAndUniformScale) {
  auto rest = Branches();
  const auto before = BuildTreeClusterHierarchy(rest).value();
  const auto q = glm::angleAxis(1.2f, glm::normalize(glm::vec3(1, 2, 3)));
  const glm::vec3 translation(13, -7, 4);
  rest.initial_model =
      glm::translate(glm::mat4(1), translation) * glm::mat4_cast(q) * glm::scale(glm::mat4(1), glm::vec3(2));
  for (auto& vertex : rest.vertices)
    vertex.position_inverse_mass = {translation + q * (2.0f * glm::vec3(vertex.position_inverse_mass)),
                                    vertex.position_inverse_mass.w};
  for (auto& rod : rest.rods) {
    rod.length_scale_mass_radius.x *= 2;
    rod.length_scale_mass_radius.w *= 2;
  }
  const auto after = BuildTreeClusterHierarchy(rest).value();
  EXPECT_NEAR(after.tree_height, 2 * before.tree_height, 1e-4f);
  EXPECT_FLOAT_EQ(after.trunk_radius, 2 * before.trunk_radius);
  for (uint32_t i = 0; i < rest.rods.size(); ++i)
    EXPECT_NEAR(after.candidates[i].extent, 2 * before.candidates[i].extent, 1e-4f);
  const std::vector<TreeClusterRule> rules{Rule(TreeClusterPolicy::Combined), Rule(TreeClusterPolicy::Size)};
  EXPECT_EQ(ClusterRoots(SelectTreeClusters(before, rules)), ClusterRoots(SelectTreeClusters(after, rules)));
}

TEST(TreeClusterHierarchy, HandlesUnsortedRodsAndRejectsNonTreeTopology) {
  auto rest = Branches();
  std::reverse(rest.rods.begin(), rest.rods.end());
  std::reverse(rest.branch_orders.begin(), rest.branch_orders.end());
  const auto hierarchy = BuildTreeClusterHierarchy(rest).value();
  const auto selection = SelectTreeClusters(hierarchy, {Rule(TreeClusterPolicy::BranchOrder)});
  EXPECT_EQ(ClusterRoots(selection), (std::vector<uint32_t>{3, 8}));
  CheckCoverage(hierarchy, selection);
  rest = Branches();
  rest.rods[1].links.y = rest.rods[0].links.y;
  EXPECT_FALSE(BuildTreeClusterHierarchy(rest));
  rest = Branches();
  rest.rods[0].links.x = rest.rods[1].links.y;
  EXPECT_FALSE(BuildTreeClusterHierarchy(rest));
  EXPECT_TRUE(BuildTreeClusterHierarchy({})->candidates.empty());
}

TEST(TreeClusterHierarchy, DeepHierarchyUsesLinearStorageAndNoRecursion) {
  Rest rest;
  constexpr uint32_t count = 50000;
  for (uint32_t i = 0; i <= count; ++i)
    rest.vertices.push_back({{0, static_cast<float>(i), 0, 1}, {}, {}, {}});
  for (uint32_t i = 0; i < count; ++i)
    rest.rods.push_back({{i, i + 1, i, i == 0 ? 1u : 0u}, {1, 1, 1, 0.01f}, {0, 0, 0, 1}, {0, 0, 0, 1}, {}});
  rest.branch_orders.assign(count, 0);
  const auto hierarchy = BuildTreeClusterHierarchy(rest).value();
  auto rule = Rule(TreeClusterPolicy::BranchOrder);
  rule.minimum_branch_order = 0;
  const auto selection = SelectTreeClusters(hierarchy, {rule});
  ASSERT_EQ(hierarchy.candidates.size(), count);
  EXPECT_EQ(hierarchy.preorder.size(), count);
  ASSERT_EQ(selection.elements.size(), 2u);
  EXPECT_EQ(selection.elements[1].root_rod, 1u);
  EXPECT_EQ(selection.elements[1].end - selection.elements[1].begin, count - 1);
  CheckCoverage(hierarchy, selection);
}
