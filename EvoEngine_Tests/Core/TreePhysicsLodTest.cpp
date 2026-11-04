#include <gtest/gtest.h>
#include <cmath>
#include <glm/gtx/quaternion.hpp>
#include "Application.hpp"
#include "Serialization.hpp"
#include "TreeDescriptor.hpp"
#include "TreePhysicsLod.hpp"

using namespace eco_sys_lab_package;
namespace {
DynamicTreeSkeleton::GpuRestState Chain(uint32_t count) {
  DynamicTreeSkeleton::GpuRestState rest;
  for (uint32_t i = 0; i <= count; ++i)
    rest.vertices.push_back(
        {{0.1f * i, 0, 0, i == 0 ? 0.0f : (i == count ? 2.0f : 1.0f)}, {0, 0, 0, 0.1f}, {0.1f * i, 0, 0, 0}, {}});
  for (uint32_t i = 0; i < count; ++i) {
    rest.rods.push_back({{i, i + 1, i, i == 0 ? 1u : 0u}, {0.1f, 1, 1, 0.1f}, {0, 0, 0, 1}, {0, 0, 0, 1}, {}});
    rest.node_rods.push_back(i);
    if (i)
      rest.angles.push_back({{i - 1, i, 0, 0}, {0, 0, 0, 1}, {1, 0, 0, 0}});
  }
  return rest;
}
glm::quat Q(glm::vec4 v) {
  return {v.w, v.x, v.y, v.z};
}
}  // namespace
TEST(TreePhysicsLod, NestedChainsPreserveMassComplianceAndRestReconstruction) {
  const auto original = Chain(33);
  const auto levels = BuildTreePhysicsLod(original);
  ASSERT_GT(levels.size(), 4u);
  EXPECT_EQ(levels.front().rest.rods.size(), original.rods.size());
  float last_error = 0;
  for (size_t l = 0; l < levels.size(); ++l) {
    const auto& level = levels[l];
    SCOPED_TRACE(l);
    float mass = 0, compliance = 0;
    for (const auto& rod : level.rest.rods) {
      mass += rod.length_scale_mass_radius.z;
      compliance += rod.length_scale_mass_radius.x / rod.length_scale_mass_radius.y;
    }
    EXPECT_NEAR(mass, 33, 1e-5f);
    EXPECT_NEAR(compliance, 3.3f, 1e-5f);
    EXPECT_GE(level.approximation_error, last_error);
    last_error = level.approximation_error;
    EXPECT_NE(level.vertices.front().links.y, UINT32_MAX);
    EXPECT_NE(level.vertices.back().links.y, UINT32_MAX);
    for (uint32_t i = 0; i < original.vertices.size(); ++i) {
      const auto& map = level.vertices[i];
      glm::vec3 position;
      if (map.links.y != UINT32_MAX)
        position = level.rest.vertices[map.links.y].position_inverse_mass;
      else {
        const auto& rod = level.rest.rods[map.links.x];
        const glm::vec3 a = level.rest.vertices[rod.links.x].position_inverse_mass,
                        b = level.rest.vertices[rod.links.y].position_inverse_mass;
        position = glm::mix(a, b, map.offset_t.w) + Q(rod.rest_rotation) * glm::vec3(map.offset_t);
      }
      EXPECT_LT(glm::length(position - glm::vec3(original.vertices[i].position_inverse_mass)), 1e-6f);
    }
    for (const auto& angle : level.rest.angles) {
      const float expected =
          2.0f / static_cast<float>(level.groups[angle.rods.x].size() + level.groups[angle.rods.y].size());
      EXPECT_NEAR(angle.bend_scale.x, expected, 1e-6f);
    }
    if (!l)
      continue;
    EXPECT_LT(level.rest.rods.size(), levels[l - 1].rest.rods.size());
    for (const auto& group : levels[l - 1].groups)
      for (auto rod : group)
        EXPECT_EQ(level.rods[rod].links.x, level.rods[group.front()].links.x);
  }
}
TEST(TreePhysicsLod, ForksAndThicknessChangesRemainExplicit) {
  auto rest = Chain(8);
  rest.vertices.push_back({{0.4f, 0.1f, 0, 2}, {0, 0, 0, 0.1f}, {0.4f, 0.1f, 0, 0}, {}});
  rest.rods.push_back(
      {{4, 9, 8, 0}, {0.1f, 1, 1, 0.1f}, {0, 0, 0.70710678f, 0.70710678f}, {0, 0, 0.70710678f, 0.70710678f}, {}});
  rest.angles.push_back({{3, 8, 0, 0}, {0, 0, 0.70710678f, 0.70710678f}, {1, 0, 0, 0}});
  rest.rods[6].length_scale_mass_radius.w = 0.01f;
  for (const auto& level : BuildTreePhysicsLod(rest)) {
    EXPECT_NE(level.vertices[4].links.y, UINT32_MAX);
    EXPECT_NE(level.vertices[8].links.y, UINT32_MAX);
    EXPECT_NE(level.vertices[9].links.y, UINT32_MAX);
    EXPECT_NE(level.rods[5].links.x, level.rods[6].links.x);
    EXPECT_NE(level.rods[6].links.x, level.rods[7].links.x);
    EXPECT_NE(level.rods[0].links.x, level.rods[1].links.x);
  }
}
TEST(TreePhysicsLod, CurvedRestOffsetsReconstructOriginalAndExposeError) {
  auto rest = Chain(16);
  for (uint32_t i = 0; i < rest.vertices.size(); ++i) {
    rest.vertices[i].position_inverse_mass.y = 0.002f * std::sin(0.2f * i);
    rest.vertices[i].rest_local = glm::vec4(glm::vec3(rest.vertices[i].position_inverse_mass), 0);
  }
  for (auto& rod : rest.rods) {
    const glm::vec3 d =
        rest.vertices[rod.links.y].position_inverse_mass - rest.vertices[rod.links.x].position_inverse_mass;
    const auto q = glm::rotation(glm::vec3(1, 0, 0), glm::normalize(d));
    rod.rotation = rod.rest_rotation = {q.x, q.y, q.z, q.w};
    rod.length_scale_mass_radius.x = glm::length(d);
  }
  const auto levels = BuildTreePhysicsLod(rest);
  ASSERT_GT(levels.size(), 1u);
  EXPECT_GT(levels.back().approximation_error, 0);
  for (const auto& level : levels)
    for (uint32_t i = 0; i < rest.vertices.size(); ++i) {
      const auto& map = level.vertices[i];
      if (map.links.y != UINT32_MAX)
        continue;
      const auto& rod = level.rest.rods[map.links.x];
      const glm::vec3 a = level.rest.vertices[rod.links.x].position_inverse_mass,
                      b = level.rest.vertices[rod.links.y].position_inverse_mass;
      const auto reconstructed = glm::mix(a, b, map.offset_t.w) + Q(rod.rest_rotation) * glm::vec3(map.offset_t);
      EXPECT_LT(glm::length(reconstructed - glm::vec3(rest.vertices[i].position_inverse_mass)), 1e-6f);
    }
}
TEST(TreePhysicsProperties, DescriptorCopiesAreIndependentAndDoNotChangeQuality) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  TreeDescriptor descriptor;
  descriptor.physics_properties.density = 350;
  DynamicTreeSkeleton a, b;
  a.physics_settings.substeps = 7;
  a.physics_settings.approximation_tolerance = 0.25f;
  a.SetPhysicsProperties(descriptor.physics_properties);
  b.SetPhysicsProperties(descriptor.physics_properties);
  a.physics_settings.density = 900;
  EXPECT_EQ(b.physics_settings.density, 350);
  EXPECT_EQ(descriptor.physics_properties.density, 350);
  EXPECT_EQ(a.physics_settings.substeps, 7);
  EXPECT_EQ(a.physics_settings.approximation_tolerance, 0.25f);
  YAML::Emitter out;
  out << YAML::BeginMap;
  SerializeTreePhysicsProperties(out, a.physics_settings);
  out << YAML::EndMap;
  TreePhysicsProperties restored;
  DeserializeTreePhysicsProperties(YAML::Load(out.c_str()), restored);
  EXPECT_TRUE(restored == static_cast<const TreePhysicsProperties&>(a.physics_settings));
}

TEST(TreePhysicsLod, FreeHingesRemainExplicit) {
  auto rest = Chain(9);
  rest.angles[3].bend_scale.x = 0;
  for (const auto& level : BuildTreePhysicsLod(rest))
    EXPECT_NE(level.rods[3].links.x, level.rods[4].links.x);
}
TEST(TreePhysicsLod, DifficultChainDoesNotBlockSimplifyingOtherBranches) {
  auto rest = Chain(17);
  rest.rods[15].length_scale_mass_radius.w = rest.rods[16].length_scale_mass_radius.w = 0.002f;
  rest.vertices.back().position_inverse_mass.y = 0.0087f;
  rest.vertices.back().rest_local.y = 0.0087f;
  const auto q = glm::angleAxis(std::atan2(0.0087f, 0.1f), glm::vec3(0, 0, 1));
  rest.rods.back().rotation = rest.rods.back().rest_rotation = {q.x, q.y, q.z, q.w};
  rest.rods.back().length_scale_mass_radius.x = std::sqrt(0.01f + 0.0087f * 0.0087f);
  rest.angles.back().rest_relative = rest.rods.back().rest_rotation;
  bool eligible = false;
  for (const auto& level : BuildTreePhysicsLod(rest))
    eligible |= level.rest.rods.size() < rest.rods.size() && std::max(level.approximation_error, level.span_error) <= 1;
  EXPECT_TRUE(eligible);
}
TEST(TreePhysicsLod, SlenderOriginalSamplingAllowsBoundedMerges) {
  auto rest = Chain(17);
  for (auto& rod : rest.rods)
    rod.length_scale_mass_radius.w = 0.002f;
  const auto levels = BuildTreePhysicsLod(rest);
  ASSERT_GT(levels.size(), 1u);
  EXPECT_LE(levels[1].span_error, 0.25f);
  for (const auto& group : levels[1].groups)
    EXPECT_LE(group.size(), 4u);
}
