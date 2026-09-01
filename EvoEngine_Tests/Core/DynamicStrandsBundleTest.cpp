#include "EvoEngine_SDK_PCH.hpp"

#include "Camera.hpp"
#include "DynamicStrandsBundleDiagnostics.hpp"
#include "DynamicStrandsBundleMath.hpp"
#include "RenderLayer.hpp"

#include <gtest/gtest.h>

using namespace eco_sys_lab_package;

TEST(DynamicStrandsBundle, LegacySettingsRemainTheMissingSceneDefault) {
  BundleSolverSettings settings;
  settings.Load("bundle_solver", YAML::Load("initialize_parameters: {}"));

  EXPECT_EQ(settings.mode, BundleSolverMode::Legacy);
  EXPECT_EQ(settings.legacy_iterations, 1);
  EXPECT_FLOAT_EQ(settings.position_compliance_scale, 1.f);
  EXPECT_FLOAT_EQ(settings.slice_spacing_factor, 1.f);
  EXPECT_EQ(settings.minimum_slice_members, 8);
}

TEST(DynamicStrandsBundle, SolverSettingsRoundTrip) {
  BundleSolverSettings source;
  source.mode = BundleSolverMode::Hybrid;
  source.legacy_iterations = 3;
  source.pair_iterations = 7;
  source.coarse_iterations = 2;
  source.position_compliance_scale = .5f;
  source.bending_compliance_scale = 2.f;
  source.torsion_compliance_scale = 3.f;
  source.shape_matching_strength = .75f;
  source.slice_spacing_factor = 1.5f;
  source.minimum_slice_members = 12;

  YAML::Emitter out;
  out << YAML::BeginMap;
  source.Save("bundle_solver", out);
  out << YAML::EndMap;
  BundleSolverSettings loaded;
  loaded.Load("bundle_solver", YAML::Load(out.c_str()));

  EXPECT_EQ(loaded.mode, source.mode);
  EXPECT_EQ(loaded.legacy_iterations, source.legacy_iterations);
  EXPECT_EQ(loaded.pair_iterations, source.pair_iterations);
  EXPECT_EQ(loaded.coarse_iterations, source.coarse_iterations);
  EXPECT_FLOAT_EQ(loaded.position_compliance_scale, source.position_compliance_scale);
  EXPECT_FLOAT_EQ(loaded.bending_compliance_scale, source.bending_compliance_scale);
  EXPECT_FLOAT_EQ(loaded.torsion_compliance_scale, source.torsion_compliance_scale);
  EXPECT_FLOAT_EQ(loaded.shape_matching_strength, source.shape_matching_strength);
  EXPECT_FLOAT_EQ(loaded.slice_spacing_factor, source.slice_spacing_factor);
  EXPECT_EQ(loaded.minimum_slice_members, source.minimum_slice_members);
}

TEST(DynamicStrandsBundle, MomentumReferenceUsesSegmentMassAndVelocities) {
  DsMaterials materials;
  DynamicStrands strands(materials);
  auto& segment = strands.segments.emplace_back();
  segment.original_mass = 2.f;
  segment.particle0.x = {-1.f, 0.f, 0.f};
  segment.particle1.x = {1.f, 0.f, 0.f};
  segment.particle0.v = {1.f, 2.f, 3.f};
  segment.particle1.v = {1.f, 2.f, 3.f};

  const auto momentum = CalculateBundleMomentum(strands);

  EXPECT_EQ(momentum.linear, glm::vec3(2.f, 4.f, 6.f));
  EXPECT_EQ(momentum.angular, glm::vec3(0.f));
}

TEST(DynamicStrandsBundle, CoupledPairProducesEqualOppositeImpulseAndTorque) {
  BundleRigidBodyState body0{{0.f, 0.f, 0.f}, {}, 1.f, glm::mat3(1.f)};
  BundleRigidBodyState body1{{2.f, 1.f, 0.f}, {}, .5f, glm::mat3(.5f)};
  BundlePairReferenceState constraint;
  constraint.segment0_midpoint_offset = {1.f, 0.f, 0.f};
  constraint.segment1_midpoint_offset = {-1.f, 0.f, 0.f};

  const auto correction = SolveBundlePairReference(body0, body1, constraint);

  const glm::vec3 impulse0 = correction.position0 / body0.inverse_mass;
  const glm::vec3 impulse1 = correction.position1 / body1.inverse_mass;
  EXPECT_NEAR(glm::length(impulse0 + impulse1), 0.f, 1e-6f);
  EXPECT_GT(glm::abs(correction.angular0.z) + glm::abs(correction.angular1.z), 0.f);
}

TEST(DynamicStrandsBundle, OffCenterBoundaryErrorIntroducesCorrectRotation) {
  BundleRigidBodyState body0{{0.f, 0.f, 0.f}, {}, 1.f, glm::mat3(1.f)};
  BundleRigidBodyState body1{{2.f, 1.f, 0.f}, {}, 0.f, glm::mat3(0.f)};
  BundlePairReferenceState constraint;
  constraint.segment0_midpoint_offset = {1.f, 0.f, 0.f};
  constraint.segment1_midpoint_offset = {-1.f, 0.f, 0.f};

  const auto correction = SolveBundlePairReference(body0, body1, constraint);

  EXPECT_GT(correction.angular0.z, 0.f);
}

TEST(DynamicStrandsBundle, CoupledPairResidualDecreasesWithIterations) {
  const auto residual = [](const int iterations) {
    BundleRigidBodyState body0{{0.f, 0.f, 0.f}, {}, 1.f, glm::mat3(1.f)};
    BundleRigidBodyState body1{{2.f, 1.f, 0.f}, {}, 0.f, glm::mat3(0.f)};
    BundlePairReferenceState constraint;
    constraint.segment0_midpoint_offset = {1.f, 0.f, 0.f};
    constraint.segment1_midpoint_offset = {-1.f, 0.f, 0.f};
    for (int iteration = 0; iteration < iterations; ++iteration)
      ApplyBundlePairCorrection(body0, body1, SolveBundlePairReference(body0, body1, constraint));
    return glm::length(body0.position + body0.rotation * constraint.segment0_midpoint_offset - body1.position -
                       body1.rotation * constraint.segment1_midpoint_offset);
  };
  const float residual1 = residual(1);
  const float residual2 = residual(2);
  const float residual4 = residual(4);
  const float residual8 = residual(8);
  EXPECT_GT(residual1, residual8);
  EXPECT_GE(residual1, residual2);
  EXPECT_GE(residual2, residual4);
  EXPECT_GE(residual4 + 1e-6f, residual8);
}

TEST(DynamicStrandsBundle, BaseSlicesSeparateBranchesAndDistanceBins) {
  const std::vector<BundleSliceSegment> segments = {{0, .1f, 1.f}, {0, .9f, 1.f}, {0, 1.1f, 1.f}, {1, .1f, 1.f}};

  const auto slices = BuildBundleBaseSlices(segments, 1.f);

  EXPECT_EQ(slices[0], slices[1]);
  EXPECT_NE(slices[1], slices[2]);
  EXPECT_NE(slices[0], slices[3]);
}

TEST(DynamicStrandsBundle, SliceFitRecoversKnownRigidTransform) {
  const glm::quat expected = glm::angleAxis(glm::radians(35.f), glm::normalize(glm::vec3(1.f, 2.f, 3.f)));
  const glm::vec3 translation{2.f, -1.f, .5f};
  std::vector<BundleSlicePoint> points;
  for (const glm::vec3 rest : {glm::vec3(-1.f, 0.f, 0.f), glm::vec3(1.f, 0.f, 0.f), glm::vec3(0.f, -1.f, .2f),
                               glm::vec3(0.f, 1.f, -.3f), glm::vec3(.2f, -.1f, 1.f)})
    points.push_back({rest, expected * rest + translation, 1.f});

  const auto fit = FitBundleSliceReference(points, 4);
  glm::vec3 rest_center(0.f);
  for (const auto& point : points)
    rest_center += point.rest;
  rest_center /= static_cast<float>(points.size());

  EXPECT_TRUE(fit.valid);
  EXPECT_NEAR(glm::length(fit.center - (expected * rest_center + translation)), 0.f, 1e-5f);
  EXPECT_NEAR(glm::abs(glm::dot(fit.rotation, expected)), 1.f, 1e-5f);
}

TEST(DynamicStrandsBundle, SliceFitRejectsSmallAndCollinearComponents) {
  EXPECT_FALSE(FitBundleSliceReference({{{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, 1.f}}, 2).valid);
  EXPECT_FALSE(FitBundleSliceReference({{{-1.f, 0.f, 0.f}, {-1.f, 1.f, 0.f}, 1.f},
                                        {{0.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, 1.f},
                                        {{1.f, 0.f, 0.f}, {1.f, 1.f, 0.f}, 1.f}},
                                       3)
                   .valid);
}
