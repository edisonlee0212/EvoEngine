#include "EvoEngine_SDK_PCH.hpp"

#ifdef min
#  undef min
#endif
#ifdef max
#  undef max
#endif

#include <gtest/gtest.h>

#include "DistributionDefaults.hpp"
#include "ElasticaSolver.hpp"
#include "GeometryPass.hpp"
#include "SorghumLeafMesh.hpp"
#include "SorghumModules.hpp"
#include "SorghumRules.hpp"

#include <array>
#include <cmath>
#include <map>
#include <set>

#include <glm/gtx/quaternion.hpp>

using namespace l_system_package;

namespace {

void SetConstantPlot(evo_engine::PlottedDistribution<float>& distribution, const float value) {
  distribution.mean.min_value = value;
  distribution.mean.max_value = value;
  distribution.mean.curve.UnsafeGetValues() = {{0.0f, 0.5f}, {1.0f, 0.5f}};
  distribution.deviation.min_value = 0.0f;
  distribution.deviation.max_value = 0.0f;
  distribution.deviation.curve.UnsafeGetValues() = {{0.0f, 0.0f}, {1.0f, 0.0f}};
}

SampledSorghumParams MakeMechanicalLeafParams(const float compliance) {
  SampledSorghumParams params;
  ApplyLinearGrowthCurveDefaultsToAll(params.width_along_sheath, params.width_along_neck, params.width_along_leaf,
                                      params.curling_along_leaf, params.waviness_along_leaf, params.bending_along_leaf);
  params.width_along_leaf.mean.curve.UnsafeGetValues() = {{0.0f, 0.35f}, {0.5f, 1.0f}, {1.0f, 0.05f}};
  params.bending_along_leaf.mean.curve.UnsafeGetValues() = {{0.0f, 0.0f}, {0.45f, 0.05f}, {0.7f, 0.4f}, {1.0f, 1.0f}};
  params.maturity_gdd = 100.0f;
  params.leaf_gravity_droop_compliance = compliance;
  params.leaf_gravity_droop_age_response.mean.min_value = 0.0f;
  params.leaf_gravity_droop_age_response.mean.max_value = 1.0f;
  params.leaf_gravity_droop_age_response.mean.curve.SetTangent(false);
  params.leaf_gravity_droop_age_response.mean.curve.UnsafeGetValues() = {{0.0f, 0.0f}, {1.0f, 1.0f}};
  params.leaf_gravity_droop_age_response.deviation.min_value = 0.0f;
  params.leaf_gravity_droop_age_response.deviation.max_value = 0.0f;
  params.leaf_gravity_droop_age_response.deviation.curve.UnsafeGetValues() = {{0.0f, 0.0f}, {1.0f, 0.0f}};
  SetConstantPlot(params.leaf_flexural_stiffness_along_leaf, 1.0f);
  SetConstantPlot(params.leaf_waviness_width_fraction, 0.0f);
  return params;
}

SorghumLeaf MakeMechanicalLeaf() {
  SorghumLeaf leaf;
  leaf.blade_length = leaf.target_blade_length = 1.0f;
  leaf.blade_max_width = leaf.target_blade_max_width = 0.10f;
  leaf.blade_thickness = leaf.target_blade_thickness = 0.0005f;
  leaf.neck_length = leaf.target_neck_length = 0.0f;
  leaf.sheath_length = leaf.target_sheath_length = 0.0f;
  leaf.blade_end_width_ratio = leaf.target_blade_end_width_ratio = 0.05f;
  leaf.insertion_angle_deg = leaf.target_insertion_angle_deg = 40.0f;
  leaf.bending = leaf.target_bending = 25.0f;
  leaf.curling = leaf.target_curling = 89.0f;
  leaf.age_gdd = 100.0f;
  leaf.axis_phytomer_count = 10;
  leaf.node_random = 0.5f;
  return leaf;
}

StemContext MakeVerticalStem() {
  StemContext stem;
  StemContext::Segment base;
  stem.segments.push_back(base);
  auto tip = base;
  tip.position = glm::vec3(0.0f, 1.0f, 0.0f);
  stem.segments.push_back(tip);
  return stem;
}

SorghumSpline BuildMechanicalLeaf(const SorghumLeaf& leaf, const SampledSorghumParams& params,
                                  const StemContext& stem = MakeVerticalStem(),
                                  const glm::vec3& gravity_local_m_s2 = glm::vec3(0.0f, -9.80665f, 0.0f)) {
  SorghumLeafMeshSettings settings;
  settings.vertical_subdivision_length = 0.02f;
  settings.gravity_local_m_s2 = gravity_local_m_s2;
  SorghumSpline spline;
  BuildLeafSplineFromState(leaf, stem, params, settings, spline);
  return spline;
}

float SignedFrameRollDegrees(const SorghumSplineSegment& reference, const SorghumSplineSegment& rolled) {
  const glm::vec3 axis = glm::normalize(rolled.front);
  const glm::vec3 reference_up = glm::normalize(reference.up - glm::dot(reference.up, axis) * axis);
  const glm::vec3 rolled_up = glm::normalize(rolled.up - glm::dot(rolled.up, axis) * axis);
  return glm::degrees(
      std::atan2(glm::dot(axis, glm::cross(reference_up, rolled_up)), glm::dot(reference_up, rolled_up)));
}

}  // namespace

TEST(LSystemLeafAtlasUv, DefaultLayoutPreservesLocalUv) {
  const SorghumLeafAtlasLayout layout;
  const glm::vec2 uv(0.25f, 0.75f);

  const auto remapped = RemapSorghumLeafAtlasUv(uv, layout);

  EXPECT_FLOAT_EQ(remapped.x, uv.x);
  EXPECT_FLOAT_EQ(remapped.y, uv.y);
}

TEST(LSystemLeafAtlasUv, MapsFirstVariantToTopLeftTile) {
  SorghumLeafAtlasLayout layout;
  layout.variant_columns = 3u;
  layout.variant_rows = 3u;
  layout.variant_count = 9u;
  layout.variant_index = 0u;
  layout.tile_uv_inset = 0.001f;

  const auto lower_left = RemapSorghumLeafAtlasUv(glm::vec2(0.0f, 0.0f), layout);
  const auto upper_right = RemapSorghumLeafAtlasUv(glm::vec2(1.0f, 1.0f), layout);

  EXPECT_NEAR(lower_left.x, 0.001f, 1.0e-6f);
  EXPECT_NEAR(lower_left.y, 2.0f / 3.0f + 0.001f, 1.0e-6f);
  EXPECT_NEAR(upper_right.x, 1.0f / 3.0f - 0.001f, 1.0e-6f);
  EXPECT_NEAR(upper_right.y, 1.0f - 0.001f, 1.0e-6f);
}

TEST(LSystemLeafAtlasUv, MapsLastVariantToBottomRightTile) {
  SorghumLeafAtlasLayout layout;
  layout.variant_columns = 3u;
  layout.variant_rows = 3u;
  layout.variant_count = 9u;
  layout.variant_index = 8u;
  layout.tile_uv_inset = 0.001f;

  const auto lower_left = RemapSorghumLeafAtlasUv(glm::vec2(0.0f, 0.0f), layout);
  const auto upper_right = RemapSorghumLeafAtlasUv(glm::vec2(1.0f, 1.0f), layout);

  EXPECT_NEAR(lower_left.x, 2.0f / 3.0f + 0.001f, 1.0e-6f);
  EXPECT_NEAR(lower_left.y, 0.001f, 1.0e-6f);
  EXPECT_NEAR(upper_right.x, 1.0f - 0.001f, 1.0e-6f);
  EXPECT_NEAR(upper_right.y, 1.0f / 3.0f - 0.001f, 1.0e-6f);
}

TEST(LSystemLeafAtlasUv, SingleVariantAlwaysSelectsFirstTile) {
  EXPECT_EQ(ComputeSorghumLeafAtlasVariant(37u, 12u, 0.0f, 1u), 0u);
  EXPECT_EQ(ComputeSorghumLeafAtlasVariant(37u, 12u, 1.0f, 1u), 0u);
}

TEST(LSystemLeafAtlasUv, LeafRandomChangesVariantSelection) {
  std::array<bool, 9> seen{};
  for (uint32_t i = 0; i < 100u; ++i) {
    const float leaf_random = (static_cast<float>(i) + 0.5f) / 100.0f;
    seen[ComputeSorghumLeafAtlasVariant(37u, 12u, leaf_random, 9u)] = true;
  }

  size_t seen_count = 0;
  for (const bool selected : seen) {
    if (selected) {
      ++seen_count;
    }
  }
  EXPECT_GT(seen_count, 1u);
}

TEST(LSystemLeafAtlasUv, ThreeByThreeAtlasVariantsAreReachablePerLeaf) {
  std::array<bool, 9> seen{};
  for (uint32_t i = 0; i < 256u; ++i) {
    const float leaf_random = (static_cast<float>((i * 37u) % 256u) + 0.5f) / 256.0f;
    seen[ComputeSorghumLeafAtlasVariant(1234u, i, leaf_random, 9u)] = true;
  }

  for (const bool selected : seen) {
    EXPECT_TRUE(selected);
  }
}

TEST(LSystemLeafMorphology, SpatialBendingProfileIsIndependentFromTemporalDevelopment) {
  SampledSorghumParams params;
  ApplyLinearGrowthCurveDefaultsToAll(params.width_along_sheath, params.width_along_neck, params.width_along_leaf,
                                      params.curling_along_leaf, params.waviness_along_leaf,
                                      params.leaf_bending_development_curve, params.bending_along_leaf);

  SorghumLeaf leaf;
  leaf.blade_length = leaf.target_blade_length = 1.0f;
  leaf.neck_length = leaf.target_neck_length = 0.0f;
  leaf.sheath_length = leaf.target_sheath_length = 0.0f;
  leaf.sheath_end_width_ratio = leaf.target_sheath_end_width_ratio = 1.0f;
  leaf.neck_end_width_ratio = leaf.target_neck_end_width_ratio = 1.0f;
  leaf.blade_end_width_ratio = leaf.target_blade_end_width_ratio = 0.05f;
  leaf.bending = leaf.target_bending = 60.0f;
  leaf.curling = leaf.target_curling = 89.0f;
  leaf.insertion_angle_deg = leaf.target_insertion_angle_deg = 0.0f;
  leaf.node_random = 0.5f;

  StemContext stem;
  stem.segments.emplace_back();
  SorghumLeafMeshSettings settings;
  settings.vertical_subdivision_length = 0.05f;

  SorghumSpline first;
  BuildLeafSplineFromState(leaf, stem, params, settings, first);
  ASSERT_FALSE(first.segments.empty());
  const glm::vec3 first_tip = first.segments.back().position;

  params.leaf_bending_development_curve.mean.curve.UnsafeGetValues() = {{0.0f, 0.0f}, {1.0f, 0.0f}};
  SorghumSpline temporal_changed;
  BuildLeafSplineFromState(leaf, stem, params, settings, temporal_changed);
  ASSERT_FALSE(temporal_changed.segments.empty());
  const glm::vec3 temporal_tip = temporal_changed.segments.back().position;
  EXPECT_NEAR(glm::distance(first_tip, temporal_tip), 0.0f, 1.0e-6f);

  params.bending_along_leaf.mean.curve.UnsafeGetValues() = {{0.0f, 0.0f}, {1.0f, 0.0f}};
  SorghumSpline spatial_changed;
  BuildLeafSplineFromState(leaf, stem, params, settings, spatial_changed);
  ASSERT_FALSE(spatial_changed.segments.empty());
  EXPECT_GT(glm::distance(first_tip, spatial_changed.segments.back().position), 0.1f);
}

TEST(LSystemLeafMorphology, DistalBendCanDoubleAtTheTipWithoutChangingTheFirstHalf) {
  auto baseline_params = MakeMechanicalLeafParams(0.0f);
  baseline_params.bending_along_leaf.mean.curve.UnsafeGetValues() = {{0.0f, 0.0f},  {0.3f, 0.0f},   {0.5f, 0.08f},
                                                                     {0.7f, 0.35f}, {0.88f, 0.75f}, {1.0f, 1.0f}};
  auto candidate_params = baseline_params;
  candidate_params.bending_along_leaf.mean.curve.UnsafeGetValues() = {{0.0f, 0.0f},   {0.3f, 0.0f},   {0.5f, 0.04f},
                                                                      {0.7f, 0.245f}, {0.88f, 0.66f}, {1.0f, 1.0f}};

  auto baseline_leaf = MakeMechanicalLeaf();
  auto candidate_leaf = baseline_leaf;
  candidate_leaf.bending = candidate_leaf.target_bending = 2.0f * baseline_leaf.bending;
  const auto baseline = BuildMechanicalLeaf(baseline_leaf, baseline_params);
  const auto candidate = BuildMechanicalLeaf(candidate_leaf, candidate_params);
  ASSERT_EQ(baseline.segments.size(), candidate.segments.size());

  const size_t halfway = baseline.segments.size() / 2;
  EXPECT_NEAR(glm::distance(baseline.segments[halfway].position, candidate.segments[halfway].position), 0.0f, 2.0e-3f);
  EXPECT_NEAR(glm::degrees(std::acos(std::clamp(
                  glm::dot(baseline.segments[halfway].front, candidate.segments[halfway].front), -1.0f, 1.0f))),
              0.0f, 0.5f);
  EXPECT_GT(glm::degrees(std::acos(
                std::clamp(glm::dot(baseline.segments.back().front, candidate.segments.back().front), -1.0f, 1.0f))),
            20.0f);
}

TEST(LSystemLeafMorphology, AbsoluteBladeWidthIsIndependentOfHostCulmRadius) {
  SampledSorghumParams params;
  ApplyLinearGrowthCurveDefaultsToAll(params.width_along_sheath, params.width_along_neck, params.width_along_leaf,
                                      params.curling_along_leaf, params.waviness_along_leaf, params.bending_along_leaf);
  params.width_along_leaf.mean.curve.UnsafeGetValues() = {{0.0f, 0.35f}, {0.5f, 1.0f}, {1.0f, 0.0f}};
  params.leaf_sheath_radius_ratio = 1.05f;
  params.leaf_sheath_wrap_angle = 390.0f;

  SorghumLeaf leaf;
  leaf.blade_length = leaf.target_blade_length = 1.0f;
  leaf.blade_max_width = leaf.target_blade_max_width = 0.12f;
  leaf.neck_length = leaf.target_neck_length = 0.1f;
  leaf.sheath_length = leaf.target_sheath_length = 0.2f;
  leaf.curling = leaf.target_curling = 89.0f;
  leaf.node_random = 0.5f;

  SorghumLeafMeshSettings settings;
  settings.vertical_subdivision_length = 0.01f;
  const auto distal_max_radius = [&](const float culm_radius) {
    StemContext stem;
    StemContext::Segment base;
    base.radius = culm_radius;
    stem.segments.push_back(base);
    auto tip = base;
    tip.position = glm::vec3(0.0f, 1.0f, 0.0f);
    stem.segments.push_back(tip);
    SorghumSpline spline;
    BuildLeafSplineFromState(leaf, stem, params, settings, spline);
    float maximum = 0.0f;
    for (const auto& segment : spline.segments) {
      if (segment.theta <= 90.0f)
        maximum = std::max(maximum, segment.radius);
    }
    return maximum;
  };

  const float narrow_culm_width = distal_max_radius(0.005f);
  const float wide_culm_width = distal_max_radius(0.015f);
  EXPECT_NEAR(narrow_culm_width, wide_culm_width, 1.0e-6f);
  EXPECT_NEAR(narrow_culm_width, 0.06f, 5.0e-4f);
}

TEST(LSystemLeafMorphology, ElasticaMatchesSmallDeflectionCantilever) {
  const auto result = RunCantileverValidation(0.4f, 0.02f, 0.001f, 65);
  EXPECT_TRUE(result.converged);
  EXPECT_LT(result.relative_error, 0.01f);
}

TEST(LSystemLeafMorphology, DistributedGravityConvergesAtSorghumScale) {
  PlanarElasticaInput input;
  input.length_m = 1.0f;
  input.station_count = 33;
  input.intrinsic_curvature_per_m = [](const float u) {
    return glm::radians(25.0f) * 6.0f * u * (1.0f - u);
  };
  input.bending_stiffness_Pa_m4 = [](const float u) {
    return 0.025f * std::max(0.18f, 1.0f - 0.82f * u);
  };
  input.mass_per_length_kg_m = [](const float u) {
    return 0.12f * 0.126f * std::sin(glm::pi<float>() * u);
  };
  input.gravity_acceleration_xz_m_s2 = glm::vec2(9.80665f, 0.0f);
  input.relaxation = 0.35f;
  input.tolerance_rad = 1.0e-5f;
  input.max_iterations = 64;

  const auto output = SolvePlanarElastica(input);

  EXPECT_TRUE(output.converged);
  EXPECT_TRUE(std::isfinite(output.positions_xz.back().x));
  EXPECT_LT(output.positions_xz.back().x, input.length_m);
}

TEST(LSystemLeafMorphology, ZeroCompliancePreservesIntrinsicSpline) {
  const auto leaf = MakeMechanicalLeaf();
  const auto baseline = BuildMechanicalLeaf(leaf, MakeMechanicalLeafParams(0.0f));

  auto young = leaf;
  young.age_gdd = 0.0f;
  const auto age_disabled = BuildMechanicalLeaf(young, MakeMechanicalLeafParams(1.0f));

  ASSERT_EQ(baseline.segments.size(), age_disabled.segments.size());
  for (size_t i = 0; i < baseline.segments.size(); ++i) {
    EXPECT_NEAR(glm::distance(baseline.segments[i].position, age_disabled.segments[i].position), 0.0f, 1.0e-6f);
    EXPECT_NEAR(glm::distance(baseline.segments[i].front, age_disabled.segments[i].front), 0.0f, 1.0e-6f);
  }
}

TEST(LSystemLeafMorphology, SagIncreasesWithComplianceAndThermalAge) {
  const auto leaf = MakeMechanicalLeaf();
  const auto baseline = BuildMechanicalLeaf(leaf, MakeMechanicalLeafParams(0.0f));
  const auto medium = BuildMechanicalLeaf(leaf, MakeMechanicalLeafParams(0.5f));
  const auto high = BuildMechanicalLeaf(leaf, MakeMechanicalLeafParams(1.0f));
  ASSERT_FALSE(baseline.segments.empty());
  ASSERT_EQ(baseline.segments.size(), medium.segments.size());
  ASSERT_EQ(baseline.segments.size(), high.segments.size());

  const float medium_sag = baseline.segments.back().position.y - medium.segments.back().position.y;
  const float high_sag = baseline.segments.back().position.y - high.segments.back().position.y;
  EXPECT_GT(medium_sag, 0.0f);
  EXPECT_GT(high_sag, medium_sag);

  auto young = leaf;
  young.age_gdd = 25.0f;
  const auto young_spline = BuildMechanicalLeaf(young, MakeMechanicalLeafParams(1.0f));
  const float young_sag = baseline.segments.back().position.y - young_spline.segments.back().position.y;
  EXPECT_LT(young_sag, high_sag);
}

TEST(LSystemLeafMorphology, ReducedComplianceProducesApproximatelyOneThirdOfGravityOnlyTipDeflection) {
  const auto leaf = MakeMechanicalLeaf();
  const auto baseline = BuildMechanicalLeaf(leaf, MakeMechanicalLeafParams(0.0f));
  const auto reduced = BuildMechanicalLeaf(leaf, MakeMechanicalLeafParams(0.045f));
  const auto full = BuildMechanicalLeaf(leaf, MakeMechanicalLeafParams(1.0f));
  const float reduced_sag = baseline.segments.back().position.y - reduced.segments.back().position.y;
  const float full_sag = baseline.segments.back().position.y - full.segments.back().position.y;
  ASSERT_GT(full_sag, 0.0f);
  EXPECT_NEAR(reduced_sag / full_sag, 1.0f / 3.0f, 0.08f);
}

TEST(LSystemLeafMorphology, StaticWindDeformationIsDeterministicThreeDimensionalAndBaseAnchored) {
  const auto leaf = MakeMechanicalLeaf();
  auto baseline_params = MakeMechanicalLeafParams(0.0f);
  const auto baseline = BuildMechanicalLeaf(leaf, baseline_params);
  baseline_params.leaf_static_wind_deflection_fraction = 0.02f;
  baseline_params.leaf_centerline_waviness_fraction = 0.01f;
  baseline_params.leaf_static_wind_azimuth_degrees = 0.0f;
  auto deformed_leaf = leaf;
  deformed_leaf.axial_twist_amplitude_deg = 10.0f;
  deformed_leaf.axial_twist_frequency_ratio = 0.5f;
  deformed_leaf.axial_twist_phase_rad = 0.3f;
  const auto first = BuildMechanicalLeaf(deformed_leaf, baseline_params);
  const auto second = BuildMechanicalLeaf(deformed_leaf, baseline_params);
  ASSERT_EQ(first.segments.size(), baseline.segments.size());
  ASSERT_EQ(first.segments.size(), second.segments.size());

  size_t blade_start = 0;
  while (blade_start < first.segments.size() && first.segments[blade_start].theta > 90.0f)
    ++blade_start;
  ASSERT_LT(blade_start, first.segments.size());
  EXPECT_NEAR(glm::distance(first.segments[blade_start].position, baseline.segments[blade_start].position), 0.0f,
              1.0e-6f);
  float maximum_out_of_plane = 0.0f;
  for (size_t i = blade_start; i < first.segments.size(); ++i) {
    EXPECT_NEAR(glm::distance(first.segments[i].position, second.segments[i].position), 0.0f, 1.0e-6f);
    maximum_out_of_plane = std::max(maximum_out_of_plane, std::abs(first.segments[i].position.x));
  }
  EXPECT_GT(maximum_out_of_plane, 0.005f);
  EXPECT_GT(glm::distance(first.segments.back().up, baseline.segments.back().up), 0.01f);
}

TEST(LSystemLeafMorphology, AxialTwistRollsTheLocalFrameWithoutMovingTheCenterline) {
  auto params = MakeMechanicalLeafParams(0.0f);
  params.leaf_waviness_wavelength_m = 0.16f;
  params.leaf_centerline_waviness_fraction = 2.0e-6f;
  auto reference_leaf = MakeMechanicalLeaf();
  reference_leaf.axial_twist_amplitude_deg = 0.0f;
  auto rolled_leaf = reference_leaf;
  rolled_leaf.axial_twist_amplitude_deg = 30.0f;
  rolled_leaf.axial_twist_frequency_ratio = 0.5f;
  rolled_leaf.axial_twist_phase_rad = 0.0f;

  const auto reference = BuildMechanicalLeaf(reference_leaf, params);
  const auto rolled = BuildMechanicalLeaf(rolled_leaf, params);
  ASSERT_EQ(reference.segments.size(), rolled.segments.size());

  float maximum_roll = 0.0f;
  int sign_changes = 0;
  int previous_sign = 0;
  for (size_t i = 0; i < rolled.segments.size(); ++i) {
    EXPECT_NEAR(glm::distance(reference.segments[i].position, rolled.segments[i].position), 0.0f, 1.0e-6f);
    const float roll = SignedFrameRollDegrees(reference.segments[i], rolled.segments[i]);
    maximum_roll = std::max(maximum_roll, std::abs(roll));
    if (std::abs(roll) < 1.0f)
      continue;
    const int sign = roll > 0.0f ? 1 : -1;
    if (previous_sign != 0 && sign != previous_sign)
      ++sign_changes;
    previous_sign = sign;
  }

  EXPECT_NEAR(SignedFrameRollDegrees(reference.segments.front(), rolled.segments.front()), 0.0f, 1.0e-4f);
  EXPECT_GT(maximum_roll, 27.0f);
  EXPECT_LE(maximum_roll, 30.1f);
  EXPECT_GE(sign_changes, 5);
  EXPECT_LE(sign_changes, 7);
}

TEST(LSystemLeafMorphology, OwnerRotationPreservesWorldGravityResponse) {
  const auto leaf = MakeMechanicalLeaf();
  const auto params = MakeMechanicalLeafParams(1.0f);
  const auto stem = MakeVerticalStem();
  const glm::vec3 world_gravity(0.0f, -9.80665f, 0.0f);

  const glm::quat rotation = glm::angleAxis(glm::radians(47.0f), glm::normalize(glm::vec3(1.0f, 0.3f, 0.5f)));
  StemContext rotated_stem = stem;
  for (auto& segment : rotated_stem.segments) {
    segment.position = rotation * segment.position;
    segment.front = rotation * segment.front;
    segment.up = rotation * segment.up;
  }
  const auto world_space = BuildMechanicalLeaf(leaf, params, rotated_stem, world_gravity);
  const auto owner_local = BuildMechanicalLeaf(leaf, params, stem, glm::conjugate(rotation) * world_gravity);
  ASSERT_EQ(world_space.segments.size(), owner_local.segments.size());
  for (size_t i = 0; i < world_space.segments.size(); ++i) {
    EXPECT_NEAR(glm::distance(rotation * owner_local.segments[i].position, world_space.segments[i].position), 0.0f,
                2.0e-5f);
    EXPECT_NEAR(glm::distance(rotation * owner_local.segments[i].front, world_space.segments[i].front), 0.0f, 2.0e-5f);
  }
}

TEST(LSystemLeafMorphology, WidthRelativeWavinessPreservesNormalizedAmplitude) {
  auto params = MakeMechanicalLeafParams(0.0f);
  SetConstantPlot(params.leaf_waviness_width_fraction, 0.12f);
  params.leaf_waviness_wavelength_m = 0.16f;

  auto narrow_leaf = MakeMechanicalLeaf();
  narrow_leaf.blade_max_width = narrow_leaf.target_blade_max_width = 0.04f;
  auto wide_leaf = narrow_leaf;
  wide_leaf.blade_max_width = wide_leaf.target_blade_max_width = 0.12f;
  const auto narrow = BuildMechanicalLeaf(narrow_leaf, params);
  const auto wide = BuildMechanicalLeaf(wide_leaf, params);
  ASSERT_EQ(narrow.segments.size(), wide.segments.size());

  float narrow_ratio_sq = 0.0f;
  float wide_ratio_sq = 0.0f;
  float narrow_offset_sq = 0.0f;
  float wide_offset_sq = 0.0f;
  int count = 0;
  for (size_t i = 0; i < narrow.segments.size(); ++i) {
    if (narrow.segments[i].radius < 0.005f || wide.segments[i].radius < 0.015f)
      continue;
    narrow_ratio_sq += glm::pow(narrow.segments[i].left_height_offset / narrow.segments[i].radius, 2.0f);
    wide_ratio_sq += glm::pow(wide.segments[i].left_height_offset / wide.segments[i].radius, 2.0f);
    narrow_offset_sq += glm::pow(narrow.segments[i].left_height_offset, 2.0f);
    wide_offset_sq += glm::pow(wide.segments[i].left_height_offset, 2.0f);
    ++count;
  }
  ASSERT_GT(count, 0);
  EXPECT_NEAR(std::sqrt(narrow_ratio_sq / count), std::sqrt(wide_ratio_sq / count), 1.0e-5f);
  EXPECT_NEAR(std::sqrt(wide_offset_sq / narrow_offset_sq), 3.0f, 0.02f);
}

TEST(LSystemLeafMorphology, SheathCrossSectionUsesConfiguredEllipseRatio) {
  const SorghumSplineSegment segment(glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f), 0.02f,
                                     195.0f, 0.0f, 0.0f, 1.4f);
  const float lateral_span = glm::distance(segment.GetLeafPoint(-90.0f), segment.GetLeafPoint(90.0f));
  const float radial_span = glm::distance(segment.GetLeafPoint(0.0f), segment.GetLeafPoint(180.0f));
  EXPECT_NEAR(lateral_span / radial_span, 1.4f, 1.0e-5f);
}

TEST(LSystemLeafMorphology, MainCulmLeanIsAppliedOnlyAtTheBasalInternode) {
  SampledSorghumParams params;
  params.total_phytomer_count = 2;
  params.main_culm_lean_angle = 6.8f;
  params.branch_azimuth_offset = 37.0f;
  params.plastochron_gdd = 1.0f;
  params.leaf_axial_twist_max_degrees = 30.0f;
  params.leaf_axial_twist_frequency_ratio_min = 0.35f;
  params.leaf_axial_twist_frequency_ratio_max = 0.5f;
  ApplyMeanStdPlotDefaultsToAll(params.internode_length, params.internode_thickness, params.leaf_blade_length,
                                params.leaf_blade_max_width, params.leaf_sheath_length, params.leaf_neck_length,
                                params.leaf_sheath_end_width_ratio, params.leaf_neck_end_width_ratio,
                                params.leaf_blade_end_width_ratio, params.leaf_insertion_angle, params.leaf_roll_angle,
                                params.leaf_curling, params.leaf_bending, params.leaf_waviness,
                                params.tiller_leaf_area_ratio_by_origin);

  SorghumGraph graph(1);
  graph.RefNode(0).symbol_id = SorghumSymbol::Root;
  graph.RefNode(0).data.Set<SorghumRoot>(SorghumRoot{});
  const auto apex_handle = graph.Extend(0, false);
  SorghumApex apex;
  apex.vigor = 2;
  apex.sampled_plastochron_gdd = 1.0f;
  apex.age_gdd = 2.0f;
  apex.axis_phytomer_count = 2;
  graph.RefNode(apex_handle).symbol_id = SorghumSymbol::Apex;
  graph.RefNode(apex_handle).data.Set<SorghumApex>(apex);
  graph.SortLists();

  SorghumEngine engine;
  engine.topology_rules = CreateSorghumTopologyRules(params);
  std::mt19937 rng(17u);
  for (int step = 0; step < 2; ++step) {
    engine.ApplyTopologyRules(graph, rng);
    graph.SortLists();
  }

  std::vector<SorghumInternode> internodes;
  std::vector<SorghumLeaf> leaves;
  for (const auto handle : graph.PeekSortedNodeList()) {
    const auto& node = graph.PeekNode(handle);
    if (node.data.Is<SorghumInternode>())
      internodes.push_back(node.data.Get<SorghumInternode>());
    if (node.data.Is<SorghumLeaf>())
      leaves.push_back(node.data.Get<SorghumLeaf>());
  }
  ASSERT_EQ(internodes.size(), 2u);
  ASSERT_EQ(leaves.size(), 2u);
  EXPECT_FLOAT_EQ(internodes[0].branch_angle, 6.8f);
  EXPECT_FLOAT_EQ(internodes[0].roll_angle, 37.0f);
  EXPECT_FLOAT_EQ(internodes[1].branch_angle, 0.0f);
  EXPECT_FLOAT_EQ(internodes[1].roll_angle, 0.0f);
  for (const auto& leaf : leaves) {
    EXPECT_GE(leaf.axial_twist_amplitude_deg, 0.0f);
    EXPECT_LE(leaf.axial_twist_amplitude_deg, 30.0f);
    EXPECT_GE(leaf.axial_twist_frequency_ratio, 0.35f);
    EXPECT_LE(leaf.axial_twist_frequency_ratio, 0.5f);
    EXPECT_GE(leaf.axial_twist_phase_rad, 0.0f);
    EXPECT_LE(leaf.axial_twist_phase_rad, glm::two_pi<float>());
  }
  EXPECT_NE(leaves[0].axial_twist_amplitude_deg, leaves[1].axial_twist_amplitude_deg);
}

TEST(LSystemLeafMorphology, FourPlastochronPostureOnsetKeepsYoungestCohortUnbent) {
  SampledSorghumParams params;
  params.gdd_step = 1.0f;
  params.maturity_gdd = 240.0f;
  params.leaf_bending_development_curve.mean.min_value = 0.0f;
  params.leaf_bending_development_curve.mean.max_value = 1.0f;
  params.leaf_bending_development_curve.mean.curve.SetTangent(false);
  params.leaf_bending_development_curve.mean.curve.UnsafeGetValues() = {
      {0.0f, 0.0f}, {0.45f, 0.0f}, {0.65f, 0.15f}, {0.82f, 0.55f}, {1.0f, 1.0f}};
  params.leaf_bending_development_curve.deviation.min_value = 0.0f;
  params.leaf_bending_development_curve.deviation.max_value = 0.0f;
  params.leaf_bending_development_curve.deviation.curve.SetTangent(false);
  params.leaf_bending_development_curve.deviation.curve.UnsafeGetValues() = {{0.0f, 0.0f}, {1.0f, 0.0f}};

  SorghumGraph graph(1);
  graph.RefNode(0).symbol_id = SorghumSymbol::Root;
  graph.RefNode(0).data.Set<SorghumRoot>(SorghumRoot{});
  for (int rank = 0; rank < 5; ++rank) {
    const auto leaf_handle = graph.Extend(0, true);
    SorghumLeaf leaf;
    leaf.rank = rank;
    leaf.age_gdd = static_cast<float>(rank * 30);
    leaf.target_bending = 60.0f;
    leaf.node_random = 0.5f;
    graph.RefNode(leaf_handle).symbol_id = SorghumSymbol::Leaf;
    graph.RefNode(leaf_handle).data.Set<SorghumLeaf>(leaf);
  }
  graph.SortLists();

  SorghumEngine engine;
  engine.growth_rules = CreateSorghumGrowthRules(params);
  std::mt19937 rng(17u);
  engine.ApplyGrowthRules(graph, rng);

  for (const auto handle : graph.PeekSortedNodeList()) {
    const auto& node = graph.PeekNode(handle);
    if (!node.data.Is<SorghumLeaf>())
      continue;
    const auto& leaf = node.data.Get<SorghumLeaf>();
    if (leaf.rank < 4)
      EXPECT_FLOAT_EQ(leaf.bending, 0.0f);
    else
      EXPECT_GT(leaf.bending, 0.0f);
  }
}

TEST(LSystemSorghumTillers, MatureGraphHasThreeCrownAttachedPrimaryTillers) {
  SampledSorghumParams params;
  params.total_phytomer_count = 10;
  params.phyllotaxis_angle = 180.0f;
  params.tiller_count = 3;
  params.tiller_origin_ranks = {3, 4, 2};
  params.tiller_emergence_main_leaf_stages = {1, 1, 1, 1, 1, 1};
  params.tiller_leaf_count_ratio = {0.90f, 0.0f};
  params.tiller_height_ratio = {0.90f, 0.0f};
  params.plastochron_gdd = 1.0f;
  params.maturity_gdd = 1.0f;
  params.gdd_step = 1.0f;
  ApplyMeanStdPlotDefaultsToAll(params.internode_length, params.internode_thickness, params.leaf_blade_length,
                                params.leaf_blade_max_width, params.leaf_sheath_length, params.leaf_neck_length,
                                params.leaf_sheath_end_width_ratio, params.leaf_neck_end_width_ratio,
                                params.leaf_blade_end_width_ratio, params.leaf_insertion_angle, params.leaf_roll_angle,
                                params.leaf_curling, params.leaf_bending, params.leaf_waviness,
                                params.tiller_leaf_area_ratio_by_origin);
  ApplyLinearGrowthCurveDefaultsToAll(
      params.internode_elongation_curve, params.internode_thickness_curve, params.leaf_sheath_length_growth_curve,
      params.leaf_neck_length_growth_curve, params.leaf_blade_growth_curve, params.leaf_sheath_width_growth_curve,
      params.leaf_neck_width_growth_curve, params.leaf_width_growth_curve, params.leaf_angle_development_curve,
      params.leaf_curling_development_curve, params.leaf_bending_development_curve, params.bending_along_leaf);

  SorghumGraph graph(1);
  graph.RefNode(0).symbol_id = SorghumSymbol::Root;
  graph.RefNode(0).data.Set<SorghumRoot>(SorghumRoot{});
  const auto apex_handle = graph.Extend(0, false);
  SorghumApex apex;
  apex.vigor = params.total_phytomer_count;
  apex.sampled_plastochron_gdd = 1.0f;
  apex.age_gdd = 1.0f;
  apex.reference_main_phytomer_count = params.total_phytomer_count;
  apex.axis_phytomer_count = params.total_phytomer_count;
  graph.RefNode(apex_handle).symbol_id = SorghumSymbol::Apex;
  graph.RefNode(apex_handle).data.Set<SorghumApex>(apex);
  graph.data.main_expanded_leaf_count = params.total_phytomer_count;
  graph.SortLists();

  SorghumEngine engine;
  engine.topology_rules = CreateSorghumTopologyRules(params);
  engine.growth_rules = CreateSorghumGrowthRules(params);
  std::mt19937 rng(17u);
  for (int step = 0; step < 64; ++step) {
    engine.ApplyGrowthRules(graph, rng);
    engine.ApplyTopologyRules(graph, rng);
    graph.SortLists();
  }

  std::set<int> tiller_axes;
  std::map<int, int> leaves_per_axis;
  std::map<int, float> first_roll_per_axis;
  int main_leaves = 0;
  for (const auto handle : graph.PeekSortedNodeList()) {
    const auto& node = graph.PeekNode(handle);
    if (node.data.Is<SorghumLeaf>()) {
      const auto& leaf = node.data.Get<SorghumLeaf>();
      if (leaf.order == 0) {
        ++main_leaves;
      } else {
        ++leaves_per_axis[leaf.axis_id];
      }
    }
    if (!node.data.Is<SorghumInternode>()) {
      continue;
    }
    const auto& internode = node.data.Get<SorghumInternode>();
    if (internode.order != 1) {
      continue;
    }
    EXPECT_EQ(internode.axis_phytomer_count, 9);
    EXPECT_EQ(internode.origin_rank, params.tiller_origin_ranks[static_cast<size_t>(internode.axis_id - 1)]);
    tiller_axes.insert(internode.axis_id);
    if (internode.rank != 0) {
      continue;
    }
    first_roll_per_axis[internode.axis_id] = internode.roll_angle;
    const auto& parent = graph.PeekNode(node.GetParentHandle());
    ASSERT_TRUE(parent.data.Is<SorghumRoot>());
  }

  EXPECT_EQ(main_leaves, 10);
  EXPECT_EQ(tiller_axes, (std::set<int>{1, 2, 3}));
  EXPECT_NEAR(first_roll_per_axis[1], 0.0f, 1.0e-5f);
  EXPECT_NEAR(first_roll_per_axis[2], 174.0f, 1.0e-5f);
  EXPECT_NEAR(first_roll_per_axis[3], 186.0f, 1.0e-5f);
  for (const int axis_id : tiller_axes) {
    EXPECT_EQ(leaves_per_axis[axis_id], ComputeSorghumTillerLeafBudget(10, 0.90f));
  }
}

TEST(LSystemSorghumTillers, TillerBranchAnglesRecoverToFinalLeanSmoothly) {
  constexpr float kInsertionAngle = 40.0f;
  constexpr float kFinalLean = 15.0f;
  constexpr int kAxisPhytomers = 10;

  const float first = ComputeSorghumTillerInternodeBranchAngle(0, kAxisPhytomers, kInsertionAngle, kFinalLean, 1.0f);
  float recovery_sum = 0.0f;
  for (int rank = 1; rank < kAxisPhytomers; ++rank) {
    const float recovery =
        ComputeSorghumTillerInternodeBranchAngle(rank, kAxisPhytomers, kInsertionAngle, kFinalLean, 1.0f);
    EXPECT_LT(recovery, 0.0f);
    recovery_sum += recovery;
  }

  EXPECT_FLOAT_EQ(first, kInsertionAngle);
  EXPECT_NEAR(recovery_sum, -(kInsertionAngle - kFinalLean), 1.0e-5f);
  EXPECT_FLOAT_EQ(
      ComputeSorghumTillerInternodeBranchAngle(kAxisPhytomers, kAxisPhytomers, kInsertionAngle, kFinalLean, 1.0f),
      0.0f);
}

TEST(LSystemSorghumTillers, SelectionAndLeafBudgetsUsePeerAxisRatios) {
  SampledSorghumParams params;
  params.tiller_origin_ranks = {3, 4, 2, 1};

  EXPECT_EQ(ComputeSorghumTillerSelectionIndex(params, 3), 0);
  EXPECT_EQ(ComputeSorghumTillerSelectionIndex(params, 4), 1);
  EXPECT_EQ(ComputeSorghumTillerSelectionIndex(params, 2), 2);
  EXPECT_EQ(ComputeSorghumTillerSelectionIndex(params, 1), 3);
  EXPECT_EQ(ComputeSorghumTillerSelectionIndex(params, 5), -1);
  EXPECT_EQ(ComputeSorghumTillerLeafBudget(18, 0.90f), 16);
  EXPECT_EQ(ComputeSorghumTillerLeafBudget(10, 0.93f), 9);
  EXPECT_FLOAT_EQ(ComputeSorghumAxisRankPosition(0, 9), 0.0f);
  EXPECT_FLOAT_EQ(ComputeSorghumAxisRankPosition(8, 9), 1.0f);
  EXPECT_NEAR(ComputeSorghumTillerCatchUpPlastochron(50.0f, 9, 16, 1.0f), 28.125f, 1.0e-6f);
}

TEST(LSystemSorghumTillers, DistichousOriginRanksReceiveCenteredSameSideSplay) {
  const std::vector<int> four_tiller_ranks{3, 4, 2, 1};
  EXPECT_FLOAT_EQ(ComputeSorghumTillerSameSideSplay(four_tiller_ranks, 0, 12.0f), -6.0f);
  EXPECT_FLOAT_EQ(ComputeSorghumTillerSameSideSplay(four_tiller_ranks, 1, 12.0f), -6.0f);
  EXPECT_FLOAT_EQ(ComputeSorghumTillerSameSideSplay(four_tiller_ranks, 2, 12.0f), 6.0f);
  EXPECT_FLOAT_EQ(ComputeSorghumTillerSameSideSplay(four_tiller_ranks, 3, 12.0f), 6.0f);

  const std::vector<int> five_tiller_ranks{3, 4, 2, 1, 5};
  EXPECT_FLOAT_EQ(ComputeSorghumTillerSameSideSplay(five_tiller_ranks, 0, 12.0f), -12.0f);
  EXPECT_FLOAT_EQ(ComputeSorghumTillerSameSideSplay(five_tiller_ranks, 3, 12.0f), 0.0f);
  EXPECT_FLOAT_EQ(ComputeSorghumTillerSameSideSplay(five_tiller_ranks, 4, 12.0f), 12.0f);
}

TEST(LSystemSorghumPanicle, MainApexEmitsFlagLeafAndTripleSpikelets) {
  SampledSorghumParams params;
  params.total_phytomer_count = 3;
  params.tiller_count = 0;
  params.tiller_origin_ranks.clear();
  params.plastochron_gdd = 1.0f;
  params.maturity_gdd = 1.0f;
  params.gdd_step = 1.0f;
  params.flag_leaf_length_scale = 0.75f;
  params.flag_leaf_width_scale = 0.80f;
  params.flag_leaf_insertion_angle_offset = -10.0f;
  params.flag_leaf_bending_scale = 0.50f;
  params.enable_panicle = true;
  params.panicle_initiation_gdd = 0.0f;
  params.panicle_maturity_gdd = 1.0f;
  params.panicle_primary_branch_count = 4;
  params.panicle_spikelet_pairs_per_branch = 5;
  ApplyMeanStdPlotDefaultsToAll(params.internode_length, params.internode_thickness, params.leaf_blade_length,
                                params.leaf_blade_max_width, params.leaf_blade_thickness, params.leaf_sheath_thickness,
                                params.leaf_sheath_length, params.leaf_neck_length, params.leaf_sheath_end_width_ratio,
                                params.leaf_neck_end_width_ratio, params.leaf_blade_end_width_ratio,
                                params.leaf_insertion_angle, params.leaf_roll_angle, params.leaf_curling,
                                params.leaf_bending, params.leaf_waviness, params.tiller_leaf_area_ratio_by_origin);
  SetConstantPlot(params.leaf_blade_length, 1.0f);
  SetConstantPlot(params.leaf_blade_max_width, 0.10f);
  SetConstantPlot(params.leaf_insertion_angle, 60.0f);
  SetConstantPlot(params.leaf_bending, 40.0f);

  SorghumGraph graph(1);
  graph.RefNode(0).symbol_id = SorghumSymbol::Root;
  graph.RefNode(0).data.Set<SorghumRoot>(SorghumRoot{});
  const auto apex_handle = graph.Extend(0, false);
  SorghumApex apex;
  apex.vigor = params.total_phytomer_count;
  apex.sampled_plastochron_gdd = 1.0f;
  apex.age_gdd = 1.0f;
  apex.axis_phytomer_count = params.total_phytomer_count;
  graph.RefNode(apex_handle).symbol_id = SorghumSymbol::Apex;
  graph.RefNode(apex_handle).data.Set<SorghumApex>(apex);
  graph.SortLists();

  SorghumEngine engine;
  engine.topology_rules = CreateSorghumTopologyRules(params);
  engine.growth_rules = CreateSorghumGrowthRules(params);
  std::mt19937 rng(17u);
  for (int step = 0; step < 16; ++step) {
    engine.ApplyGrowthRules(graph, rng);
    engine.ApplyTopologyRules(graph, rng);
    graph.SortLists();
  }

  int flag_leaf_count = 0;
  int rachis_count = 0;
  int branch_count = 0;
  int spikelet_count = 0;
  int pedicellate_count = 0;
  for (const auto handle : graph.PeekSortedNodeList()) {
    const auto& node = graph.PeekNode(handle);
    if (node.data.Is<SorghumLeaf>()) {
      const auto& leaf = node.data.Get<SorghumLeaf>();
      EXPECT_FLOAT_EQ(leaf.growth_progress, 1.0f);
      EXPECT_FLOAT_EQ(leaf.blade_length, leaf.target_blade_length);
      if (!leaf.is_flag_leaf)
        continue;
      ++flag_leaf_count;
      EXPECT_EQ(leaf.rank, 2);
      EXPECT_FLOAT_EQ(leaf.target_blade_length, 0.75f);
      EXPECT_FLOAT_EQ(leaf.target_blade_max_width, 0.08f);
      EXPECT_FLOAT_EQ(leaf.target_insertion_angle_deg, 50.0f);
      EXPECT_FLOAT_EQ(leaf.target_bending, 20.0f);
    } else if (node.data.Is<SorghumPanicleRachis>()) {
      ++rachis_count;
    } else if (node.data.Is<SorghumPanicleBranch>()) {
      ++branch_count;
      EXPECT_TRUE(node.data.Get<SorghumPanicleBranch>().spikelets_emitted);
      ASSERT_GE(node.GetParentHandle(), 0);
      EXPECT_TRUE(graph.PeekNode(node.GetParentHandle()).data.Is<SorghumPanicleRachis>());
    } else if (node.data.Is<SorghumPanicleSpikelet>()) {
      ++spikelet_count;
      pedicellate_count += node.data.Get<SorghumPanicleSpikelet>().pedicellate ? 1 : 0;
      ASSERT_GE(node.GetParentHandle(), 0);
      EXPECT_TRUE(graph.PeekNode(node.GetParentHandle()).data.Is<SorghumPanicleBranch>());
    } else if (node.data.Is<SorghumInternode>()) {
      const auto& internode = node.data.Get<SorghumInternode>();
      EXPECT_FLOAT_EQ(internode.growth_progress, 1.0f);
      EXPECT_FLOAT_EQ(internode.length, internode.target_length);
    }
  }

  EXPECT_EQ(flag_leaf_count, 1);
  EXPECT_EQ(rachis_count, 1);
  EXPECT_EQ(branch_count, params.panicle_primary_branch_count);
  EXPECT_EQ(spikelet_count, params.panicle_primary_branch_count * params.panicle_spikelet_pairs_per_branch * 3);
  EXPECT_EQ(pedicellate_count, spikelet_count * 2 / 3);
}
