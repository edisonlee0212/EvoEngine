#include <gtest/gtest.h>
#include <limits>
#include "UniverseLayer.hpp"

using namespace universe_package;

namespace {
void ExpectMatrix(const glm::dmat4& actual, const glm::dmat4& expected, const double tolerance = 1e-8) {
  for (int col = 0; col < 4; ++col)
    for (int row = 0; row < 4; ++row)
      EXPECT_NEAR(actual[col][row], expected[col][row], tolerance);
}
StarPickSnapshot Select(const StarClusterBatch& batch, const std::shared_ptr<StarCluster>& cluster,
                        const uint32_t ordinal = 0) {
  StarPickSnapshot result;
  result.result.valid = 1;
  result.cluster = cluster;
  result.seed = cluster->seed;
  result.identity = batch.clocks.at(cluster.get()).identity;
  result.ordinal = ordinal;
  return result;
}
std::shared_ptr<StarCluster> Cluster() {
  auto cluster = std::make_shared<StarCluster>();
  cluster->SetStarCount(4);
  return cluster;
}
}  // namespace

TEST(UniverseStarFollow, ReferenceFrameCentersStarAndPointsAtClusterOrigin) {
  const auto world = glm::translate(glm::dmat4(1), glm::dvec3(50, -30, 20)) *
                     glm::rotate(glm::dmat4(1), 0.8, glm::normalize(glm::dvec3(1, 2, 3))) *
                     glm::scale(glm::dmat4(1), glm::dvec3(2, 3, 4));
  const glm::dvec3 star(123, 42, 89);
  const auto frame = StarReferenceFrame(star, world);
  const auto inverse = glm::inverse(frame);
  EXPECT_LT(glm::length(glm::dvec3(inverse * glm::dvec4(star, 1))), 1e-10);
  const auto center = inverse * world[3];
  EXPECT_NEAR(center.x, 0, 1e-10);
  EXPECT_NEAR(center.y, 0, 1e-10);
  EXPECT_NEAR(center.z, -glm::distance(star, glm::dvec3(world[3])), 1e-10);
  ExpectMatrix(glm::transpose(glm::dmat4(glm::dmat3(frame))) * glm::dmat4(glm::dmat3(frame)), glm::dmat4(1));
  const auto example = glm::inverse(StarReferenceFrame({0, 0, 1000}, glm::dmat4(1))) * glm::dvec4(0, 0, 0, 1);
  EXPECT_DOUBLE_EQ(example.z, -1000);
}

TEST(UniverseStarFollow, FocusPoseAlignsStarAndClusterAtTwentyRadii) {
  for (const double radius : {0.25, 1.0, 2.0}) {
    const auto pose = CalculateStarFollowCameraPose(radius);
    ASSERT_TRUE(pose.valid);
    EXPECT_EQ(pose.position, glm::dvec3(0, 0, 20 * radius));
    EXPECT_EQ(pose.rotation * glm::dvec3(0, 0, -1), glm::dvec3(0, 0, -1));
    for (const auto star : {glm::dvec3(10, 20, 30), glm::dvec3(0), glm::dvec3(0, 100, 0)}) {
      const auto frame = StarReferenceFrame(star, glm::dmat4(1));
      const auto center = glm::inverse(frame) * glm::dvec4(0, 0, 0, 1);
      EXPECT_NEAR(center.x, 0, 1e-10);
      EXPECT_NEAR(center.y, 0, 1e-10);
      EXPECT_LE(center.z, 0);
    }
  }
}

TEST(UniverseStarFollow, DistributedRadiusDrivesFollowDistanceAndOverviewBounds) {
  const auto cluster = Cluster();
  cluster->radius_standard_deviation = 0.5;
  cluster->radius_min = 0.1;
  cluster->radius_max = 3;
  StarClusterBatch batch;
  batch.Update({{cluster}}, 0);
  StarFollowState follow;
  follow.Update(Select(batch, cluster, 2), batch, true);
  ASSERT_TRUE(follow.following);
  const auto& p = batch.parameters[0];
  const double expected = EvaluateStar(p, batch.samples[2]).world_position_radius.w;
  EXPECT_DOUBLE_EQ(follow.selected_radius, expected);
  EXPECT_DOUBLE_EQ(CalculateStarFollowCameraPose(follow.selected_radius).position.z, 20 * expected);
  auto uniform = p;
  uniform.center_offset.w = 0;
  EXPECT_DOUBLE_EQ(StarClusterBoundingRadius(p, batch.gaussian_bounds[0]) -
                       StarClusterBoundingRadius(uniform, batch.gaussian_bounds[0]),
                   cluster->radius_max - cluster->visual_radius);
  for (const auto& sample : batch.samples) {
    const auto star = EvaluateStar(p, sample).world_position_radius;
    EXPECT_LE(glm::length(glm::dvec3(star)) + star.w, StarClusterBoundingRadius(p, batch.gaussian_bounds[0]));
  }
}

TEST(UniverseStarFollow, OverviewAtOriginUsesCurrentCameraBackwardDirection) {
  const auto rotation = glm::angleAxis(0.75, glm::normalize(glm::dvec3(1, 2, 3)));
  const auto projection = glm::perspective(glm::radians(60.0), 16.0 / 9, 0.1, 1000000.0);
  const auto pose = CalculateStarOverviewCameraPose({}, rotation, 100, projection, 0.1);
  ASSERT_TRUE(pose.valid);
  EXPECT_NEAR(glm::length(pose.position), 204, 1e-10);
  EXPECT_LT(glm::length(glm::normalize(pose.position) - rotation * glm::dvec3(0, 0, 1)), 1e-10);
  EXPECT_LT(glm::length(pose.rotation * glm::dvec3(0, 0, -1) + glm::normalize(pose.position)), 1e-10);
}

TEST(UniverseStarFollow, InvalidRadiusRejectsFocusAndFollowEntryWithoutLosingSelection) {
  const auto cluster = Cluster();
  StarClusterBatch batch;
  batch.Update({{cluster}}, 0);
  const auto selected = Select(batch, cluster);
  for (const double radius : {0.0, -0.25, 1e100, 1e-100, std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::max()}) {
    const auto pose = CalculateStarFollowCameraPose(radius);
    EXPECT_FALSE(pose.valid);
    batch.parameters[0].tilt_radius.w = radius;
    StarFollowState follow;
    const auto change = follow.Update(selected, batch, true);
    EXPECT_TRUE(follow.available);
    EXPECT_FALSE(follow.following);
    EXPECT_FALSE(change.rebase);
    EXPECT_NE(follow.status.find("radius"), std::string::npos);
    EXPECT_EQ(follow.generation, 0u);
    EXPECT_TRUE(selected.result.valid);
  }
}

TEST(UniverseStarFollow, SmallRepresentableRadiusProducesNonzeroCameraPose) {
  const auto pose = CalculateStarFollowCameraPose(1e-40);
  ASSERT_TRUE(pose.valid);
  const auto position = glm::vec3(pose.position);
  EXPECT_NE(position, glm::vec3(0));
  EXPECT_EQ(position.x, 0);
  EXPECT_EQ(position.y, 0);
  EXPECT_GT(position.z, 0);
  EXPECT_NEAR(glm::length(pose.position) / 1e-40, 20, 1e-10);
  const auto cluster = Cluster();
  cluster->visual_radius = 1e-40;
  StarClusterBatch batch;
  batch.Update({{cluster}}, 0);
  StarFollowState follow;
  EXPECT_TRUE(follow.Update(Select(batch, cluster), batch, true).rebase);
  EXPECT_TRUE(follow.following);
}

TEST(UniverseStarFollow, InvalidRadiusDuringFollowDetachesWithoutAutomaticResume) {
  const auto cluster = Cluster();
  StarClusterBatch batch;
  batch.Update({{cluster}}, 0);
  const auto selected = Select(batch, cluster);
  StarFollowState follow;
  follow.Update(selected, batch, true);
  ASSERT_TRUE(follow.following);
  EXPECT_DOUBLE_EQ(follow.selected_radius, cluster->visual_radius);
  const auto frame = follow.reference_to_world;
  batch.parameters[0].tilt_radius.w = 0;
  const auto exit = follow.Update(selected, batch, false);
  EXPECT_TRUE(exit.rebase);
  EXPECT_FALSE(follow.following);
  ExpectMatrix(exit.camera_transform, frame);
  EXPECT_NE(follow.status.find("radius"), std::string::npos);
  batch.parameters[0].tilt_radius.w = 1;
  EXPECT_FALSE(follow.Update(selected, batch, false).rebase);
  EXPECT_FALSE(follow.following);
}

TEST(UniverseStarFollow, DegenerateAxesProduceFiniteProperRotation) {
  auto previous = glm::dmat3(glm::rotate(glm::dmat4(1), 0.5, glm::dvec3(1, 0, 0)));
  for (const auto position : {glm::dvec3(0), glm::dvec3(0, 1000, 0), glm::dvec3(1e-8, 1000, 0)}) {
    const auto frame = StarReferenceFrame(position, glm::dmat4(1), previous);
    EXPECT_NEAR(glm::determinant(glm::dmat3(frame)), 1, 1e-10);
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        EXPECT_TRUE(std::isfinite(frame[i][j]));
    if (position == glm::dvec3(0))
      ExpectMatrix(glm::dmat4(glm::dmat3(frame)), glm::dmat4(previous));
  }
}

TEST(UniverseStarFollow, EnterAnimateRetargetAndExitPreservePoseAcrossCoordinateChanges) {
  const auto first = Cluster(), second = Cluster();
  StarClusterBatch batch;
  batch.Update({{first}, {second}}, 0);
  StarFollowState follow;
  auto selected = Select(batch, first, 2);
  const auto enter = follow.Update(selected, batch, true);
  ASSERT_TRUE(follow.following);
  EXPECT_TRUE(enter.look_at);
  ExpectMatrix(follow.reference_to_world * enter.camera_transform, glm::dmat4(1));
  const auto initial_frame = follow.reference_to_world;
  batch.Update({{first}, {second}}, 5);
  const auto animate = follow.Update(selected, batch, false);
  EXPECT_FALSE(animate.rebase);
  EXPECT_NE(follow.reference_to_world, initial_frame);
  const auto previous_frame = follow.reference_to_world;
  selected = Select(batch, second, 1);
  const auto retarget = follow.Update(selected, batch, false);
  EXPECT_TRUE(retarget.rebase);
  EXPECT_TRUE(retarget.look_at);
  ExpectMatrix(follow.reference_to_world * retarget.camera_transform, previous_frame);
  const auto last_frame = follow.reference_to_world;
  const auto exit = follow.Update(selected, batch, true);
  EXPECT_FALSE(follow.following);
  EXPECT_FALSE(exit.look_at);
  ExpectMatrix(exit.camera_transform, last_frame);
  EXPECT_EQ(follow.generation, 3u);
}

TEST(UniverseStarFollow, PauseRepackAndIndependentSettingsPreserveFollowIdentity) {
  const auto first = Cluster(), second = Cluster();
  StarClusterBatch batch;
  batch.Update({{first}, {second}}, 0);
  const auto selected = Select(batch, second, 2);
  StarFollowState follow;
  follow.Update(selected, batch, true);
  const auto original = follow.reference_to_world;
  second->paused = true;
  first->SetStarCount(257);
  batch.Update({{first}, {second}}, 3);
  EXPECT_FALSE(follow.Update(selected, batch, false).rebase);
  ExpectMatrix(follow.reference_to_world, original);
  second->paused = false;
  batch.Update({{first}, {second}}, 4);
  follow.Update(selected, batch, false);
  EXPECT_NE(follow.reference_to_world, original);
  const auto source = batch.parameters[1];
  auto transformed = source;
  const auto matrix =
      glm::inverse(follow.reference_to_world) * glm::dmat4(source.world0, source.world1, source.world2, source.world3);
  transformed.world0 = matrix[0];
  transformed.world1 = matrix[1];
  transformed.world2 = matrix[2];
  transformed.world3 = matrix[3];
  EXPECT_LT(glm::length(
                glm::dvec3(EvaluateStar(transformed, batch.samples[batch.ranges[1].offset + 2]).world_position_radius)),
            1e-8);
}

TEST(UniverseStarFollow, UnavailableTargetsDetachWithoutAutomaticResumeAndResetClearsRuntime) {
  for (int mutation = 0; mutation < 4; ++mutation) {
    const auto cluster = Cluster();
    StarClusterBatch batch;
    batch.Update({{cluster}}, 0);
    const auto selected = Select(batch, cluster, 3);
    StarFollowState follow;
    follow.Update(selected, batch, true);
    const auto last_frame = follow.reference_to_world;
    if (mutation == 0)
      cluster->SetStarCount(3);
    if (mutation == 1)
      ++cluster->seed;
    if (mutation == 2)
      batch.Update({{cluster, glm::dmat4(1), false}}, 1);
    else if (mutation == 3)
      batch.Update({}, 1);
    else
      batch.Update({{cluster}}, 1);
    const auto exit = follow.Update(selected, batch, false);
    EXPECT_FALSE(follow.available);
    EXPECT_FALSE(follow.following);
    EXPECT_TRUE(exit.rebase);
    ExpectMatrix(exit.camera_transform, last_frame);
    cluster->SetStarCount(4);
    cluster->seed = selected.seed;
    batch.Update({{cluster}}, 2);
    follow.Update(selected, batch, false);
    EXPECT_FALSE(follow.following);
    follow = {};
    EXPECT_FALSE(follow.following);
    EXPECT_FALSE(follow.available);
    ExpectMatrix(follow.reference_to_world, glm::dmat4(1));
  }
}

TEST(UniverseStarFollow, ViewScaleEasesAndReversesWithoutJumpOrPopulationChanges) {
  StarViewTransition view;
  view.SetLocked(true, 10);
  EXPECT_DOUBLE_EQ(view.disk_scale, 1);
  view.Update(10.5);
  EXPECT_DOUBLE_EQ(view.disk_scale, 1 + 99 * 0.9375);
  const double halfway = view.disk_scale;
  view.SetLocked(false, 10.5);
  EXPECT_DOUBLE_EQ(view.disk_scale, halfway);
  view.Update(11.5);
  EXPECT_DOUBLE_EQ(view.disk_scale, 1);
  view.SetLocked(true, 12);
  view.Update(13);
  EXPECT_DOUBLE_EQ(view.disk_scale, 100);
  const auto cluster = Cluster();
  StarClusterBatch batch;
  ASSERT_TRUE(batch.Update({{cluster}}, 0));
  const auto samples = batch.samples;
  const auto bounds = batch.gaussian_bounds;
  const auto revision = batch.population_revision;
  for (const double scale : {1.0, halfway, 100.0, 1.0}) {
    EXPECT_FALSE(batch.Update({{cluster}}, 0, scale));
    EXPECT_EQ(batch.population_revision, revision);
    EXPECT_EQ(batch.gaussian_bounds, bounds);
    EXPECT_EQ(std::memcmp(samples.data(), batch.samples.data(), samples.size() * sizeof(StarBaseSample)), 0);
    EXPECT_DOUBLE_EQ(batch.parameters[0].ellipse0.x + batch.parameters[0].ellipse0.y, 30000 * scale);
    EXPECT_DOUBLE_EQ(batch.parameters[0].tilt_radius.w, 1);
    EXPECT_DOUBLE_EQ(cluster->disk_diameter, 30000);
  }
}

TEST(UniverseStarFollow, OverviewBoundsEveryStarAcrossPhasesTransformsAndViewportShapes) {
  const auto cluster = Cluster();
  cluster->SetStarCount(1000);
  cluster->center_offset = {300, -200, 100};
  cluster->center_position = {-500, 800, 90};
  const auto world = glm::translate(glm::dmat4(1), glm::dvec3(100, 200, 300)) *
                     glm::rotate(glm::dmat4(1), 0.7, glm::normalize(glm::dvec3(1, 2, 3))) *
                     glm::scale(glm::dmat4(1), glm::dvec3(2, 3, 4));
  StarClusterBatch batch;
  batch.Update({{cluster, world}}, 0);
  for (const double time : {0.0, 100.0, 1000000.0}) {
    cluster->disk_tilt_x = time == 0 ? 0 : time == 100 ? 35 : 179;
    cluster->core_tilt_z = time == 100 ? -70 : 0;
    cluster->center_tilt_x = time == 100 ? -20 : 0;
    const auto parameters = BuildStarClusterParameters(*cluster, world, time);
    const double bound = StarClusterBoundingRadius(parameters, batch.gaussian_bounds[0]);
    for (const double aspect : {0.5, 1.0, 16.0 / 9}) {
      const auto projection = glm::perspective(glm::radians(60.0), aspect, 0.1, 1000000.0);
      const auto pose = CalculateStarOverviewCameraPose({3, 4, 5}, glm::dquat(1, 0, 0, 0), bound, projection, 0.1);
      ASSERT_TRUE(pose.valid);
      const auto view = glm::inverse(glm::translate(glm::dmat4(1), pose.position) * glm::mat4_cast(pose.rotation));
      for (const auto& sample : batch.samples) {
        const auto star = EvaluateStar(parameters, sample).world_position_radius;
        EXPECT_LE(glm::length(glm::dvec3(star)) + star.w, bound);
        const auto clip = projection * view * glm::dvec4(glm::dvec3(star), 1);
        EXPECT_GT(clip.w, 0);
        EXPECT_LT(std::abs(clip.x / clip.w), 1);
        EXPECT_LT(std::abs(clip.y / clip.w), 1);
        EXPECT_LT(clip.z / clip.w, 1);
      }
    }
  }
}

TEST(UniverseStarFollow, OverviewCombinesVerticalTailsWithHorizontalOrbitForCloserFit) {
  const auto cluster = Cluster();
  cluster->SetStarCount(500000);
  StarClusterBatch batch;
  batch.Update({{cluster}}, 0);
  const auto& p = batch.parameters[0];
  const auto spread = batch.gaussian_bounds[0] * glm::dvec3(cluster->xz_spread, cluster->y_spread, cluster->xz_spread) *
                      cluster->disk_diameter;
  const double old_bound = ((std::max)(p.ellipse0.x, p.ellipse0.y) + glm::length(spread)) / 20 + cluster->visual_radius;
  const double tighter_bound = StarClusterBoundingRadius(p, batch.gaussian_bounds[0]);
  EXPECT_LT(tighter_bound, old_bound * 0.9);
  for (const auto& sample : batch.samples)
    EXPECT_LE(glm::length(glm::dvec3(EvaluateStar(p, sample).world_position_radius)) + cluster->visual_radius,
              tighter_bound);
}
