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

TEST(UniverseStarFollow, FocusPoseUsesFiftyRadiiAndPreservesApproachDirection) {
  for (const auto position : {glm::dvec3(3, 4, 5), glm::dvec3(0, 10, 0), glm::dvec3(0, -10, 0)}) {
    const auto pose = CalculateStarFollowCameraPose(position, glm::dquat(1, 0, 0, 0), 0.25);
    ASSERT_TRUE(pose.valid);
    EXPECT_NEAR(glm::length(pose.position), 12.5, 1e-10);
    EXPECT_LT(glm::length(glm::normalize(pose.position) - glm::normalize(position)), 1e-10);
    EXPECT_LT(glm::length(pose.rotation * glm::dvec3(0, 0, -1) + glm::normalize(position)), 1e-10);
  }
}

TEST(UniverseStarFollow, FocusPoseAtOriginUsesCurrentCameraBackwardDirection) {
  const auto rotation = glm::angleAxis(0.75, glm::normalize(glm::dvec3(1, 2, 3)));
  const auto pose = CalculateStarFollowCameraPose({}, rotation, 2);
  ASSERT_TRUE(pose.valid);
  EXPECT_NEAR(glm::length(pose.position), 100, 1e-10);
  EXPECT_LT(glm::length(pose.position / 100.0 - rotation * glm::dvec3(0, 0, 1)), 1e-10);
  EXPECT_LT(glm::length(pose.rotation * glm::dvec3(0, 0, -1) + glm::normalize(pose.position)), 1e-10);
}

TEST(UniverseStarFollow, InvalidRadiusRejectsFocusAndFollowEntryWithoutLosingSelection) {
  const auto cluster = Cluster();
  StarClusterBatch batch;
  batch.Update({{cluster}}, 0);
  const auto selected = Select(batch, cluster);
  for (const double radius : {0.0, -0.25, 1e100, 1e-100, std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::max()}) {
    const auto pose = CalculateStarFollowCameraPose({0, 0, 3}, glm::dquat(1, 0, 0, 0), radius);
    EXPECT_FALSE(pose.valid);
    EXPECT_EQ(pose.position, glm::dvec3(0, 0, 3));
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
  const auto pose = CalculateStarFollowCameraPose({1, 1, 1}, glm::dquat(1, 0, 0, 0), 1e-40);
  ASSERT_TRUE(pose.valid);
  const auto position = glm::vec3(pose.position);
  EXPECT_NE(position, glm::vec3(0));
  EXPECT_GT(position.x, 0);
  EXPECT_GT(position.y, 0);
  EXPECT_GT(position.z, 0);
  EXPECT_NEAR(glm::length(pose.position) / 1e-40, 50, 1e-10);
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
