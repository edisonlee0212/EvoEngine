#include <gtest/gtest.h>

#include "StarCluster.hpp"
#include "UniverseSerializationAdapters.hpp"

using namespace universe_package;

TEST(UniverseStarCluster, DeterministicSamplesDependOnlyOnSeedAndStableId) {
  const auto first = StarCluster::GenerateBaseSample(42, 17);
  const auto repeated = StarCluster::GenerateBaseSample(42, 17);
  const auto reseeded = StarCluster::GenerateBaseSample(43, 17);
  EXPECT_EQ(0, std::memcmp(&first, &repeated, sizeof(first)));
  EXPECT_NE(0, std::memcmp(&first, &reseeded, sizeof(first)));
}

TEST(UniverseStarCluster, StableIdsAndSwapRemovalKeepDenseSamplesAligned) {
  StarCluster cluster;
  const auto ids = cluster.AddStars(4);
  const auto last_sample = cluster.GetBaseSamples().back();
  ASSERT_TRUE(cluster.RemoveStar(ids[1]));
  EXPECT_EQ((std::vector<StarId>{ids[0], ids[3], ids[2]}), cluster.GetStarIds());
  EXPECT_EQ(0, std::memcmp(&last_sample, &cluster.GetBaseSamples()[1], sizeof(last_sample)));
  EXPECT_FALSE(cluster.RemoveStar(ids[1]));
  EXPECT_EQ(ids[3] + 1, cluster.AddStar());
}

TEST(UniverseStarCluster, ClustersKeepIndependentPopulationAndSettings) {
  StarCluster first;
  StarCluster second;
  first.seed = 5;
  second.seed = 9;
  first.disk_diameter = 123.0;
  (void)first.AddStars(3);
  (void)second.AddStars(2);
  first.ClearStars();
  EXPECT_EQ(0u, first.GetStarCount());
  EXPECT_EQ(2u, second.GetStarCount());
  EXPECT_EQ(3000.0, second.disk_diameter);
}

TEST(UniverseStarCluster, SettingsEditsDoNotRegenerateBaseSamples) {
  StarCluster cluster;
  (void)cluster.AddStars(8);
  const auto samples = cluster.GetBaseSamples();
  const auto population_revision = cluster.GetPopulationRevision();
  const auto parameter_revision = cluster.GetParameterRevision();
  cluster.disk_diameter = 900.0;
  cluster.disk_color = glm::vec3(0.25f);
  cluster.MarkParametersDirty();
  EXPECT_EQ(0, std::memcmp(samples.data(), cluster.GetBaseSamples().data(), samples.size() * sizeof(StarBaseSample)));
  EXPECT_EQ(population_revision, cluster.GetPopulationRevision());
  EXPECT_EQ(parameter_revision + 1, cluster.GetParameterRevision());
  EXPECT_EQ(900.0, cluster.BuildGpuParameters(glm::dmat4(1.0), 3.0).ellipse0.x * 2.0);
}

TEST(UniverseStarCluster, ReseedRegeneratesSamplesWithoutChangingIds) {
  StarCluster cluster;
  const auto ids = cluster.AddStars(4);
  const auto samples = cluster.GetBaseSamples();
  cluster.Reseed(99);
  EXPECT_EQ(ids, cluster.GetStarIds());
  EXPECT_NE(0, std::memcmp(samples.data(), cluster.GetBaseSamples().data(), samples.size() * sizeof(StarBaseSample)));
}

TEST(UniverseStarCluster, SerializationPersistsAuthoringButNotRuntimeState) {
  StarCluster source;
  source.seed = 77;
  source.disk_diameter = 456.0;
  source.time_scale = -2.5;
  source.paused = true;
  const auto ids = source.AddStars(3);
  ASSERT_TRUE(source.RemoveStar(ids[1]));

  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  SerializeStarCluster(emitter, source);
  emitter << YAML::EndMap;
  const std::string text = emitter.c_str();
  EXPECT_EQ(std::string::npos, text.find("capacity"));
  EXPECT_EQ(std::string::npos, text.find("buffer"));
  EXPECT_EQ(std::string::npos, text.find("revision"));
  EXPECT_EQ(std::string::npos, text.find("particle"));
  EXPECT_EQ(std::string::npos, text.find("cast_shadow"));

  StarCluster restored;
  DeserializeStarCluster(YAML::Load(text), restored);
  EXPECT_EQ(source.GetStarIds(), restored.GetStarIds());
  EXPECT_EQ(source.GetBaseSamples().size(), restored.GetBaseSamples().size());
  EXPECT_EQ(77u, restored.seed);
  EXPECT_EQ(456.0, restored.disk_diameter);
  EXPECT_EQ(-2.5, restored.time_scale);
  EXPECT_TRUE(restored.paused);
  EXPECT_EQ(0u, restored.GetCapacity());
  EXPECT_EQ(ids[2] + 1, restored.AddStar());
  restored.PostCloneAction({});
  EXPECT_EQ(3u, restored.GetStarCount());
  EXPECT_EQ(0u, restored.GetCapacity());
}

TEST(UniverseStarCluster, GpuBufferLayoutsRemainExact) {
  EXPECT_EQ(32u, sizeof(StarBaseSample));
  EXPECT_EQ(448u, sizeof(StarClusterGpuParameters));
  EXPECT_EQ(64u, sizeof(StarClusterGpuResult));
  EXPECT_EQ(0u, offsetof(StarClusterGpuResult, world_position_radius));
  EXPECT_EQ(32u, offsetof(StarClusterGpuResult, color_emission));
  EXPECT_EQ(48u, offsetof(StarClusterGpuResult, alpha_padding));
  EXPECT_EQ(0u, offsetof(StarClusterGpuParameters, population_revision));
  EXPECT_EQ(16u, offsetof(StarClusterGpuParameters, ellipse0));
  EXPECT_EQ(240u, offsetof(StarClusterGpuParameters, world0));
  EXPECT_EQ(368u, offsetof(StarClusterGpuParameters, disk_color_intensity));
  EXPECT_EQ(416u, offsetof(StarClusterGpuParameters, time_padding));
}

TEST(UniverseStarCluster, PooledReuseRestoresAuthoringDefaults) {
  StarCluster cluster;
  cluster.seed = 99;
  cluster.disk_diameter = 12.0;
  cluster.paused = true;
  (void)cluster.AddStars(3);
  cluster.OnDestroy();
  cluster.OnCreate();
  EXPECT_EQ(1u, cluster.seed);
  EXPECT_EQ(3000.0, cluster.disk_diameter);
  EXPECT_FALSE(cluster.paused);
  EXPECT_EQ(0u, cluster.GetStarCount());
  EXPECT_EQ(1u, cluster.AddStar());
}
