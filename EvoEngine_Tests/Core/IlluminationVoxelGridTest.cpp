#include <gtest/gtest.h>

#include "IlluminationVoxelGrid.hpp"

using namespace evo_engine;

TEST(IlluminationVoxelGrid, GeneratesDeterministicPointsInsideEveryVoxel) {
  const glm::vec3 volume_size(4.0f, 2.0f, 2.0f);
  const glm::ivec3 resolution(2, 1, 1);
  const auto points = GenerateIlluminationVoxelPoints(volume_size, resolution, 3, 42);
  const auto repeated = GenerateIlluminationVoxelPoints(volume_size, resolution, 3, 42);

  ASSERT_EQ(points.size(), 6);
  ASSERT_EQ(repeated.size(), points.size());
  for (size_t i = 0; i < points.size(); i++) {
    EXPECT_EQ(points[i].coordinate, repeated[i].coordinate);
    EXPECT_EQ(points[i].position, repeated[i].position);
  }
  for (size_t i = 0; i < 3; i++) {
    EXPECT_EQ(points[i].coordinate, glm::ivec3(0, 0, 0));
    EXPECT_GE(points[i].position.x, -2.0f);
    EXPECT_LT(points[i].position.x, 0.0f);
  }
  for (size_t i = 3; i < 6; i++) {
    EXPECT_EQ(points[i].coordinate, glm::ivec3(1, 0, 0));
    EXPECT_GE(points[i].position.x, 0.0f);
    EXPECT_LT(points[i].position.x, 2.0f);
  }
}

TEST(IlluminationVoxelGrid, AveragesPointEnergyPerVoxel) {
  const glm::vec3 volume_size(2.0f, 1.0f, 1.0f);
  const glm::ivec3 resolution(2, 1, 1);
  const std::vector<IlluminationVoxelPoint> points = {
      {{0, 0, 0}, {-0.8f, 0.0f, 0.0f}},
      {{0, 0, 0}, {-0.2f, 0.0f, 0.0f}},
      {{1, 0, 0}, {0.5f, 0.0f, 0.0f}},
  };
  const std::vector<glm::vec3> energies = {
      {1.0f, 2.0f, 3.0f},
      {3.0f, 4.0f, 5.0f},
      {8.0f, 6.0f, 4.0f},
  };

  const auto voxels = AggregateIlluminationVoxels(points, energies, volume_size, resolution);

  ASSERT_EQ(voxels.size(), 2);
  EXPECT_EQ(voxels[0].center, glm::vec3(-0.5f, 0.0f, 0.0f));
  EXPECT_EQ(voxels[0].energy, glm::vec3(2.0f, 3.0f, 4.0f));
  EXPECT_EQ(voxels[0].sample_count, 2);
  EXPECT_EQ(voxels[1].center, glm::vec3(0.5f, 0.0f, 0.0f));
  EXPECT_EQ(voxels[1].energy, glm::vec3(8.0f, 6.0f, 4.0f));
  EXPECT_EQ(voxels[1].sample_count, 1);
}

TEST(IlluminationVoxelGrid, RejectsInvalidSettingsAndMismatchedResults) {
  EXPECT_TRUE(GenerateIlluminationVoxelPoints(glm::vec3(0.0f), glm::ivec3(1), 1, 0).empty());
  EXPECT_TRUE(GenerateIlluminationVoxelPoints(glm::vec3(1.0f), glm::ivec3(0), 1, 0).empty());
  EXPECT_TRUE(GenerateIlluminationVoxelPoints(glm::vec3(1.0f), glm::ivec3(1), 0, 0).empty());
  EXPECT_TRUE(
      AggregateIlluminationVoxels({{{0, 0, 0}, {0.0f, 0.0f, 0.0f}}}, {}, glm::vec3(1.0f), glm::ivec3(1)).empty());
}
