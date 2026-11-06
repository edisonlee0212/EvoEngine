#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include <limits>
#include "PrefabField.hpp"

using namespace evo_engine;

TEST(PrefabField, BuildsCenteredRowMajorGrid) {
  PrefabField field;
  field.rows = 2;
  field.columns = 3;
  field.spacing = glm::vec2(0.5f, 1.25f);
  field.seed = 12;

  const auto placements = field.BuildPlacements(1);

  ASSERT_EQ(placements.size(), 6);
  EXPECT_EQ(placements.front().asset_index, 0);
  EXPECT_EQ(placements.front().position, glm::vec3(-0.5f, 0.0f, -0.625f));
  EXPECT_EQ(placements[2].position, glm::vec3(0.5f, 0.0f, -0.625f));
  EXPECT_EQ(placements.back().position, glm::vec3(0.5f, 0.0f, 0.625f));
}

TEST(PrefabField, UsesSeedDeterministically) {
  PrefabField field;
  field.rows = 3;
  field.columns = 4;
  field.seed = 42;
  field.random_yaw_degrees = 30.0f;

  const auto first = field.BuildPlacements(5);
  const auto second = field.BuildPlacements(5);

  ASSERT_EQ(first.size(), second.size());
  for (size_t index = 0; index < first.size(); ++index) {
    EXPECT_EQ(first[index].asset_index, second[index].asset_index);
    EXPECT_EQ(first[index].position, second[index].position);
    EXPECT_FLOAT_EQ(first[index].yaw_degrees, second[index].yaw_degrees);
    EXPECT_GE(first[index].yaw_degrees, -30.0f);
    EXPECT_LE(first[index].yaw_degrees, 30.0f);
  }
}

TEST(PrefabField, SamplesLargeAssetLibraryWithoutReplacement) {
  PrefabField field;
  field.rows = 4;
  field.columns = 10;
  field.seed = 7;

  const auto placements = field.BuildPlacements(256);
  std::unordered_set<size_t> selected_assets;
  for (const auto& placement : placements)
    selected_assets.insert(placement.asset_index);

  EXPECT_EQ(placements.size(), 40);
  EXPECT_EQ(selected_assets.size(), placements.size());
}

TEST(PrefabField, RejectsEmptyInputs) {
  PrefabField field;
  EXPECT_TRUE(field.BuildPlacements(0).empty());
  field.rows = 0;
  EXPECT_TRUE(field.BuildPlacements(2).empty());
  field.rows = 2;
  field.columns = -1;
  EXPECT_TRUE(field.BuildPlacements(2).empty());
}

TEST(PrefabField, DefaultRoiSelectsEntireField) {
  PrefabField field;
  field.rows = 4;
  field.columns = 10;
  const auto region = field.GetMeasurementRegion();
  EXPECT_FLOAT_EQ(field.illumination_roi, 1.0f);
  EXPECT_EQ(region.row_start, 0);
  EXPECT_EQ(region.column_start, 0);
  EXPECT_EQ(region.rows, 4);
  EXPECT_EQ(region.columns, 10);
  EXPECT_TRUE(region.Contains(0, 0));
  EXPECT_TRUE(region.Contains(3, 9));
  EXPECT_FALSE(region.Contains(4, 9));
}

TEST(PrefabField, HalfRoiSelectsCentralFortyOfOneHundredSixtyPlants) {
  PrefabField field;
  field.rows = 8;
  field.columns = 20;
  field.illumination_roi = 0.5f;
  field.seed = 42;
  const auto region = field.GetMeasurementRegion();
  EXPECT_EQ(region.row_start, 2);
  EXPECT_EQ(region.column_start, 5);
  EXPECT_EQ(region.rows, 4);
  EXPECT_EQ(region.columns, 10);
  EXPECT_FALSE(region.Contains(1, 5));
  EXPECT_FALSE(region.Contains(2, 4));
  EXPECT_FALSE(region.Contains(6, 14));

  const auto placements = field.BuildPlacements(256);
  ASSERT_EQ(placements.size(), 160);
  size_t measured_count = 0;
  for (size_t index = 0; index < placements.size(); ++index)
    if (region.Contains(static_cast<int>(index) / field.columns, static_cast<int>(index) % field.columns))
      ++measured_count;
  EXPECT_EQ(measured_count, 40);

  field.illumination_roi = 1.0f;
  const auto full = field.BuildPlacements(256);
  for (size_t index = 0; index < full.size(); ++index) {
    EXPECT_EQ(placements[index].asset_index, full[index].asset_index);
    EXPECT_EQ(placements[index].position, full[index].position);
  }
}

TEST(PrefabField, RoiRoundsToCellsAndRetainsAtLeastOneCell) {
  PrefabField field;
  field.rows = 5;
  field.columns = 9;
  field.illumination_roi = 0.5f;
  auto region = field.GetMeasurementRegion();
  EXPECT_EQ(region.rows, 3);
  EXPECT_EQ(region.columns, 5);
  EXPECT_EQ(region.row_start, 1);
  EXPECT_EQ(region.column_start, 2);
  field.illumination_roi = 0.001f;
  region = field.GetMeasurementRegion();
  EXPECT_EQ(region.rows, 1);
  EXPECT_EQ(region.columns, 1);
  EXPECT_EQ(region.row_start, 2);
  EXPECT_EQ(region.column_start, 4);
}

TEST(PrefabField, RoiRejectsInvalidFractionsAndEmptyGrids) {
  PrefabField field;
  for (const float roi :
       {0.0f, -1.0f, 1.1f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
    field.illumination_roi = roi;
    const auto region = field.GetMeasurementRegion();
    EXPECT_EQ(region.rows, 0);
    EXPECT_EQ(region.columns, 0);
    EXPECT_FALSE(region.Contains(0, 0));
  }
  field.illumination_roi = 1.0f;
  field.rows = 0;
  EXPECT_EQ(field.GetMeasurementRegion().rows, 0);
}

TEST(PrefabField, AggregatesPerModelIllumination) {
  std::vector<PrefabFieldIlluminationResult> models = {
      {0, 7, 0, 0, 2.0f, glm::vec3(4.0f, 2.0f, 0.0f), glm::vec3(2.0f, 1.0f, 0.0f)},
      {1, 9, 0, 1, 1.0f, glm::vec3(1.0f, 4.0f, 3.0f), glm::vec3(1.0f, 4.0f, 3.0f)},
  };

  const auto summary = PrefabField::SummarizeIllumination(std::move(models));

  ASSERT_EQ(summary.models.size(), 2);
  EXPECT_FLOAT_EQ(summary.total_area, 3.0f);
  EXPECT_EQ(summary.total_flux, glm::vec3(5.0f, 6.0f, 3.0f));
  EXPECT_EQ(summary.area_weighted_average_flux, glm::vec3(5.0f / 3.0f, 2.0f, 1.0f));
  EXPECT_EQ(summary.mean_model_average_flux, glm::vec3(1.5f, 2.5f, 1.5f));
}

TEST(PrefabField, AggregatesEmptyIllumination) {
  const auto summary = PrefabField::SummarizeIllumination({});
  EXPECT_TRUE(summary.models.empty());
  EXPECT_FLOAT_EQ(summary.total_area, 0.0f);
  EXPECT_EQ(summary.total_flux, glm::vec3(0.0f));
  EXPECT_EQ(summary.area_weighted_average_flux, glm::vec3(0.0f));
  EXPECT_EQ(summary.mean_model_average_flux, glm::vec3(0.0f));
  EXPECT_TRUE(summary.organs.empty());
}

TEST(PrefabField, AggregatesOrgansByAreaAndRetainsUnknownMeshNames) {
  PrefabFieldIlluminationResult first;
  first.organs = {{"leaf", {2.0f, glm::vec3(12.0f), glm::vec3(3.0f)}},
                  {"stem", {1.0f, glm::vec3(1.0f), glm::vec3(1.0f)}}};
  PrefabFieldIlluminationResult second;
  second.organs = {{"leaf", {1.0f, glm::vec3(12.0f), glm::vec3(6.0f)}},
                   {"petiole", {0.5f, glm::vec3(0.25f), glm::vec3(0.5f)}},
                   {"flower", {0.25f, glm::vec3(0.5f), glm::vec3(2.0f)}},
                   {"empty", {}}};

  const auto summary = PrefabField::SummarizeIllumination({first, second});

  ASSERT_EQ(summary.organs.size(), 5);
  EXPECT_FLOAT_EQ(summary.organs.at("leaf").total_area, 3.0f);
  EXPECT_EQ(summary.organs.at("leaf").total_flux, glm::vec3(24.0f));
  EXPECT_EQ(summary.organs.at("leaf").average_flux, glm::vec3(4.0f));
  EXPECT_EQ(summary.organs.at("petiole").average_flux, glm::vec3(0.5f));
  EXPECT_EQ(summary.organs.at("stem").total_flux, glm::vec3(1.0f));
  EXPECT_EQ(summary.organs.at("flower").average_flux, glm::vec3(2.0f));
  EXPECT_EQ(summary.organs.at("empty").average_flux, glm::vec3(0.0f));
}

TEST(PrefabField, PreservesFaceMeanWhenIncidentFluxIncludesBothLeafFaces) {
  // A double-sided leaf (area 2, face mean 3) and a one-sided stem (area 1, mean 2).
  const auto summary = PrefabField::SummarizeIllumination({
      {0, 0, 0, 0, 2.0f, glm::vec3(12.0f), glm::vec3(3.0f)},
      {1, 1, 0, 1, 1.0f, glm::vec3(2.0f), glm::vec3(2.0f)},
  });

  EXPECT_FLOAT_EQ(summary.total_area, 3.0f);
  EXPECT_EQ(summary.total_flux, glm::vec3(14.0f));
  EXPECT_EQ(summary.area_weighted_average_flux, glm::vec3(8.0f / 3.0f));
  EXPECT_EQ(summary.mean_model_average_flux, glm::vec3(2.5f));
}
