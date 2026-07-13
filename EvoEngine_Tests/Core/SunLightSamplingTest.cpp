#include <gtest/gtest.h>

#include "SunLightSampling.hpp"

using namespace evo_engine;

TEST(SunLightSampling, ZeroDiameterIsDeterministic) {
  const glm::vec3 axis = glm::normalize(glm::vec3(0.2f, 0.9f, -0.3f));
  EXPECT_LT(glm::distance(SampleSunDirection(axis, 0.0f, 0.0f, 0.0f), axis), 1e-6f);
  EXPECT_LT(glm::distance(SampleSunDirection(axis, 0.0f, 1.0f, 1.0f), axis), 1e-6f);
}

TEST(SunLightSampling, SamplesStayInsideAngularRadius) {
  const glm::vec3 axis(0.0f, 1.0f, 0.0f);
  constexpr float diameter = 0.2f;
  for (int radial = 0; radial <= 10; radial++) {
    for (int azimuth = 0; azimuth <= 10; azimuth++) {
      const glm::vec3 sample = SampleSunDirection(axis, diameter, radial / 10.0f, azimuth / 10.0f);
      EXPECT_NEAR(glm::length(sample), 1.0f, 1e-5f);
      EXPECT_LE(glm::acos(glm::clamp(glm::dot(axis, sample), -1.0f, 1.0f)), diameter * 0.5f + 1e-5f);
    }
  }
}

TEST(SunLightSampling, RadialBoundaryReachesHalfDiameter) {
  const glm::vec3 axis(0.0f, 0.0f, 1.0f);
  constexpr float diameter = 0.2f;
  const glm::vec3 sample = SampleSunDirection(axis, diameter, 1.0f, 0.25f);
  EXPECT_NEAR(glm::acos(glm::clamp(glm::dot(axis, sample), -1.0f, 1.0f)), diameter * 0.5f, 1e-5f);
}
