#include "Math.hpp"

using namespace evo_engine;

float Random::Uniform(std::mt19937& random_engine, const float min_value, const float max_value) {
  if (min_value == max_value)
    return min_value;
  assert(min_value < max_value);
  std::uniform_real_distribution distribution(min_value, max_value);
  return distribution(random_engine);
}

float Random::Gaussian(std::mt19937& random_engine, const float mean, const float std_dev) {
  if (std_dev == 0.f)
    return mean;
  std::normal_distribution distribution(mean, std_dev);
  return distribution(random_engine);
}

glm::vec2 Random::Disk(std::mt19937& random_engine, const float radius) {
  // Generate radius and angle
  std::uniform_real_distribution<> distribution(0.0, 1.0);
  const float r =
      radius * static_cast<float>(std::sqrt(distribution(random_engine)));  // Adjust radius for uniform area
  const float theta =
      2.0f * glm::pi<float>() * static_cast<float>(distribution(random_engine));  // Uniform angle [0, 2pi]
  // Convert to Cartesian coordinates
  float x = r * std::cos(theta);
  float y = r * std::sin(theta);
  return {x, y};
}
