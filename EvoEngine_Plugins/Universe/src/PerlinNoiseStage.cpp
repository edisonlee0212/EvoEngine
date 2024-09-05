#include "PerlinNoiseStage.hpp"
#include "glm/gtc/noise.hpp"

void Universe::PerlinNoiseStage::Process(glm::dvec3 point, double previous_result, double& elevation) {
  elevation = 1.0 + glm::perlin(point * 2.0) * 0.2;
  if (elevation < 0.2)
    elevation = 0.2;
}
