#pragma once
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_ENABLE_EXPERIMENTAL

#include "glm/detail/type_half.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/noise.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtc/random.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/closest_point.hpp"
#include "glm/gtx/matrix_decompose.hpp"
#include "glm/gtx/rotate_vector.hpp"
#include "glm/gtx/transform.hpp"

namespace evo_engine {

class Random {
 public:
  static float Uniform(std::mt19937& random_engine, float min_value, float max_value);
  static float Gaussian(std::mt19937& random_engine, float mean, float std_dev);
  static glm::vec2 Disk(std::mt19937& random_engine, float radius);
};

}  // namespace evo_engine