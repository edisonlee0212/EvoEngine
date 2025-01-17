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

class ActivationFunction {
 public:
  static float Sigmoid(float a, float b, float offset, float speed, float x);
  static float SoftSign(float a, float b, float offset, float speed, float x);
  static float Tanh(float a, float b, float offset, float speed, float x);
  static float Sigmoid(float offset, float speed, float x);
  static float SoftSign(float offset, float speed, float x);
  static float Tanh(float offset, float speed, float x);
};

}  // namespace evo_engine