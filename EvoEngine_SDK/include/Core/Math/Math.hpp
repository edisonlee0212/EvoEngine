
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

/**
 * @brief A utility class for generating random values.
 */
class Random {
 public:
  /**
   * @brief Generates a uniform random float value within the given range.
   * @param random_engine A reference to a random number generator.
   * @param min_value The minimum value of the range.
   * @param max_value The maximum value of the range.
   * @return A random float value within the range [min_value, max_value].
   */
  static float Uniform(std::mt19937& random_engine, float min_value, float max_value);

  /**
   * @brief Generates a random float value following a Gaussian (normal) distribution.
   * @param random_engine A reference to a random number generator.
   * @param mean The mean of the Gaussian distribution.
   * @param std_dev The standard deviation of the Gaussian distribution.
   * @return A random float value based on the specified Gaussian distribution.
   */
  static float Gaussian(std::mt19937& random_engine, float mean, float std_dev);

  /**
   * @brief Generates a random 2D point within a disk of a given radius.
   * @param random_engine A reference to a random number generator.
   * @param radius The radius of the disk.
   * @return A random 2D point within the disk.
   */
  static glm::vec2 Disk(std::mt19937& random_engine, float radius);
};

/**
 * @brief A collection of mathematical activation functions.
 */
class ActivationFunction {
 public:
  /**
   * @brief Computes the Sigmoid activation function.
   * @param a The scaling factor for the output.
   * @param b An additive constant for the output.
   * @param offset The offset to apply to the input.
   * @param speed The speed or steepness of the function.
   * @param x The input value.
   * @return The output of the Sigmoid function.
   */
  static float Sigmoid(float a, float b, float offset, float speed, float x);

  /**
   * @brief Computes the SoftSign activation function.
   * @param a The scaling factor for the output.
   * @param b An additive constant for the output.
   * @param offset The offset to apply to the input.
   * @param speed The speed or steepness of the function.
   * @param x The input value.
   * @return The output of the SoftSign function.
   */
  static float SoftSign(float a, float b, float offset, float speed, float x);

  /**
   * @brief Computes the Tanh (hyperbolic tangent) activation function.
   * @param a The scaling factor for the output.
   * @param b An additive constant for the output.
   * @param offset The offset to apply to the input.
   * @param speed The speed or steepness of the function.
   * @param x The input value.
   * @return The output of the Tanh function.
   */
  static float Tanh(float a, float b, float offset, float speed, float x);

  /**
   * @brief Computes the Sigmoid activation function with fewer parameters.
   * @param offset The offset to apply to the input.
   * @param speed The speed or steepness of the function.
   * @param x The input value.
   * @return The output of the Sigmoid function.
   */
  static float Sigmoid(float offset, float speed, float x);

  /**
   * @brief Computes the SoftSign activation function with fewer parameters.
   * @param offset The offset to apply to the input.
   * @param speed The speed or steepness of the function.
   * @param x The input value.
   * @return The output of the SoftSign function.
   */
  static float SoftSign(float offset, float speed, float x);

  /**
   * @brief Computes the Tanh (hyperbolic tangent) activation function with fewer parameters.
   * @param offset The offset to apply to the input.
   * @param speed The speed or steepness of the function.
   * @param x The input value.
   * @return The output of the Tanh function.
   */
  static float Tanh(float offset, float speed, float x);
};

}  // namespace evo_engine
