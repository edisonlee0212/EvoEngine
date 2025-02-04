
#pragma once
#include <glm/glm.hpp>

namespace evo_engine {

/**
 * @brief Structure representing hit information in a physics or graphics engine.
 */
struct HitInfo {
  glm::vec3 position = glm::vec3(0.0f);  /**< Position of the hit point in world coordinates. */
  glm::vec3 normal = glm::vec3(0.0f);    /**< Normal vector at the hit surface. */
  glm::vec3 tangent = glm::vec3(0.0f);   /**< Tangent vector at the hit surface. */
  glm::vec4 color = glm::vec4(1.0f);     /**< Color information at the hit point. */
  glm::vec2 tex_coord = glm::vec2(0.0f); /**< Texture coordinates at the hit point. */
  glm::vec3 data = glm::vec4(0.0f);      /**< Additional data vector for user-defined purposes. */
  glm::vec2 data2 = glm::vec4(0.0f);     /**< Extra data for user-specific needs. */
};

/**
 * @brief Structure representing a sampled point in a point-cloud or ray tracing system.
 */
struct PointCloudSample {
  // Input
  glm::vec3 direction = glm::vec3(0.0f); /**< Ray or sampling direction vector. */
  glm::vec3 start = glm::vec3(0.0f);     /**< Starting point of the ray or sample. */

  // Output
  uint64_t handle = 0; /**< Handle or identifier for the point-cloud object. */
  bool hit = false;    /**< Flag indicating whether a hit occurred. */

  HitInfo hit_info; /**< Detailed information about the hit, if one occurred. */
};

}  // namespace evo_engine
