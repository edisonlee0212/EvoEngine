
#pragma once

namespace evo_engine {

/**
 * @struct Vertex
 * @brief Represents a basic vertex with position, normal, tangent, color, texture coordinates, and additional vertex
 * information.
 */
struct Vertex {
  glm::vec3 position = glm::vec3(0.0f);     /**< The position of the vertex in 3D space. */
  float vertex_info1 = 0.0f;                /**< Additional vertex-specific information (float). */
  glm::vec3 normal = glm::vec3(0.0f);       /**< The normal vector of the vertex. */
  float vertex_info2 = 0.0f;                /**< Additional vertex-specific information (float). */
  glm::vec3 tangent = glm::vec3(0.0f);      /**< The tangent vector of the vertex. */
  float vertex_info3 = 0.0f;                /**< Additional vertex-specific information (float). */
  glm::vec4 color = glm::vec4(1.0f);        /**< The color of the vertex in RGBA format. */
  glm::vec2 tex_coord = glm::vec2(0.0f);    /**< The texture coordinates of the vertex. */
  glm::vec2 vertex_info4 = glm::vec2(0.0f); /**< Additional vertex-specific information (2D vector). */
  glm::vec2 tex_coord_1 = glm::vec2(0.0f);  /**< The secondary texture coordinates of the vertex. */
  glm::vec2 padding = glm::vec2(0.0f);
};

/**
 * @struct SkinnedVertex
 * @brief Represents a skinned vertex used in animations, with position, normal, tangent, skinning weights and IDs, and
 * other attributes.
 */
struct SkinnedVertex {
  glm::vec3 position = glm::vec3(0.0f);     /**< The position of the vertex in 3D space. */
  float vertex_info1 = 0.0f;                /**< Additional vertex-specific information (float). */
  glm::vec3 normal = glm::vec3(0.0f);       /**< The normal vector of the vertex. */
  float vertex_info2 = 0.0f;                /**< Additional vertex-specific information (float). */
  glm::vec3 tangent = glm::vec3(0.0f);      /**< The tangent vector of the vertex. */
  float vertex_info3 = 0.0f;                /**< Additional vertex-specific information (float). */
  glm::vec4 color = glm::vec4(1.0f);        /**< The color of the vertex in RGBA format. */
  glm::vec2 tex_coord = glm::vec2(0.0f);    /**< The texture coordinates of the vertex. */
  glm::vec2 vertex_info4 = glm::vec2(0.0f); /**< Additional vertex-specific information (2D vector). */

  glm::ivec4 bond_id = {};  /**< Bone IDs influencing this vertex (set 1). */
  glm::vec4 weight = {};    /**< Weights corresponding to the bone IDs (set 1). */
  glm::ivec4 bond_id2 = {}; /**< Bone IDs influencing this vertex (set 2). */
  glm::vec4 weight2 = {};   /**< Weights corresponding to the bone IDs (set 2). */

  glm::vec2 tex_coord_1 = glm::vec2(0.0f); /**< The secondary texture coordinates of the vertex. */
  glm::vec2 padding = glm::vec2(0.0f);
};

/**
 * @struct StrandPoint
 * @brief Represents a point on a strand or hair-like structure, with position, thickness, normal, texture coordinate,
 * and color.
 */
struct StrandPoint {
  glm::vec3 position = glm::vec3(0.0f); /**< The position of the strand point in 3D space. */
  float thickness = 0.0f;               /**< The thickness of the strand at this point. */
  glm::vec3 normal = glm::vec3(0.0f);   /**< The normal vector at the strand point. */
  float tex_coord = 0.0f;               /**< The texture coordinate at the strand point. */
  glm::vec4 color = glm::vec4(1.0f);    /**< The color at the strand point in RGBA format. */
};

}  // namespace evo_engine
