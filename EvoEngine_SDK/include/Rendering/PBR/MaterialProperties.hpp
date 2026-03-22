
#pragma once
#include <glm/glm.hpp>

namespace evo_engine {

/**
 * @struct MaterialProperties
 * @brief Represents the physical material properties used in rendering.
 *
 * This structure contains various parameters used to define the appearance
 * of a material, including colors and factors affecting surface reflections,
 * subsurface scattering, and transmission.
 */
struct MaterialProperties {
  /**
   * @brief Base color of the material.
   */
  glm::vec3 albedo_color = glm::vec3(1.0f);

  /**
   * @brief Color of light scattered below the surface of the material.
   */
  glm::vec3 subsurface_color = glm::vec3(1.0f);

  /**
   * @brief Factor determining the strength of subsurface scattering.
   */
  float subsurface_factor = 0.0f;

  /**
   * @brief Radii that determine how far light scatters beneath the surface in each RGB channel.
   */
  glm::vec3 subsurface_radius = glm::vec3(1.0f, 0.2f, 0.1f);

  /**
   * @brief Factor determining the metallic nature of the material.
   *
   * Value ranges between 0.0 (non-metal) and 1.0 (fully metallic).
   */
  float metallic = 0.1f;

  /**
   * @brief Specular reflection intensity.
   */
  float specular = 0.5f;

  /**
   * @brief Tint applied to the specular reflection.
   *
   * Value ranges between 0.0 (no tint) and 1.0 (full tint).
   */
  float specular_tint = 0.0f;

  /**
   * @brief Roughness of the material's surface.
   *
   * Value ranges between 0.0 (smooth) and 1.0 (rough).
   */
  float roughness = 0.3f;

  /**
   * @brief Intensity of the sheen effect, often used for cloth-like surfaces.
   */
  float sheen = 0.0f;

  /**
   * @brief Tint of the sheen effect.
   *
   * Value ranges between 0.0 (no tint) and 1.0 (tinted).
   */
  float sheen_tint = 0.5f;

  /**
   * @brief Intensity of the clear coat layer on the material.
   */
  float clear_coat = 0.0f;

  /**
   * @brief Roughness of the clear coat layer.
   */
  float clear_coat_roughness = 0.03f;

  /**
   * @brief Index of refraction for the material.
   *
   * This determines bending of light when it passes through the material.
   */
  float ior = 1.45f;

  /**
   * @brief Factor defining the material's ability to transmit light.
   */
  float transmission = 0.0f;

  /**
   * @brief Roughness of transmitted light through the material.
   */
  float transmission_roughness = 0.0f;

  /**
   * @brief Amount of light emitted by the material.
   */
  float emission = 0.0f;

  /**
   * @brief Intensity/scale of displacement mapping.
   *
   * Controls how strongly the displacement map offsets geometry.
   */
  float displacement_intensity = 0.1f;
};

}  // namespace evo_engine
