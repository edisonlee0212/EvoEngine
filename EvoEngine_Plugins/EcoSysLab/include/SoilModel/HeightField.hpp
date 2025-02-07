
#pragma once

#include <Vertex.hpp>
#include <glm/glm.hpp>
#include <vector>

#include "Noises.hpp"
using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @brief Represents a height field generated using procedural noise.
 *
 * This class provides functionality to generate and manipulate a height field
 * based on procedural noise, as well as to generate corresponding mesh data.
 */
class HeightField : public IAsset {
 public:
  /// 2D noise generator for creating procedural height values.
  Noise2D noises_2d;

  /// Precision level for height field computations.
  int precision_level = 2;

  /**
   * @brief Retrieves the height value at a given 2D position.
   * @param position The 2D position in the height field.
   * @return The height value at the specified position.
   */
  [[nodiscard]] float GetValue(const glm::vec2& position) const;

  /**
   * @brief Applies a random offset to the noise function.
   * @param min The minimum offset value.
   * @param max The maximum offset value.
   */
  void RandomOffset(float min, float max);

  /**
   * @brief Handles editor inspection for the height field.
   *
   * This function will be invoked by the editor layer to inspect and modify the height field.
   * @param editorLayer A shared pointer to the editor layer.
   * @return True if the asset's content remains unmodified during inspection, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

  /**
   * @brief Serializes the height field data into a YAML emitter.
   * @param out The YAML emitter used to store the serialized data.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the height field data from a YAML node.
   * @param in The YAML node containing the serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Generates a thumbnail texture representing the height field.
   * @return A shared pointer to the generated texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;

  /**
   * @brief Generates a mesh representation of the height field.
   *
   * This function generates a mesh using the given parameters, storing the
   * resulting vertex and triangle data in the provided vectors.
   *
   * @param start The starting position of the mesh.
   * @param resolution The resolution of the mesh grid.
   * @param unitSize The size of each unit in the grid.
   * @param vertices The vector where generated vertices will be stored.
   * @param triangles The vector where generated triangle indices will be stored.
   * @param xDepth The depth scaling factor in the X direction (default: 1.0f).
   * @param zDepth The depth scaling factor in the Z direction (default: 1.0f).
   */
  void GenerateMesh(const glm::vec2& start, const glm::uvec2& resolution, float unitSize, std::vector<Vertex>& vertices,
                    std::vector<glm::uvec3>& triangles, float xDepth = 1.0f, float zDepth = 1.0f) const;
};

}  // namespace eco_sys_lab_plugin
