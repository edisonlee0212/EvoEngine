
#pragma once
#include "RandomSampler.hpp"

namespace evo_engine {
/**
 * @class FileUtils
 * @brief A utility class for file handling operations such as loading, opening, and saving files.
 */
class EVOENGINE_API FileUtils {
 public:
  /**
   * @brief Loads the contents of a file as a string.
   * @param path The file path. Defaults to an empty path if not provided.
   * @return The contents of the file as a string.
   */
  static std::string LoadFileAsString(const std::filesystem::path& path = "");
};

/**
 * @class SphereMeshGenerator
 * @brief A utility class for generating mesh data for 3D spheres.
 */
class EVOENGINE_API SphereMeshGenerator {
 public:
  /**
   * @brief Generates an icosahedron mesh.
   * @param vertices The output vector of vertices.
   * @param triangles The output vector of triangle indices.
   */
  static void Icosahedron(std::vector<glm::vec3>& vertices, std::vector<glm::uvec3>& triangles);
};

}  // namespace evo_engine
