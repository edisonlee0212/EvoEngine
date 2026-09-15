#include "Utilities.hpp"
#include "Console.hpp"

using namespace evo_engine;

std::string FileUtils::LoadFileAsString(const std::filesystem::path& path) {
  std::ifstream file;
  file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  try {
    // open files
    file.open(path);
    std::stringstream stream;
    // read file's buffer contents into streams
    stream << file.rdbuf();
    // close file handlers
    file.close();
    // convert stream into string
    return stream.str();
  } catch (const std::ifstream::failure& e) {
    EVOENGINE_ERROR("Failed to load: " + path.string());
    throw;
  }
}

void SphereMeshGenerator::Icosahedron(std::vector<glm::vec3>& vertices, std::vector<glm::uvec3>& triangles) {
  vertices.clear();
  triangles.clear();

  const float phi = (1.0f + glm::sqrt(5.0f)) * 0.5f;  // golden ratio
  const float a = 1.0f;
  const float b = 1.0f / phi;

  // add vertices
  vertices.push_back(glm::normalize(glm::vec3(0, b, -a)));
  vertices.push_back(glm::normalize(glm::vec3(b, a, 0)));
  vertices.push_back(glm::normalize(glm::vec3(-b, a, 0)));
  vertices.push_back(glm::normalize(glm::vec3(0, b, a)));
  vertices.push_back(glm::normalize(glm::vec3(0, -b, a)));
  vertices.push_back(glm::normalize(glm::vec3(-a, 0, b)));
  vertices.push_back(glm::normalize(glm::vec3(0, -b, -a)));
  vertices.push_back(glm::normalize(glm::vec3(a, 0, -b)));
  vertices.push_back(glm::normalize(glm::vec3(a, 0, b)));
  vertices.push_back(glm::normalize(glm::vec3(-a, 0, -b)));
  vertices.push_back(glm::normalize(glm::vec3(b, -a, 0)));
  vertices.push_back(glm::normalize(glm::vec3(-b, -a, 0)));

  // add triangles
  triangles.emplace_back(3, 2, 1);
  triangles.emplace_back(2, 3, 4);
  triangles.emplace_back(6, 5, 4);
  triangles.emplace_back(5, 9, 4);
  triangles.emplace_back(8, 7, 1);
  triangles.emplace_back(7, 10, 1);
  triangles.emplace_back(12, 11, 5);
  triangles.emplace_back(11, 12, 7);
  triangles.emplace_back(10, 6, 3);
  triangles.emplace_back(6, 10, 12);
  triangles.emplace_back(9, 8, 2);
  triangles.emplace_back(8, 9, 11);
  triangles.emplace_back(3, 6, 4);
  triangles.emplace_back(9, 2, 4);
  triangles.emplace_back(10, 3, 1);
  triangles.emplace_back(2, 8, 1);
  triangles.emplace_back(12, 10, 7);
  triangles.emplace_back(8, 11, 7);
  triangles.emplace_back(6, 12, 5);
  triangles.emplace_back(11, 9, 5);

  for (auto& i : triangles) {
    i.x -= 1;
    i.y -= 1;
    i.z -= 1;
  }
}
