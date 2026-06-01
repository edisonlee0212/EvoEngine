
#pragma once
#include "SorghumSpline.hpp"

namespace digital_agriculture_package {
using namespace evo_engine;

/**
 * @brief Structure representing the settings for Sorghum mesh generation.
 */
struct SorghumMeshGeneratorSettings {
  bool enable_panicle = true;      ///< Enables panicle generation.
  bool enable_stem = true;         ///< Enables stem generation.
  bool enable_leaves = true;       ///< Enables leaves generation.
  bool enable_leaf_sheath = true;  ///< Enables leaf sheath generation.
  int single_leaf_index = -1;      ///< Index for a single leaf (-1 for all).
  bool bottom_face = true;         ///< Enables bottom face generation.
  bool leaf_separated = true;      ///< Determines if leaves are separate objects.
  float leaf_thickness = 0.001f;   ///< Thickness of leaves.
};

/**
 * @brief Class representing the panicle descriptor for Sorghum.
 */
class SorghumPanicleDescriptor {
 public:
  glm::vec3 panicle_size = glm::vec3(0, 0, 0);  ///< Size of the panicle.
  int seed_amount = 0;                          ///< Number of seeds.
  float seed_radius = 0.002f;                   ///< Radius of seeds.

  /**
   * @brief Serializes the panicle descriptor to YAML.
   * @param out YAML emitter.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes the panicle descriptor from YAML.
   * @param in YAML node.
   */
  void Deserialize(const YAML::Node& in);

  /**
   * @brief Generates the geometry for the panicle.
   * @param stem_tip Position of the stem tip.
   * @param vertices Vector to store generated vertices.
   * @param indices Vector to store generated indices.
   */
  void GenerateGeometry(const glm::vec3& stem_tip, std::vector<Vertex>& vertices,
                        std::vector<unsigned int>& indices) const;

  /**
   * @brief Generates the geometry with particle information.
   * @param stem_tip Position of the stem tip.
   * @param vertices Vector to store generated vertices.
   * @param indices Vector to store generated indices.
   * @param particle_info_list Shared pointer to particle information list.
   */
  void GenerateGeometry(const glm::vec3& stem_tip, std::vector<Vertex>& vertices, std::vector<unsigned int>& indices,
                        const std::shared_ptr<ParticleInfoList>& particle_info_list) const;
};

/**
 * @brief Class representing the descriptor for a Sorghum stem.
 */
class SorghumStemDescriptor {
 public:
  SorghumSpline spline;  ///< Spline describing the stem shape.

  /**
   * @brief Serializes the stem descriptor to YAML.
   * @param out YAML emitter.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes the stem descriptor from YAML.
   * @param in YAML node.
   */
  void Deserialize(const YAML::Node& in);

  /**
   * @brief Generates the geometry for the stem.
   * @param vertices Vector to store generated vertices.
   * @param indices Vector to store generated indices.
   */
  void GenerateGeometry(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
};

/**
 * @brief Class representing the descriptor for a Sorghum leaf.
 */
class SorghumLeafDescriptor {
 public:
  int index = 0;         ///< Index of the leaf.
  SorghumSpline spline;  ///< Spline describing the leaf shape.

  /**
   * @brief Serializes the leaf descriptor to YAML.
   * @param out YAML emitter.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes the leaf descriptor from YAML.
   * @param in YAML node.
   */
  void Deserialize(const YAML::Node& in);

  /**
   * @brief Generates the geometry for the leaf.
   * @param vertices Vector to store generated vertices.
   * @param indices Vector to store generated indices.
   * @param mesh_generator_settings Mesh generator settings.
   * @param current_bottom_face Whether the current leaf has a bottom face.
   */
  void GenerateGeometry(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices,
                        const SorghumMeshGeneratorSettings& mesh_generator_settings,
                        bool current_bottom_face = false) const;
};

/**
 * @brief Class representing the complete Sorghum descriptor.
 */
class SorghumDescriptor : public IAsset {
 public:
  SorghumPanicleDescriptor panicle;           ///< Descriptor for the panicle.
  SorghumStemDescriptor stem;                 ///< Descriptor for the stem.
  std::vector<SorghumLeafDescriptor> leaves;  ///< List of leaf descriptors.

  /**
   * @brief Creates an entity representing the Sorghum model.
   * @param name Name of the entity.
   * @return The created entity.
   */
  [[maybe_unused]] Entity CreateEntity(const std::string& name) const;

  /**
   * @brief Generates a thumbnail texture representing the Sorghum model.
   * @return Shared pointer to the generated texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture();

  /**
   * @brief Imports a prediction file containing structural data of the Sorghum model.
   * @param yaml_path Path to the YAML file.
   * @return Optional containing structured data if successful.
   */
  [[nodiscard]] std::optional<std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>>> ImportPrediction(
      const std::filesystem::path& yaml_path);


  void ExtractTraits() const;

private:
  float CalculateLeafArea(int leaf_index) const;
  float CalculateLeafLength(int leaf_index) const;
  float CalculateLeafWidth(int leaf_index) const;
  std::vector<float> CalculateInterNodeLengths() const;
};

}  // namespace digital_agriculture_package
