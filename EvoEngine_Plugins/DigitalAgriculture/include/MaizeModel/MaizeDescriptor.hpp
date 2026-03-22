#pragma once
#include "MaizeSpline.hpp"

namespace digital_agriculture_plugin {
using namespace evo_engine;

/**
 * @brief Structure representing the settings for Maize mesh generation.
 */
struct MaizeMeshGeneratorSettings {
  bool enable_panicle = true;      ///< Enables panicle generation.
  bool enable_stem = true;         ///< Enables stem generation.
  bool enable_leaves = true;       ///< Enables leaves generation.
  bool enable_leaf_sheath = true;  ///< Enables leaf sheath generation.
  int single_leaf_index = -1;      ///< Index for a single leaf (-1 for all).
  bool bottom_face = true;         ///< Enables bottom face generation.
  bool leaf_separated = false;     ///< Determines if leaves are separate objects.
  float leaf_thickness = 0.001f;   ///< Thickness of leaves.

  /**
   * @brief Inspects the settings in the editor.
   * @param editor_layer Shared pointer to the editor layer.
   * @return True if content is not modified, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
};

/**
 * @brief Class representing the panicle descriptor for Maize.
 */
class MaizePanicleDescriptor {
 public:
  glm::vec3 panicle_size = glm::vec3(0, 0, 0);  ///< Size of the panicle.
  int seed_amount = 0;                          ///< Number of seeds.
  float seed_radius = 0.002f;                   ///< Radius of seeds.

  /**
   * @brief Inspects the panicle descriptor in the editor.
   * @param editor_layer Shared pointer to the editor layer.
   * @return True if content is not modified, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);

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
 * @brief Class representing the descriptor for a Maize stem.
 */
class MaizeStemDescriptor {
 public:
  MaizeSpline spline;  ///< Spline describing the stem shape. (Using SorghumSpline for now as underlying data)

  /**
   * @brief Inspects the stem descriptor in the editor.
   * @param editor_layer Shared pointer to the editor layer.
   * @return True if content is not modified, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);

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
 * @brief Class representing the descriptor for a Maize leaf.
 */
class MaizeLeafDescriptor {
 public:
  int index = 0;         ///< Index of the leaf.
  float current_growth = 1.0f; ///< Current growth factor of the leaf.
  MaizeSpline spline;  ///< Spline describing the leaf shape. (Using SorghumSpline)

  /**
   * @brief Inspects the leaf descriptor in the editor.
   * @param editor_layer Shared pointer to the editor layer.
   * @return True if content is not modified, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);

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
                        const MaizeMeshGeneratorSettings& mesh_generator_settings,
                        bool current_bottom_face = false) const;
};

/**
 * @brief Class representing the complete Maize descriptor.
 */
class MaizeDescriptor : public IAsset {
 public:
  MaizePanicleDescriptor panicle;           ///< Descriptor for the panicle.
  MaizeStemDescriptor stem;                 ///< Descriptor for the stem.
  std::vector<MaizeLeafDescriptor> leaves;  ///< List of leaf descriptors.

  /**
   * @brief Inspects the Maize descriptor in the editor.
   * @param editor_layer Shared pointer to the editor layer.
   * @return True if content is not modified, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Serializes the Maize descriptor to YAML.
   * @param out YAML emitter.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the Maize descriptor from YAML.
   * @param in YAML node.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Creates an entity representing the Maize model.
   * @param name Name of the entity.
   * @return The created entity.
   */
  [[maybe_unused]] Entity CreateEntity(const std::string& name) const;

  /**
   * @brief Generates a thumbnail texture representing the Maize model.
   * @return Shared pointer to the generated texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;

  /**
   * @brief Imports a prediction file containing structural data of the Maize model.
   * @param yaml_path Path to the YAML file.
   * @return Optional containing structured data if successful.
   */
  [[nodiscard]] std::optional<std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>>> ImportPrediction(
      const std::filesystem::path& yaml_path);
};

}  // namespace digital_agriculture_plugin
