
#pragma once
#include "Curve.hpp"
#include "Plot2D.hpp"
#include "SorghumDescriptor.hpp"

namespace digital_agriculture_package {
using namespace evo_engine;
#pragma region States

/**
 * @brief Represents the state modes available.
 */
enum class StateMode { Default, CubicBezier };

/**
 * @brief Represents the state of a sorghum panicle.
 */
struct SorghumPanicleState {
  glm::vec3 panicle_size = glm::vec3(0, 0, 0);  ///< Size of the panicle.
  int seed_amount = 0;                          ///< Number of seeds in the panicle.
  float seed_radius = 0.002f;                   ///< Radius of seeds in the panicle.

  bool saved = false;  ///< Indicates whether the state has been saved.

  /// @brief Default constructor.
  SorghumPanicleState();

  /**
   * @brief Serializes the state to a YAML emitter.
   * @param out The YAML emitter to serialize to.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes the state from a YAML node.
   * @param in The YAML node to deserialize from.
   */
  void Deserialize(const YAML::Node& in);

  /**
   * @brief Applies the state to a SorghumPanicleDescriptor.
   * @param target_sorghum_panicle_descriptor Target panicle descriptor.
   */
  void Apply(SorghumPanicleDescriptor& target_sorghum_panicle_descriptor) const;
};

/**
 * @brief Represents the state of a sorghum stem.
 */
struct SorghumStemState {
  BezierSpline spline;              ///< Bezier spline defining the stem shape.
  glm::vec3 direction = {0, 1, 0};  ///< Growth direction of the stem.
  Plot2D<float> width_along_stem;   ///< Width variation along the stem.
  float length = 0;                 ///< Length of the stem.

  bool saved = false;  ///< Indicates whether the state has been saved.

  /// @brief Default constructor.
  SorghumStemState();

  /**
   * @brief Gets a point along the stem.
   * @param point Position along the stem.
   * @return A vec3 representing the point's position.
   */
  [[nodiscard]] glm::vec3 GetPoint(float point) const;

  /**
   * @brief Serializes the state to a YAML emitter.
   * @param out The YAML emitter to serialize to.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes the state from a YAML node.
   * @param in The YAML node to deserialize from.
   */
  void Deserialize(const YAML::Node& in);

  /**
   * @brief Applies the state to a SorghumStemDescriptor.
   * @param target_sorghum_stem_descriptor Target stem descriptor.
   */
  void Apply(SorghumStemDescriptor& target_sorghum_stem_descriptor) const;
};

/**
 * @brief Represents the state of a sorghum leaf.
 */
struct SorghumLeafState {
  bool dead = false;          ///< Indicates if the leaf is dead.
  BezierSpline spline;        ///< Bezier spline defining the leaf shape.
  int index = 0;              ///< Index of the leaf.
  float starting_point = 0;   ///< Starting position along the stem.
  float length = 0.35f;       ///< Length of the leaf.
  float roll_angle = 0;       ///< Roll angle of the leaf.
  float branching_angle = 0;  ///< Branching angle from the stem.

  Plot2D<float> width_along_leaf;                     ///< Width variation along the leaf.
  Plot2D<float> curling_along_leaf;                   ///< Curling pattern along the leaf.
  Plot2D<float> bending_along_leaf;                   ///< Bending pattern along the leaf.
  Plot2D<float> waviness_along_leaf;                  ///< Waviness pattern along the leaf.
  glm::vec2 waviness_period_start = glm::vec2(0.0f);  ///< Start period for waviness.
  float waviness_frequency = 0.0f;                    ///< Frequency of waviness.

  bool saved = false;  ///< Indicates whether the state has been saved.

  /// @brief Default constructor.
  SorghumLeafState();

  /**
   * @brief Copies the shape of another leaf state.
   * @param another The leaf state to copy from.
   */
  void CopyShape(const SorghumLeafState& another);

  /**
   * @brief Serializes the state to a YAML emitter.
   * @param out The YAML emitter to serialize to.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes the state from a YAML node.
   * @param in The YAML node to deserialize from.
   */
  void Deserialize(const YAML::Node& in);

  /**
   * @brief Applies the state to a SorghumLeafDescriptor.
   * @param stem_state The associated stem state.
   * @param target_sorghum_leaf_descriptor Target leaf descriptor.
   */
  void Apply(const SorghumStemState& stem_state, SorghumLeafDescriptor& target_sorghum_leaf_descriptor) const;

  /**
   * @brief Modifies waviness of the leaf.
   * @param factor Modification factor.
   * @param stem_state The associated stem state.
   * @param mesh_generator_settings Mesh settings for procedural generation.
   * @param target_leaf_state Target leaf state.
   */
  void ChangeWaviness(float factor, const SorghumStemState& stem_state,
                      const SorghumMeshGeneratorSettings& mesh_generator_settings,
                      SorghumLeafState& target_leaf_state) const;
};

#pragma endregion

/**
 * @brief Represents the overall state of a sorghum plant.
 */
class SorghumState : public IAsset {
  friend class SorghumGrowthStages;
  friend void SerializeSorghumState(YAML::Emitter& out, const SorghumState& target);
  friend void DeserializeSorghumState(const YAML::Node& in, SorghumState& target);

 public:
  /// @brief Default constructor.
  SorghumState();

  bool saved = false;                    ///< Indicates whether the state has been saved.
  std::string name = "Unnamed";          ///< Name of the state.
  SorghumPanicleState panicle;           ///< Panicle components of the sorghum.
  SorghumStemState stem;                 ///< Stem components of the sorghum.
  std::vector<SorghumLeafState> leaves;  ///< Leaves present in the sorghum.

  /**
   * @brief Applies the state to a SorghumDescriptor.
   * @param target_sorghum_descriptor Target sorghum descriptor.
   */
  void Apply(const std::shared_ptr<SorghumDescriptor>& target_sorghum_descriptor) const;

  /**
   * @brief Creates an entity with the given name.
   * @param name Name of the entity.
   * @return Created entity.
   */
  [[maybe_unused]] Entity CreateEntity(const std::string& name) const;

  /**
   * @brief Generates a thumbnail texture representation.
   * @return Smart pointer to the generated texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture();

  /**
   * @brief Modifies waviness of all leaves in the state.
   * @param factor Modification factor.
   * @param mesh_generator_settings Mesh settings for procedural generation.
   * @param target_sorghum_state Target sorghum state.
   */
  void ChangeWaviness(float factor, const SorghumMeshGeneratorSettings& mesh_generator_settings,
                      SorghumState& target_sorghum_state) const;
};

}  // namespace digital_agriculture_package
