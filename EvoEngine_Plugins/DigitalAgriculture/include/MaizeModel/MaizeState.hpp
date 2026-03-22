#pragma once
#include "Curve.hpp"
#include "Plot2D.hpp"
#include "MaizeDescriptor.hpp"

namespace digital_agriculture_plugin {
using namespace evo_engine;
#pragma region States

/**
 * @brief Represents the state modes available.
 */
enum class MaizeStateMode { Default, CubicBezier };

/**
 * @brief Represents the state of a maize panicle (tassel).
 */
struct MaizePanicleState {
  glm::vec3 panicle_size = glm::vec3(0, 0, 0);
  int seed_amount = 0;
  float seed_radius = 0.002f;

  bool saved = false;

  MaizePanicleState();

  bool OnInspectImpl();

  void Serialize(YAML::Emitter& out) const;

  void Deserialize(const YAML::Node& in);

  void Apply(MaizePanicleDescriptor& target_maize_panicle_descriptor) const;
};

/**
 * @brief Represents the state of a maize stem.
 */
struct MaizeStemState {
  BezierSpline spline;
  glm::vec3 direction = {0, 1, 0};
  Plot2D<float> width_along_stem;
  float length = 0;

  bool saved = false;

  MaizeStemState();

  [[nodiscard]] glm::vec3 GetPoint(float point) const;

  void Serialize(YAML::Emitter& out) const;

  void Deserialize(const YAML::Node& in);

  bool OnInspectImpl(int mode);

  void Apply(MaizeStemDescriptor& target_maize_stem_descriptor) const;
};

/**
 * @brief Represents the state of a maize leaf.
 */
struct MaizeLeafState {
  bool dead = false;
  BezierSpline spline;
  int index = 0;
  float starting_point = 0;
  float length = 0.35f;
  float roll_angle = 0;
  float branching_angle = 0;

  Plot2D<float> width_along_leaf;
  Plot2D<float> curling_along_leaf;
  Plot2D<float> bending_along_leaf;
  Plot2D<float> waviness_along_leaf;
  glm::vec2 waviness_period_start = glm::vec2(0.0f);
  float waviness_frequency = 0.0f;
  float current_growth = 1.0f;

  bool saved = false;

  MaizeLeafState();

  void CopyShape(const MaizeLeafState& another);

  void Serialize(YAML::Emitter& out) const;

  void Deserialize(const YAML::Node& in);

  bool OnInspectImpl(int mode);

  void Apply(const MaizeStemState& stem_state, MaizeLeafDescriptor& target_maize_leaf_descriptor) const;

  void ChangeWaviness(float factor, const MaizeStemState& stem_state,
                      const MaizeMeshGeneratorSettings& mesh_generator_settings,
                      MaizeLeafState& target_leaf_state) const;
};

#pragma endregion

/**
 * @brief Represents the overall state of a maize plant.
 */
class MaizeState : public IAsset {
 public:
  MaizeState();

  bool saved = false;
  std::string name = "Unnamed";
  MaizePanicleState panicle;
  MaizeStemState stem;
  std::vector<MaizeLeafState> leaves;

  bool OnInspectImpl(int mode);

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  void Apply(const std::shared_ptr<MaizeDescriptor>& target_maize_descriptor) const;

  void Serialize(YAML::Emitter& out) const override;

  void Deserialize(const YAML::Node& in) override;

  [[maybe_unused]] Entity CreateEntity(const std::string& name) const;

  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;

  void ChangeWaviness(float factor, const MaizeMeshGeneratorSettings& mesh_generator_settings,
                      MaizeState& target_maize_state) const;
};

}  // namespace digital_agriculture_plugin
