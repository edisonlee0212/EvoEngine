#pragma once
#include "Noises.hpp"
#include "ProceduralNoise.hpp"
#include "TreeModel.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
class ShootDescriptor : public IAsset {
 public:
  /**
   * \brief The expected height gain for the tree for one year (max root distance).
   */
  float growth_rate = 0.25f;
  float straight_trunk = 0.0f;

#pragma region Internode
  int base_internode_count = 1;
  glm::vec2 base_node_apical_angle_mean_variance = glm::vec2(0.0f);

  /**
   * \brief The mean and variance of the angle between the direction of a lateral bud and its parent shoot.
   */
  glm::vec2 branching_angle_mean_variance = glm::vec2(45, 2);
  /**
   * \brief The mean and variance of an angular difference orientation of lateral buds between two internodes
   */
  glm::vec2 roll_angle_mean_variance = glm::vec2(30, 2);
  /**
   * \brief The procedural noise of an angular difference orientation of lateral buds between two internodes
   */
  AssetRef roll_angle{};
  Noise2D roll_angle_noise_2d{};
  /**
   * \brief The mean and variance of an angular difference orientation of lateral buds between two internodes
   */
  glm::vec2 apical_angle_mean_variance = glm::vec2(0, 3);
  /**
   * \brief The procedural noise of an angular difference orientation of lateral buds between two internodes
   */
  AssetRef apical_angle{};
  Noise2D apical_angle_noise_2d{};
  /**
   * \brief The gravitropism.
   */
  float gravitropism = 0.0;
  /**
   * \brief The phototropism
   */
  float phototropism = 0.045f;
  /**
   * \brief The horizontal tropism
   */
  float horizontal_tropism = 0.0f;

  float gravity_bending_strength = 0.f;
  float gravity_bending_thickness_factor = 1.f;
  float gravity_bending_max = 1.f;

  /**
   * \brief The internode length
   */
  float internode_length = 0.03f;
  /*
   * \brief How the thickness of branch effect the length of the actual node.
   */
  float internode_length_thickness_factor = 0.15f;
  /**
   * \brief Thickness of end internode
   */
  float end_node_thickness = 0.004f;
  /**
   * \brief The thickness accumulation factor
   */
  float thickness_accumulation_factor = 0.45f;
  /**
   * \brief The extra thickness gained from node length.
   */
  float thickness_age_factor = 0.0f;
  /**
   * \brief The shadow volume factor of the internode.
   */
  float internode_shadow_factor = 0.03f;

#pragma endregion
#pragma region Bud fate
  /**
   * \brief The number of lateral buds an internode contains
   */
  int lateral_bud_count = 1;
  int max_order = -1;
  /**
   * \brief The probability of death of apical bud each year.
   */
  float apical_bud_extinction_rate = 0.0f;
  /**
   * \brief The probability of death of lateral bud each year.
   */
  float lateral_bud_flushing_rate = 0.5f;
  /**
   * \brief Apical control base
   */
  float apical_control = 1.25f;
  /**
   * \brief Apical control base
   */
  float root_distance_control = 0.f;
  /**
   * \brief Apical control base
   */
  float height_control = 0.f;

  /**
   * \brief How much inhibitor will an internode generate.
   */
  float apical_dominance = 0.25f;
  /**
   * \brief How much inhibitor will shrink when going through the branch.
   */
  float apical_dominance_loss = 0.08f;

#pragma endregion
#pragma region Pruning
  bool trunk_protection = false;

  int max_flow_length = 0;

  /**
   * \brief The pruning factor for branch because of absence of light
   */
  float light_pruning_factor = 0.0f;

  float branch_strength = 1.f;
  float branch_strength_thickness_factor = 3.f;
  float branch_strength_lighting_threshold = 0.f;
  float branch_strength_lighting_loss = 0.f;
  float branch_breaking_multiplier = 1.f;
  float branch_breaking_factor = 1.f;
#pragma endregion

  AssetRef bark_material;
#pragma region Leaf
  float leaf_flushing_lighting_requirement = 0.1f;
  float leaf_fall_probability;
  float leaf_distance_to_branch_end_limit;
#pragma endregion
#pragma region Fruit
  float fruit_flushing_lighting_requirement = 0.1f;
  float fruit_fall_probability;
  float fruit_distance_to_branch_end_limit;
#pragma endregion
  void PrepareController(ShootGrowthController& shootGrowthController) const;

  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};

}  // namespace eco_sys_lab