#pragma once
#include "TreeGrowthData.hpp"

using namespace evo_engine;
namespace eco_sys_lab_plugin {
struct ShootGrowthController {
  bool branch_push = false;
  bool use_level_for_apical_control = false;
#pragma region Internode
  int base_internode_count = 1;
  /**
   * \brief The mean and variance of the angular difference between the growth direction and the direction of the apical
   * bud
   */
  std::function<float(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)>
      base_node_apical_angle;

  /**
   * \brief The expected elongation length for an internode for one year.
   */
  float internode_growth_rate;
  /**
   * \brief The mean and variance of the angle between the direction of a lateral bud and its parent shoot.
   */
  std::function<float(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)> branching_angle;
  /**
   * \brief The mean and variance of an angular difference orientation of lateral buds between two internodes
   */
  std::function<float(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)> roll_angle;
  /**
   * \brief The mean and variance of the angular difference between the growth direction and the direction of the apical
   * bud
   */
  std::function<float(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)> apical_angle;
  /**
   * \brief The gravitropism.
   */
  std::function<float(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)> gravitropism;
  /**
   * \brief The phototropism
   */
  std::function<float(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)> phototropism;
  /**
   * \brief The phototropism
   */
  std::function<float(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)>
      horizontal_tropism;
  /**
   * \brief The strength of gravity bending.
   */
  std::function<float(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)> sagging;

  /**
   * \brief The internode length
   */
  float internode_length;
  /*
   * \brief How the thickness of branch effect the length of the actual node.
   */
  float internode_length_thickness_factor = 0.0f;
  /**
   * \brief Thickness of end internode
   */
  float end_node_thickness;
  /**
   * \brief The thickness accumulation factor
   */
  float thickness_accumulation_factor;
  /**
   * \brief The extra thickness gained from node length.
   */
  float thickness_age_factor;
  /**
   * \brief The shadow volume factor of the internode.
   */
  float internode_shadow_factor = 1.f;
#pragma endregion
#pragma region Bud
  /**
   * \brief The number of lateral buds an internode contains
   */
  std::function<int(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)> lateral_bud_count;
  /**
   * \brief Extinction rate of apical bud.
   */
  std::function<float(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)>
      apical_bud_extinction_rate;

  /**
   * \brief Flushing rate of a bud.
   */
  std::function<float(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)>
      lateral_bud_flushing_rate;
  /**
   * \brief Apical control base
   */
  float apical_control;
  /**
   * \brief Root distance control base
   */
  float root_distance_control;
  /**
   * \brief Height control base
   */
  float height_control;

  /**
   * \brief How much inhibitor will an internode generate.
   */
  std::function<float(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)>
      apical_dominance;
  /**
   * \brief How much inhibitor will shrink when going through the branch.
   */
  float apical_dominance_loss;
  std::function<float(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)>
      internode_strength;
#pragma endregion
#pragma region Pruning
  std::function<float(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)> breaking_force;
  /**
   * \brief The The impact of the amount of incoming light on the shedding of end internodes.
   */
  std::function<float(std::mt19937& random_engine, const glm::mat4& global_transform, ClimateModel& climate_model,
                      const ShootSkeleton& shoot_skeleton, const SkeletonNode<InternodeGrowthData>& internode)>
      end_to_root_pruning_factor;
  /**
   * \brief The The impact of the amount of incoming light on the shedding of end internodes.
   */
  std::function<float(std::mt19937& random_engine, const glm::mat4& global_transform, ClimateModel& climate_model,
                      const ShootSkeleton& shoot_skeleton, const SkeletonNode<InternodeGrowthData>& internode)>
      root_to_end_pruning_factor;
#pragma endregion
#pragma region Leaf

  std::function<float(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)> leaf;

  /**
   * \brief The probability of leaf falling after health return to 0.0
   */
  std::function<float(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)>
      leaf_fall_probability;
#pragma endregion
#pragma region Fruit
  std::function<float(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)> fruit;
  /**
   * \brief The probability of fruit falling after health return to 0.0
   */
  std::function<float(std::mt19937& random_engine, const SkeletonNode<InternodeGrowthData>& internode)>
      fruit_fall_probability;
#pragma endregion
};
}  // namespace eco_sys_lab_plugin