#pragma once
#include "TreeGrowthData.hpp"

using namespace evo_engine;
namespace eco_sys_lab {
struct ShootGrowthController {
  bool m_branchPush = false;
  bool m_useLevelForApicalControl = false;
#pragma region Internode
  int m_baseInternodeCount = 1;
  /**
   * \brief The mean and variance of the angular difference between the growth direction and the direction of the apical
   * bud
   */
  std::function<float(const SkeletonNode<InternodeGrowthData>& internode)> m_baseNodeApicalAngle;

  /**
   * \brief The expected elongation length for an internode for one year.
   */
  float m_internodeGrowthRate;
  /**
   * \brief The mean and variance of the angle between the direction of a lateral bud and its parent shoot.
   */
  std::function<float(const SkeletonNode<InternodeGrowthData>& internode)> m_branchingAngle;
  /**
   * \brief The mean and variance of an angular difference orientation of lateral buds between two internodes
   */
  std::function<float(const SkeletonNode<InternodeGrowthData>& internode)> m_rollAngle;
  /**
   * \brief The mean and variance of the angular difference between the growth direction and the direction of the apical
   * bud
   */
  std::function<float(const SkeletonNode<InternodeGrowthData>& internode)> m_apicalAngle;
  /**
   * \brief The gravitropism.
   */
  std::function<float(const SkeletonNode<InternodeGrowthData>& internode)> m_gravitropism;
  /**
   * \brief The phototropism
   */
  std::function<float(const SkeletonNode<InternodeGrowthData>& internode)> m_phototropism;
  /**
   * \brief The phototropism
   */
  std::function<float(const SkeletonNode<InternodeGrowthData>& internode)> m_horizontalTropism;
  /**
   * \brief The strength of gravity bending.
   */
  std::function<float(const SkeletonNode<InternodeGrowthData>& internode)> m_sagging;

  /**
   * \brief The internode length
   */
  float m_internodeLength;
  /*
   * \brief How the thickness of branch effect the length of the actual node.
   */
  float m_internodeLengthThicknessFactor = 0.0f;
  /**
   * \brief Thickness of end internode
   */
  float m_endNodeThickness;
  /**
   * \brief The thickness accumulation factor
   */
  float m_thicknessAccumulationFactor;
  /**
   * \brief The extra thickness gained from node length.
   */
  float m_thicknessAgeFactor;
  /**
   * \brief The shadow volume factor of the internode.
   */
  float m_internodeShadowFactor = 1.f;
#pragma endregion
#pragma region Bud
  /**
   * \brief The number of lateral buds an internode contains
   */
  std::function<int(const SkeletonNode<InternodeGrowthData>& internode)> m_lateralBudCount;
  /**
   * \brief Extinction rate of apical bud.
   */
  std::function<float(const SkeletonNode<InternodeGrowthData>& internode)> m_m_apicalBudExtinctionRate;

  /**
   * \brief Flushing rate of a bud.
   */
  std::function<float(const SkeletonNode<InternodeGrowthData>& internode)> m_lateralBudFlushingRate;
  /**
   * \brief Apical control base
   */
  float m_apicalControl;
  /**
   * \brief Root distance control base
   */
  float m_rootDistanceControl;
  /**
   * \brief Height control base
   */
  float m_heightControl;

  /**
   * \brief How much inhibitor will an internode generate.
   */
  std::function<float(const SkeletonNode<InternodeGrowthData>& internode)> m_apicalDominance;
  /**
   * \brief How much inhibitor will shrink when going through the branch.
   */
  float m_apicalDominanceLoss;
  std::function<float(const SkeletonNode<InternodeGrowthData>& internode)> m_internodeStrength;
#pragma endregion
#pragma region Pruning
  std::function<float(const SkeletonNode<InternodeGrowthData>& internode)> m_breakingForce;
  /**
   * \brief The The impact of the amount of incoming light on the shedding of end internodes.
   */
  std::function<float(const glm::mat4& globalTransform, ClimateModel& climateModel, const ShootSkeleton& shootSkeleton,
                      const SkeletonNode<InternodeGrowthData>& internode)>
      m_endToRootPruningFactor;
  /**
   * \brief The The impact of the amount of incoming light on the shedding of end internodes.
   */
  std::function<float(const glm::mat4& globalTransform, ClimateModel& climateModel, const ShootSkeleton& shootSkeleton,
                      const SkeletonNode<InternodeGrowthData>& internode)>
      m_rootToEndPruningFactor;
#pragma endregion
#pragma region Leaf

  std::function<float(const SkeletonNode<InternodeGrowthData>& internode)> m_leaf;

  /**
   * \brief The probability of leaf falling after health return to 0.0
   */
  std::function<float(const SkeletonNode<InternodeGrowthData>& internode)> m_leafFallProbability;
#pragma endregion
#pragma region Fruit
  std::function<float(const SkeletonNode<InternodeGrowthData>& internode)> m_fruit;
  /**
   * \brief The probability of fruit falling after health return to 0.0
   */
  std::function<float(const SkeletonNode<InternodeGrowthData>& internode)> m_fruitFallProbability;
#pragma endregion
};
}  // namespace eco_sys_lab