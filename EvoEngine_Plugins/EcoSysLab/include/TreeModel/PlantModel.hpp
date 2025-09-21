#pragma once
#include "ClimateModel.hpp"
#include "Octree.hpp"
#include "TreeControllers.hpp"
#include "TreeGrowthSettings.hpp"
using namespace evo_engine;
namespace eco_sys_lab_plugin {
/**
 * @brief Represents the procedural structure and behavior of a tree model.
 */
class PlantModel {
 protected:
  bool initialized_ = false;         ///< Tracks whether the model has been initialized.
  int age_in_year_ = 0;              ///< Integer representation of the tree's age in years.
  float current_delta_time_ = 1.0f;  ///< Time step used for growth calculations.
  std::mt19937 random_engine_;       ///< Random number generator engine.
  int iteration_ = 0;                ///< The current growth iteration of the tree.
  friend class Tree;

 public:
  int history_limit = -1;  ///< The limit for stored history states.
  int seed = 0;            ///< The seed value for random number generation.
                           /**
                            * @brief Applies tropism effects to a directional vector.
                            * @param target_dir The target direction vector.
                            * @param tropism The strength of the tropism effect.
                            * @param front The front-facing direction to be adjusted.
                            * @param up The up direction to be maintained.
                            */
  static void ApplyTropism(const glm::vec3& target_dir, float tropism, glm::vec3& front, glm::vec3& up);

  /**
   * @brief Applies tropism effects to a rotation quaternion.
   * @param target_dir The target direction vector.
   * @param tropism The strength of the tropism effect.
   * @param rotation The quaternion rotation to be modified.
   */
  static void ApplyTropism(const glm::vec3& target_dir, float tropism, glm::quat& rotation);
};
}  // namespace eco_sys_lab_plugin