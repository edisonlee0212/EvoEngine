
#pragma once
#include "IPrivateComponent.hpp"

namespace evo_engine {
void RegisterWayPointsHandlers();

/**
 * @brief Class representing waypoints for an entity.
 *
 * The WayPoints class is used to handle entities' movement along predefined
 * paths with configurable speeds and modes.
 */
class EVOENGINE_API WayPoints : public IPrivateComponent {
 public:
  /**
   * @brief Enum representing modes of waypoint traversal.
   */
  enum class Mode {
    FixedTime,              /**< Move with a fixed time interval between waypoints. */
    FixedVelocity           /**< Move with a fixed velocity between waypoints. */
  } mode = Mode::FixedTime; /**< Specifies the mode of waypoint traversal. */

  /**
   * @brief The speed for movement between waypoints.
   */
  float speed = 1.0f;

  /**
   * @brief The list of entity references that define the waypoints.
   */
  std::vector<EntityRef> entities;

  /**
   * @brief Function called when the component is created.
   */
  void OnCreate() override;

  /**
   * @brief Function called when the component is destroyed.
   */
  void OnDestroy() override;

  /**
   * @brief Function called once per frame to perform updates.
   */
  void Update() override;
};

}  // namespace evo_engine
