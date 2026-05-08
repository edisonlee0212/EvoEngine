
#pragma once
#include <chrono>

namespace evo_engine {

/**
 * @class Times
 * @brief A class that manages and provides time-related functionality for the engine.
 */
class Times {
  friend class Scene;        ///< Grant access to private members for the `Scene` class.
  friend class Application;  ///< Grant access to private members for the `Application` class.

  std::chrono::time_point<std::chrono::system_clock> start_time_;  ///< The point in time when the engine started.
  std::chrono::time_point<std::chrono::system_clock>
      last_fixed_update_time_;  ///< The point in time of the last fixed update.
  std::chrono::time_point<std::chrono::system_clock> last_update_time_;  ///< The point in time of the last update.
  double delta_time_ = 0.0;                                              ///< Time elapsed between the last two updates.
  double fixed_delta_time_ = 0.0;                                        ///< Time step for fixed updates.
  size_t frames_ = 0;                                                    ///< Number of frames processed.
  size_t steps_ = 0;                                                     ///< Number of fixed update steps processed.
  double time_step_ = 0.016;  ///< Custom time step value defined by the user.

 public:
  /**
   * @brief Displays the inspection interface for the time settings.
   */
  void OnInspect();

  /**
   * @brief Sets the custom time step value.
   * @param value The new time step value.
   */
  void SetTimeStep(double value);

  /**
   * @brief Gets the custom time step value.
   * @return The current time step value.
   */
  [[nodiscard]] double TimeStep() const;

  /**
   * @brief Gets the current time since the engine started.
   * @return The current time in seconds.
   */
  [[nodiscard]] double Now() const;

  /**
   * @brief Gets the time step for fixed updates.
   * @return The fixed delta time value.
   */
  [[nodiscard]] double FixedDeltaTime() const;

  /**
   * @brief Gets the time elapsed between the last two updates.
   * @return The delta time value.
   */
  [[nodiscard]] double DeltaTime() const;

  /**
   * @brief Gets the time of the last update.
   * @return The time of the last update in seconds.
   */
  [[nodiscard]] double LastUpdateTime() const;

  /**
   * @brief Gets the time of the last fixed update.
   * @return The time of the last fixed update in seconds.
   */
  [[nodiscard]] double LastFixedUpdateTime() const;
};

}  // namespace evo_engine
