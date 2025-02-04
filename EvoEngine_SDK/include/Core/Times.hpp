
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

  static std::chrono::time_point<std::chrono::system_clock>
      start_time_;  ///< The point in time when the engine started.
  static std::chrono::time_point<std::chrono::system_clock>
      last_fixed_update_time_;  ///< The point in time of the last fixed update.
  static std::chrono::time_point<std::chrono::system_clock>
      last_update_time_;            ///< The point in time of the last update.
  static double delta_time_;        ///< Time elapsed between the last two updates.
  static double fixed_delta_time_;  ///< Time step for fixed updates.
  static size_t frames_;            ///< Number of frames processed.
  static size_t steps_;             ///< Number of fixed update steps processed.
  static double time_step_;         ///< Custom time step value defined by the user.

 public:
  /**
   * @brief Displays the inspection interface for the time settings.
   */
  static void OnInspect();

  /**
   * @brief Sets the custom time step value.
   * @param value The new time step value.
   */
  static void SetTimeStep(double value);

  /**
   * @brief Gets the custom time step value.
   * @return The current time step value.
   */
  [[nodiscard]] static double TimeStep();

  /**
   * @brief Gets the current time since the engine started.
   * @return The current time in seconds.
   */
  [[nodiscard]] static double Now();

  /**
   * @brief Gets the time step for fixed updates.
   * @return The fixed delta time value.
   */
  [[nodiscard]] static double FixedDeltaTime();

  /**
   * @brief Gets the time elapsed between the last two updates.
   * @return The delta time value.
   */
  [[nodiscard]] static double DeltaTime();

  /**
   * @brief Gets the time of the last update.
   * @return The time of the last update in seconds.
   */
  [[nodiscard]] static double LastUpdateTime();

  /**
   * @brief Gets the time of the last fixed update.
   * @return The time of the last fixed update in seconds.
   */
  [[nodiscard]] static double LastFixedUpdateTime();
};

}  // namespace evo_engine
