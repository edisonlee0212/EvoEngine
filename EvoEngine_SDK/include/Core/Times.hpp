#pragma once
namespace evo_engine {
class Times {
  friend class Scene;
  friend class Application;
  static std::chrono::time_point<std::chrono::system_clock> start_time_;
  static std::chrono::time_point<std::chrono::system_clock> last_fixed_update_time_;
  static std::chrono::time_point<std::chrono::system_clock> last_update_time_;
  static double delta_time_;
  static double fixed_delta_time_;
  static size_t frames_;
  static size_t steps_;
  static double time_step_;

 public:
  static void OnInspect();
  static void SetTimeStep(double value);
  [[nodiscard]] static double TimeStep();
  [[nodiscard]] static double Now();
  [[nodiscard]] static double FixedDeltaTime();
  [[nodiscard]] static double DeltaTime();
  [[nodiscard]] static double LastUpdateTime();
  [[nodiscard]] static double LastFixedUpdateTime();
};
}  // namespace evo_engine