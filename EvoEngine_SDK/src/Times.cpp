#include "Times.hpp"
using namespace evo_engine;

double Times::time_step_ = 0.016;
double Times::delta_time_ = 0;
double Times::fixed_delta_time_ = 0;
size_t Times::frames_ = 0;
size_t Times::steps_ = 0;
std::chrono::time_point<std::chrono::system_clock> Times::start_time_ = {};
std::chrono::time_point<std::chrono::system_clock> Times::last_fixed_update_time_ = {};
std::chrono::time_point<std::chrono::system_clock> Times::last_update_time_ = {};

void Times::OnInspect() {
  if (ImGui::CollapsingHeader("Times Settings")) {
    float time_step = time_step_;
    if (ImGui::DragFloat("Times step", &time_step, 0.001f, 0.001f, 1.0f)) {
      time_step_ = time_step;
    }
  }
}

double Times::TimeStep() {
  return time_step_;
}
void Times::SetTimeStep(const double value) {
  time_step_ = value;
}
double Times::FixedDeltaTime() {
  return fixed_delta_time_;
}

double Times::DeltaTime() {
  return delta_time_;
}

double Times::Now() {
  const auto now = std::chrono::system_clock::now();
  const std::chrono::duration<double> duration = now - start_time_;
  return duration.count();
}

double Times::LastUpdateTime() {
  const std::chrono::duration<double> duration = last_update_time_ - start_time_;
  return duration.count();
}

double Times::LastFixedUpdateTime() {
  const std::chrono::duration<double> duration = last_fixed_update_time_ - start_time_;
  return duration.count();
}