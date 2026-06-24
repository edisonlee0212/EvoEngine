#pragma once

#include <cstdint>

namespace l_system_package {

class ScotsPine;

struct ScotsPineCalendarSettings {
  int simulation_year = 0;
  float simulation_day_of_year = 0.0f;
  bool calendar_end_enabled = true;
  int calendar_end_year = 2;
  float calendar_end_day_of_year = 0.0f;
};

struct ScotsPineTemporalSample {
  float gdd_per_day = 2.0f;
};

struct ScotsPineCalendarStepResult {
  float descriptor_target_gdd = -1.0f;
  float effective_target_gdd = -1.0f;
  float delta_gdd = 0.0f;
  float target_gdd_before = 0.0f;
  float target_gdd_after = 0.0f;
  std::uint32_t growth_steps = 0;
  bool reached_target = true;
};

struct ScotsPineTemporalFastForwardResult {
  float requested_target_gdd = -1.0f;
  float target_gdd_after = 0.0f;
  int simulation_year_after = 0;
  float simulation_day_of_year_after = 0.0f;
  std::uint32_t calendar_steps = 0;
  std::uint32_t growth_steps = 0;
  bool reached_target = false;
  bool calendar_end_reached = false;
};

float NormalizeScotsPineDayOfYear(float day);
float SampleScotsPineDescriptorTargetGdd(ScotsPine& pine);
ScotsPineTemporalSample SampleScotsPineTemporalParameters(ScotsPine& pine);
void ResetScotsPineTemporalGrowth(ScotsPine& pine);
ScotsPineCalendarStepResult AdvanceScotsPineTemporalGrowth(ScotsPine& pine,
                                                           const ScotsPineCalendarSettings& settings,
                                                           float delta_days,
                                                           float requested_target_gdd = -1.0f);
ScotsPineTemporalFastForwardResult FastForwardScotsPineTemporalGrowth(ScotsPine& pine,
                                                                      ScotsPineCalendarSettings settings,
                                                                      float requested_target_gdd = -1.0f,
                                                                      float step_days = 1.0f,
                                                                      std::uint32_t max_calendar_steps = 200000);
void ResetScotsPineCalendarState(ScotsPine& pine);
ScotsPineCalendarStepResult AdvanceScotsPineCalendarStep(ScotsPine& pine,
                                                         const ScotsPineCalendarSettings& settings,
                                                         float delta_days,
                                                         float requested_target_gdd = -1.0f);

}  // namespace l_system_package
