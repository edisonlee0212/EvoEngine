#include "ScotsPineTemporalGrowth.hpp"

#include "LSystemRuleHelpers.hpp"
#include "ScotsPine.hpp"
#include "ScotsPineDescriptor.hpp"

#include <algorithm>
#include <cmath>
#include <random>

using namespace l_system_package;
using namespace evo_engine;

float l_system_package::NormalizeScotsPineDayOfYear(float day) {
  if (!std::isfinite(day)) {
    return 0.0f;
  }
  day = std::fmod(day, 365.0f);
  if (day < 0.0f) {
    day += 365.0f;
  }
  return day;
}

namespace {

double CalendarAbsoluteDay(const int year, const float day_of_year) {
  return static_cast<double>(std::max(0, year)) * 365.0 + static_cast<double>(NormalizeScotsPineDayOfYear(day_of_year));
}

void SetCalendarFromAbsoluteDay(ScotsPineCalendarSettings& settings, const double absolute_day) {
  const double safe_day = std::max(0.0, absolute_day);
  settings.simulation_year = static_cast<int>(std::floor(safe_day / 365.0));
  settings.simulation_day_of_year = static_cast<float>(safe_day - static_cast<double>(settings.simulation_year) * 365.0);
  settings.simulation_day_of_year = NormalizeScotsPineDayOfYear(settings.simulation_day_of_year);
}

float AdvanceCalendarSettings(ScotsPineCalendarSettings& settings, const float requested_delta_days,
                              bool& calendar_end_reached) {
  calendar_end_reached = false;
  const float safe_delta_days = std::isfinite(requested_delta_days) ? std::max(0.0f, requested_delta_days) : 0.0f;
  const double current = CalendarAbsoluteDay(settings.simulation_year, settings.simulation_day_of_year);
  double target = current + static_cast<double>(safe_delta_days);
  const double end = CalendarAbsoluteDay(settings.calendar_end_year, settings.calendar_end_day_of_year);
  if (current >= end) {
    target = end;
    calendar_end_reached = true;
  } else if (target >= end) {
    target = end;
    calendar_end_reached = true;
  }
  SetCalendarFromAbsoluteDay(settings, target);
  return static_cast<float>(std::max(0.0, target - current));
}

}  // namespace

float l_system_package::SampleScotsPineDescriptorTargetGdd(ScotsPine& pine) {
  auto descriptor = pine.descriptor_ref.Get<ScotsPineDescriptor>();
  if (!descriptor) {
    return -1.0f;
  }

  std::mt19937 rng(pine.seed);
  return std::max(0.0f, SampleDistribution(descriptor->target_gdd, rng));
}

ScotsPineTemporalSample l_system_package::SampleScotsPineTemporalParameters(ScotsPine& pine) {
  ScotsPineTemporalSample sample;
  auto descriptor = pine.descriptor_ref.Get<ScotsPineDescriptor>();
  if (!descriptor) {
    return sample;
  }

  std::mt19937 rng(static_cast<std::uint32_t>(pine.seed) ^ 0x5f3759dfu);
  sample.gdd_per_day = std::max(0.0f, SampleDistribution(descriptor->gdd_per_day, rng));
  return sample;
}

void l_system_package::ResetScotsPineCalendarState(ScotsPine& pine) {
  ResetScotsPineTemporalGrowth(pine);
}

void l_system_package::ResetScotsPineTemporalGrowth(ScotsPine& pine) {
  pine.target_gdd = 0.0f;
  pine.growth_model.Reset();
  pine.ClearGeometryEntities();
}

ScotsPineCalendarStepResult l_system_package::AdvanceScotsPineCalendarStep(
    ScotsPine& pine, const ScotsPineCalendarSettings& settings, const float delta_days,
    const float requested_target_gdd) {
  return AdvanceScotsPineTemporalGrowth(pine, settings, delta_days, requested_target_gdd);
}

ScotsPineCalendarStepResult l_system_package::AdvanceScotsPineTemporalGrowth(
    ScotsPine& pine, const ScotsPineCalendarSettings& settings, const float delta_days,
    const float requested_target_gdd) {
  ScotsPineCalendarStepResult result;
  (void)settings;
  result.target_gdd_before = pine.target_gdd;

  const float safe_delta_days = std::isfinite(delta_days) ? std::max(0.0f, delta_days) : 0.0f;
  const auto pine_temporal = SampleScotsPineTemporalParameters(pine);
  result.delta_gdd = std::max(0.0f, pine_temporal.gdd_per_day) * safe_delta_days;
  constexpr float pine_season_length_years = 1.0f;

  pine.SetSeasonalChronologicalMode(true);

  const float delta_years = safe_delta_days / 365.0f;
  result.descriptor_target_gdd = SampleScotsPineDescriptorTargetGdd(pine);
  result.effective_target_gdd =
      requested_target_gdd >= 0.0f ? std::max(0.0f, requested_target_gdd) : result.descriptor_target_gdd;

  if (delta_years > 0.0f && pine.EnsureGrowthModelInitializedForGrowth()) {
    pine.growth_model.graph.data.clock.SyncSeasonalState(true, true, pine_season_length_years);
  }

  const float effective_gdd_per_day = std::max(0.0f, pine_temporal.gdd_per_day);
  const float target_room_gdd = result.effective_target_gdd >= 0.0f
                                    ? std::max(0.0f, result.effective_target_gdd - pine.target_gdd)
                                    : result.delta_gdd;
  float remaining_gdd = std::min(std::max(0.0f, result.delta_gdd), target_room_gdd);
  const float growth_step_gdd =
      pine.growth_model.IsInitialized() ? std::max(1.0e-4f, pine.growth_model.gdd_per_growth_step) : 1.0f;
  while (remaining_gdd > 1.0e-5f && effective_gdd_per_day > 1.0e-6f) {
    const float sub_delta_gdd = std::min(remaining_gdd, growth_step_gdd);
    const float sub_delta_years = (sub_delta_gdd / effective_gdd_per_day) / 365.0f;
    if (sub_delta_years > 0.0f && pine.growth_model.IsInitialized()) {
      pine.growth_model.AdvanceChronologicalYears(sub_delta_years);
      pine.growth_model.graph.data.clock.AdvanceYears(sub_delta_years);
      pine.growth_model.graph.data.clock.SyncSeasonalState(true, true, pine_season_length_years);
    }

    pine.target_gdd = std::max(0.0f, pine.target_gdd + sub_delta_gdd);
    if (result.effective_target_gdd >= 0.0f) {
      pine.target_gdd = std::min(pine.target_gdd, result.effective_target_gdd);
    }
    pine.GrowToTargetGDD(false, false);
    result.growth_steps += pine.growth_model.last_growth_steps;
    remaining_gdd -= sub_delta_gdd;
  }

  if (result.growth_steps > 0) {
    pine.RebuildGeometry();
  } else if (delta_years > 0.0f && pine.growth_model.IsInitialized()) {
    pine.RebuildGeometry();
  }
  result.target_gdd_after = pine.target_gdd;
  result.reached_target = result.effective_target_gdd < 0.0f || pine.target_gdd + 1.0e-4f >= result.effective_target_gdd;
  return result;
}

ScotsPineTemporalFastForwardResult l_system_package::FastForwardScotsPineTemporalGrowth(
    ScotsPine& pine, ScotsPineCalendarSettings settings, const float requested_target_gdd, const float step_days,
    const std::uint32_t max_calendar_steps) {
  ScotsPineTemporalFastForwardResult result;
  const float sampled_target_gdd = requested_target_gdd >= 0.0f ? std::max(0.0f, requested_target_gdd)
                                                                 : SampleScotsPineDescriptorTargetGdd(pine);
  result.requested_target_gdd = sampled_target_gdd;
  if (sampled_target_gdd < 0.0f) {
    result.target_gdd_after = pine.target_gdd;
    result.simulation_year_after = settings.simulation_year;
    result.simulation_day_of_year_after = settings.simulation_day_of_year;
    return result;
  }

  const float safe_step_days = std::max(1.0e-4f, std::isfinite(step_days) ? step_days : 1.0f);
  const std::uint32_t safe_max_steps = std::max<std::uint32_t>(1, max_calendar_steps);
  for (; result.calendar_steps < safe_max_steps; ++result.calendar_steps) {
    bool calendar_end_reached = false;
    const float actual_delta_days = AdvanceCalendarSettings(settings, safe_step_days, calendar_end_reached);
    result.calendar_end_reached = calendar_end_reached;
    if (actual_delta_days <= 0.0f && calendar_end_reached) {
      break;
    }

    const auto step_result = AdvanceScotsPineTemporalGrowth(pine, settings, actual_delta_days, sampled_target_gdd);
    result.growth_steps += step_result.growth_steps;
    result.target_gdd_after = step_result.target_gdd_after;
    if (step_result.reached_target) {
      result.reached_target = true;
      ++result.calendar_steps;
      break;
    }
    if (calendar_end_reached) {
      break;
    }
  }

  result.target_gdd_after = pine.target_gdd;
  result.simulation_year_after = settings.simulation_year;
  result.simulation_day_of_year_after = settings.simulation_day_of_year;
  result.reached_target = result.reached_target || pine.target_gdd + 1.0e-4f >= sampled_target_gdd;
  return result;
}
