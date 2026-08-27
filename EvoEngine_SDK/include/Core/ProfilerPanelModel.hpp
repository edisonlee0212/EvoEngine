#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace evo_engine::profiler_panel_detail {

struct TimingInterval {
  double start_ms = 0.0;
  double duration_ms = 0.0;
};

struct FrameOverviewSample {
  double cpu_active_ms = 0.0;
  double synchronization_ms = 0.0;
  double cpu_wall_ms = 0.0;
  double gpu_ms = 0.0;
  double total_ms = 0.0;
  bool gpu_available = false;
};

enum class CpuExecutorGroup : uint8_t { MainThread, Worker, AssetIo, GpuSubmission, Render, Background, Other };

[[nodiscard]] EVOENGINE_API double IntervalUnionMilliseconds(std::vector<TimingInterval> intervals);
[[nodiscard]] EVOENGINE_API FrameOverviewSample BuildFrameOverviewSample(double cpu_wall_ms, double synchronization_ms,
                                                                         std::optional<double> gpu_ms);
[[nodiscard]] EVOENGINE_API CpuExecutorGroup ClassifyCpuExecutor(const std::string& thread_name);
[[nodiscard]] EVOENGINE_API const char* CpuExecutorGroupName(CpuExecutorGroup group);
[[nodiscard]] EVOENGINE_API std::string StableHierarchyKey(CpuExecutorGroup group, const std::string& parent_path,
                                                           const std::string& category, const std::string& name);

template <typename Entry, typename Key, typename KeySelector>
Entry& AppendFirstSeen(std::vector<Entry>& entries, const Key& key, KeySelector&& key_selector, Entry entry) {
  const auto search = std::find_if(entries.begin(), entries.end(), [&](const auto& existing) {
    return key_selector(existing) == key;
  });
  if (search != entries.end())
    return *search;
  entries.emplace_back(std::move(entry));
  return entries.back();
}

struct RollingSummary {
  double average = 0.0;
  double maximum = 0.0;
  double p95 = 0.0;
  size_t observed_count = 0;
};

inline RollingSummary SummarizeWithMissingZeros(std::vector<double> observed_values, const size_t frame_count) {
  RollingSummary result;
  result.observed_count = observed_values.size();
  if (frame_count == 0)
    return result;
  for (const double value : observed_values) {
    result.average += value;
    result.maximum = std::max(result.maximum, value);
  }
  result.average /= static_cast<double>(frame_count);
  observed_values.resize(frame_count, 0.0);
  std::sort(observed_values.begin(), observed_values.end());
  result.p95 = observed_values[static_cast<size_t>(std::ceil(static_cast<double>(frame_count) * 0.95)) - 1];
  return result;
}

}  // namespace evo_engine::profiler_panel_detail
