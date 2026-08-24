#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

namespace evo_engine {

struct ProfilerScopeEvent {
  uint64_t frame_index = 0;
  uint64_t thread_id = 0;
  std::string thread_name;
  std::string name;
  std::string category;
  uint32_t depth = 0;
  double start_ms = 0.0;
  double duration_ms = 0.0;
};

struct ProfilerCounter {
  std::string name;
  std::string category;
  std::string unit;
  double value = 0.0;
};

struct ProfilerFrameSnapshot {
  uint64_t frame_index = 0;
  uint64_t application_frame_index = 0;
  double duration_ms = 0.0;
  std::vector<ProfilerScopeEvent> events;
  std::vector<ProfilerCounter> counters;
  uint64_t capture_session_index = 0;
  uint32_t dropped_event_count = 0;
};

struct ProfilerHierarchyNode {
  std::string name;
  std::string category;
  uint32_t count = 0;
  double inclusive_ms = 0.0;
  double self_ms = 0.0;
  double max_ms = 0.0;
  std::vector<ProfilerHierarchyNode> children;
};

struct ProfilerAggregateTotal {
  std::string name;
  std::string category;
  uint32_t count = 0;
  double total_ms = 0.0;
  double average_ms = 0.0;
  double max_ms = 0.0;
};

struct ProfilerThreadLane {
  uint64_t thread_id = 0;
  std::string thread_name;
  double total_ms = 0.0;
  std::vector<ProfilerScopeEvent> events;
  std::vector<ProfilerHierarchyNode> hierarchy;
};

struct ProfilerFrameStats {
  uint64_t frame_index = 0;
  uint64_t application_frame_index = 0;
  uint64_t capture_session_index = 0;
  double duration_ms = 0.0;
  uint32_t event_count = 0;
  uint32_t dropped_event_count = 0;
  double total_event_ms = 0.0;
  double max_event_ms = 0.0;
  std::vector<ProfilerThreadLane> thread_lanes;
  std::vector<ProfilerAggregateTotal> category_totals;
  std::vector<ProfilerAggregateTotal> named_event_totals;
  std::vector<ProfilerCounter> counters;
};

struct ProfilerDurationSummary {
  size_t frame_count = 0;
  size_t observed_frame_count = 0;
  double selected_ms = 0.0;
  double average_ms = 0.0;
  double median_ms = 0.0;
  double p95_ms = 0.0;
  double maximum_ms = 0.0;
};

struct ProfilerHistoryNode {
  std::string name;
  std::string category;
  ProfilerDurationSummary inclusive;
  ProfilerDurationSummary self;
  std::vector<ProfilerHistoryNode> children;
};

struct ProfilerThreadHistory {
  uint64_t thread_id = 0;
  std::string thread_name;
  ProfilerDurationSummary root_work;
  std::vector<ProfilerHistoryNode> hierarchy;
};

struct ProfilerCounterHistory {
  std::string name;
  std::string category;
  std::string unit;
  size_t frame_count = 0;
  size_t observed_frame_count = 0;
  double selected = 0.0;
  double average = 0.0;
  double median = 0.0;
  double p95 = 0.0;
  double maximum = 0.0;
};

struct ProfilerHistoryStats {
  size_t frame_count = 0;
  ProfilerDurationSummary frame_duration;
  ProfilerDurationSummary main_thread_wall;
  ProfilerDurationSummary worker_cpu_work;
  std::vector<ProfilerThreadHistory> threads;
  std::vector<ProfilerCounterHistory> counters;
};

struct ProfilerScopeToken {
  bool active = false;
  uint64_t capture_session_index = 0;
};

[[nodiscard]] ProfilerFrameStats BuildProfilerFrameStats(const ProfilerFrameSnapshot& snapshot);
[[nodiscard]] std::vector<ProfilerFrameStats> BuildProfilerFrameStatsHistory(
    const std::vector<ProfilerFrameSnapshot>& snapshots);
[[nodiscard]] ProfilerHistoryStats BuildProfilerHistoryStats(
    const std::vector<ProfilerFrameStats>& frames, size_t selected_frame_index = (std::numeric_limits<size_t>::max)());
[[nodiscard]] bool ExportProfilerChromeTrace(const std::filesystem::path& path,
                                             const std::vector<ProfilerFrameSnapshot>& snapshots,
                                             std::string* error = nullptr);

class Profiler final {
 public:
  static Profiler& GetInstance();

  void SetEnabled(bool enabled);
  [[nodiscard]] bool IsEnabled() const;
  void Reset();
  void ClearFrameHistory();
  void SetMaxFrameHistory(size_t max_frame_history);

  void RegisterThread(const std::string& name);
  uint64_t BeginFrame(uint64_t application_frame_index = 0);
  void EndFrame();
  [[nodiscard]] ProfilerScopeToken BeginScope(const std::string& name, const std::string& category = "CPU");
  void EndScope(ProfilerScopeToken& token);
  void RecordCounter(const std::string& name, double value, const std::string& category = "Frame",
                     const std::string& unit = {});

  [[nodiscard]] ProfilerFrameSnapshot GetLatestFrameSnapshot() const;
  [[nodiscard]] std::vector<ProfilerFrameSnapshot> GetFrameHistorySnapshot() const;
  [[nodiscard]] ProfilerFrameStats GetLatestFrameStatsSnapshot() const;
  [[nodiscard]] std::vector<ProfilerFrameStats> GetFrameStatsHistorySnapshot() const;
  [[nodiscard]] bool ExportChromeTrace(const std::filesystem::path& path, std::string* error = nullptr) const;
};

class ProfilerScope final {
  ProfilerScopeToken token_;

 public:
  explicit ProfilerScope(const std::string& name, const std::string& category = "CPU");
  ~ProfilerScope();
  ProfilerScope(const ProfilerScope&) = delete;
  ProfilerScope& operator=(const ProfilerScope&) = delete;
};

class ProfilerFrameScope final {
 public:
  explicit ProfilerFrameScope(uint64_t application_frame_index = 0);
  ~ProfilerFrameScope();
  ProfilerFrameScope(const ProfilerFrameScope&) = delete;
  ProfilerFrameScope& operator=(const ProfilerFrameScope&) = delete;
};

}  // namespace evo_engine
