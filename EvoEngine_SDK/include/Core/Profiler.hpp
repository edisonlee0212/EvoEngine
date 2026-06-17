#pragma once

#include <cstdint>
#include <filesystem>
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

struct ProfilerFrameSnapshot {
  uint64_t frame_index = 0;
  double duration_ms = 0.0;
  std::vector<ProfilerScopeEvent> events;
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
};

struct ProfilerFrameStats {
  uint64_t frame_index = 0;
  double duration_ms = 0.0;
  uint32_t event_count = 0;
  double total_event_ms = 0.0;
  double max_event_ms = 0.0;
  std::vector<ProfilerThreadLane> thread_lanes;
  std::vector<ProfilerAggregateTotal> category_totals;
  std::vector<ProfilerAggregateTotal> named_event_totals;
};

struct ProfilerScopeToken {
  bool active = false;
};

[[nodiscard]] ProfilerFrameStats BuildProfilerFrameStats(const ProfilerFrameSnapshot& snapshot);
[[nodiscard]] std::vector<ProfilerFrameStats> BuildProfilerFrameStatsHistory(
    const std::vector<ProfilerFrameSnapshot>& snapshots);
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
  uint64_t BeginFrame();
  void EndFrame();
  [[nodiscard]] ProfilerScopeToken BeginScope(const std::string& name, const std::string& category = "CPU");
  void EndScope(ProfilerScopeToken& token);

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
  ProfilerFrameScope();
  ~ProfilerFrameScope();
  ProfilerFrameScope(const ProfilerFrameScope&) = delete;
  ProfilerFrameScope& operator=(const ProfilerFrameScope&) = delete;
};

}  // namespace evo_engine
