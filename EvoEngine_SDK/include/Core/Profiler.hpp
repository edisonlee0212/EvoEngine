#pragma once

#include <cstdint>
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

struct ProfilerScopeToken {
  bool active = false;
};

class Profiler final {
 public:
  static Profiler& GetInstance();

  void SetEnabled(bool enabled);
  [[nodiscard]] bool IsEnabled() const;
  void Reset();
  void SetMaxFrameHistory(size_t max_frame_history);

  void RegisterThread(const std::string& name);
  uint64_t BeginFrame();
  void EndFrame();
  [[nodiscard]] ProfilerScopeToken BeginScope(const std::string& name, const std::string& category = "CPU");
  void EndScope(ProfilerScopeToken& token);

  [[nodiscard]] ProfilerFrameSnapshot GetLatestFrameSnapshot() const;
  [[nodiscard]] std::vector<ProfilerFrameSnapshot> GetFrameHistorySnapshot() const;
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
