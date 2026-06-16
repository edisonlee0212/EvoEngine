#include "Profiler.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <utility>

using namespace evo_engine;

namespace {
using ProfilerClock = std::chrono::steady_clock;

struct ActiveProfilerScope {
  std::string name;
  std::string category;
  ProfilerClock::time_point start_time;
  ProfilerClock::time_point frame_start_time;
  uint64_t frame_index = 0;
  uint64_t thread_id = 0;
  uint32_t depth = 0;
};

thread_local std::vector<ActiveProfilerScope> g_active_scopes;

uint64_t CurrentThreadId() {
  return static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
}

double MillisecondsBetween(const ProfilerClock::time_point start_time, const ProfilerClock::time_point end_time) {
  return std::chrono::duration<double, std::milli>(end_time - start_time).count();
}

std::string DefaultThreadName(const uint64_t thread_id) {
  return "Thread " + std::to_string(thread_id);
}

class ProfilerState {
 public:
  std::atomic_bool enabled{true};
  mutable std::mutex mutex;
  uint64_t active_frame_index = 0;
  bool frame_open = false;
  ProfilerClock::time_point frame_start_time = ProfilerClock::now();
  std::vector<ProfilerScopeEvent> completed_events;
  std::deque<ProfilerFrameSnapshot> frame_history;
  std::unordered_map<uint64_t, std::string> thread_names;
  size_t max_frame_history = 240;
  size_t max_completed_event_size = 65536;
};

ProfilerState& State() {
  static ProfilerState state;
  return state;
}
}  // namespace

Profiler& Profiler::GetInstance() {
  static Profiler profiler;
  return profiler;
}

void Profiler::SetEnabled(const bool enabled) {
  State().enabled = enabled;
}

bool Profiler::IsEnabled() const {
  return State().enabled.load();
}

void Profiler::Reset() {
  auto& state = State();
  std::lock_guard lock(state.mutex);
  state.active_frame_index = 0;
  state.frame_open = false;
  state.frame_start_time = ProfilerClock::now();
  state.completed_events.clear();
  state.frame_history.clear();
  state.thread_names.clear();
  g_active_scopes.clear();
}

void Profiler::SetMaxFrameHistory(const size_t max_frame_history) {
  auto& state = State();
  std::lock_guard lock(state.mutex);
  state.max_frame_history = std::max<size_t>(1, max_frame_history);
  while (state.frame_history.size() > state.max_frame_history) {
    state.frame_history.pop_front();
  }
}

void Profiler::RegisterThread(const std::string& name) {
  auto& state = State();
  std::lock_guard lock(state.mutex);
  state.thread_names[CurrentThreadId()] = name;
}

uint64_t Profiler::BeginFrame() {
  if (!IsEnabled()) {
    return 0;
  }
  auto& state = State();
  std::lock_guard lock(state.mutex);
  state.frame_open = true;
  state.frame_start_time = ProfilerClock::now();
  return ++state.active_frame_index;
}

void Profiler::EndFrame() {
  if (!IsEnabled()) {
    return;
  }
  auto& state = State();
  std::lock_guard lock(state.mutex);
  if (!state.frame_open) {
    return;
  }

  ProfilerFrameSnapshot frame;
  frame.frame_index = state.active_frame_index;
  frame.duration_ms = MillisecondsBetween(state.frame_start_time, ProfilerClock::now());
  auto new_end = std::remove_if(state.completed_events.begin(), state.completed_events.end(),
                                [&](const ProfilerScopeEvent& event) {
                                  if (event.frame_index <= frame.frame_index) {
                                    frame.events.emplace_back(event);
                                    return true;
                                  }
                                  return false;
                                });
  state.completed_events.erase(new_end, state.completed_events.end());
  state.frame_history.emplace_back(std::move(frame));
  while (state.frame_history.size() > state.max_frame_history) {
    state.frame_history.pop_front();
  }
  state.frame_open = false;
}

ProfilerScopeToken Profiler::BeginScope(const std::string& name, const std::string& category) {
  if (!IsEnabled()) {
    return {};
  }
  auto& state = State();
  const auto now = ProfilerClock::now();
  ActiveProfilerScope scope;
  scope.name = name;
  scope.category = category;
  scope.start_time = now;
  scope.thread_id = CurrentThreadId();
  scope.depth = static_cast<uint32_t>(g_active_scopes.size());
  {
    std::lock_guard lock(state.mutex);
    scope.frame_index = state.active_frame_index;
    scope.frame_start_time = state.frame_open ? state.frame_start_time : now;
  }
  g_active_scopes.emplace_back(std::move(scope));
  return {true};
}

void Profiler::EndScope(ProfilerScopeToken& token) {
  if (!token.active) {
    return;
  }
  token.active = false;
  if (g_active_scopes.empty()) {
    return;
  }

  auto scope = std::move(g_active_scopes.back());
  g_active_scopes.pop_back();
  ProfilerScopeEvent event;
  event.frame_index = scope.frame_index;
  event.thread_id = scope.thread_id;
  event.name = std::move(scope.name);
  event.category = std::move(scope.category);
  event.depth = scope.depth;
  const auto end_time = ProfilerClock::now();
  event.start_ms = MillisecondsBetween(scope.frame_start_time, scope.start_time);
  event.duration_ms = MillisecondsBetween(scope.start_time, end_time);

  auto& state = State();
  std::lock_guard lock(state.mutex);
  if (const auto search = state.thread_names.find(event.thread_id); search != state.thread_names.end()) {
    event.thread_name = search->second;
  } else {
    event.thread_name = DefaultThreadName(event.thread_id);
  }
  state.completed_events.emplace_back(std::move(event));
  if (state.completed_events.size() > state.max_completed_event_size) {
    state.completed_events.erase(state.completed_events.begin());
  }
}

ProfilerFrameSnapshot Profiler::GetLatestFrameSnapshot() const {
  const auto& state = State();
  std::lock_guard lock(state.mutex);
  if (state.frame_history.empty()) {
    return {};
  }
  return state.frame_history.back();
}

std::vector<ProfilerFrameSnapshot> Profiler::GetFrameHistorySnapshot() const {
  const auto& state = State();
  std::lock_guard lock(state.mutex);
  return {state.frame_history.begin(), state.frame_history.end()};
}

ProfilerScope::ProfilerScope(const std::string& name, const std::string& category)
    : token_(Profiler::GetInstance().BeginScope(name, category)) {
}

ProfilerScope::~ProfilerScope() {
  Profiler::GetInstance().EndScope(token_);
}

ProfilerFrameScope::ProfilerFrameScope() {
  Profiler::GetInstance().BeginFrame();
}

ProfilerFrameScope::~ProfilerFrameScope() {
  Profiler::GetInstance().EndFrame();
}
