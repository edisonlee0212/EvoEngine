#include "Profiler.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <mutex>
#include <sstream>
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

std::string JsonEscape(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char c : value) {
    switch (c) {
      case '"':
        escaped += "\\\"";
        break;
      case '\\':
        escaped += "\\\\";
        break;
      case '\b':
        escaped += "\\b";
        break;
      case '\f':
        escaped += "\\f";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          std::ostringstream stream;
          stream << "\\u" << std::hex << std::uppercase << std::setw(4) << std::setfill('0')
                 << static_cast<int>(static_cast<unsigned char>(c));
          escaped += stream.str();
        } else {
          escaped += c;
        }
        break;
    }
  }
  return escaped;
}

void AccumulateTotal(std::vector<ProfilerAggregateTotal>& totals, std::unordered_map<std::string, size_t>& indices,
                     const std::string& key, const std::string& name, const std::string& category,
                     const double duration_ms) {
  auto search = indices.find(key);
  if (search == indices.end()) {
    search = indices.emplace(key, totals.size()).first;
    auto& total = totals.emplace_back();
    total.name = name;
    total.category = category;
  }

  auto& total = totals[search->second];
  ++total.count;
  total.total_ms += duration_ms;
  total.average_ms = total.total_ms / total.count;
  total.max_ms = std::max(total.max_ms, duration_ms);
}

void SortTotals(std::vector<ProfilerAggregateTotal>& totals) {
  std::sort(totals.begin(), totals.end(), [](const ProfilerAggregateTotal& a, const ProfilerAggregateTotal& b) {
    if (a.total_ms != b.total_ms) {
      return a.total_ms > b.total_ms;
    }
    if (a.category != b.category) {
      return a.category < b.category;
    }
    return a.name < b.name;
  });
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

ProfilerFrameStats evo_engine::BuildProfilerFrameStats(const ProfilerFrameSnapshot& snapshot) {
  ProfilerFrameStats stats;
  stats.frame_index = snapshot.frame_index;
  stats.duration_ms = snapshot.duration_ms;
  stats.event_count = static_cast<uint32_t>(snapshot.events.size());

  std::unordered_map<uint64_t, size_t> lane_indices;
  std::unordered_map<std::string, size_t> category_indices;
  std::unordered_map<std::string, size_t> event_indices;
  for (const auto& event : snapshot.events) {
    stats.total_event_ms += event.duration_ms;
    stats.max_event_ms = std::max(stats.max_event_ms, event.duration_ms);

    auto lane_search = lane_indices.find(event.thread_id);
    if (lane_search == lane_indices.end()) {
      lane_search = lane_indices.emplace(event.thread_id, stats.thread_lanes.size()).first;
      auto& lane = stats.thread_lanes.emplace_back();
      lane.thread_id = event.thread_id;
      lane.thread_name = event.thread_name;
    }
    auto& lane = stats.thread_lanes[lane_search->second];
    lane.total_ms += event.duration_ms;
    lane.events.emplace_back(event);

    AccumulateTotal(stats.category_totals, category_indices, event.category, event.category, event.category,
                    event.duration_ms);
    AccumulateTotal(stats.named_event_totals, event_indices, event.category + "\n" + event.name, event.name,
                    event.category, event.duration_ms);
  }

  for (auto& lane : stats.thread_lanes) {
    std::sort(lane.events.begin(), lane.events.end(), [](const ProfilerScopeEvent& a, const ProfilerScopeEvent& b) {
      if (a.start_ms != b.start_ms) {
        return a.start_ms < b.start_ms;
      }
      return a.depth < b.depth;
    });
  }
  std::sort(stats.thread_lanes.begin(), stats.thread_lanes.end(),
            [](const ProfilerThreadLane& a, const ProfilerThreadLane& b) {
              if (a.thread_name != b.thread_name) {
                return a.thread_name < b.thread_name;
              }
              return a.thread_id < b.thread_id;
            });
  SortTotals(stats.category_totals);
  SortTotals(stats.named_event_totals);
  return stats;
}

std::vector<ProfilerFrameStats> evo_engine::BuildProfilerFrameStatsHistory(
    const std::vector<ProfilerFrameSnapshot>& snapshots) {
  std::vector<ProfilerFrameStats> stats;
  stats.reserve(snapshots.size());
  for (const auto& snapshot : snapshots) {
    stats.emplace_back(BuildProfilerFrameStats(snapshot));
  }
  return stats;
}

bool evo_engine::ExportProfilerChromeTrace(const std::filesystem::path& path,
                                           const std::vector<ProfilerFrameSnapshot>& snapshots, std::string* error) {
  if (path.empty()) {
    if (error) {
      *error = "Profiler trace export path is empty.";
    }
    return false;
  }

  std::error_code filesystem_error;
  const auto parent = path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, filesystem_error);
    if (filesystem_error) {
      if (error) {
        *error = filesystem_error.message();
      }
      return false;
    }
  }

  std::ofstream stream(path);
  if (!stream.is_open()) {
    if (error) {
      *error = "Failed to open profiler trace export file.";
    }
    return false;
  }

  stream << "{\"traceEvents\":[";
  bool first_event = true;
  double frame_start_ms = 0.0;
  for (const auto& frame : snapshots) {
    for (const auto& event : frame.events) {
      if (!first_event) {
        stream << ',';
      }
      first_event = false;
      const auto timestamp_us = static_cast<uint64_t>((frame_start_ms + event.start_ms) * 1000.0);
      const auto duration_us = static_cast<uint64_t>(event.duration_ms * 1000.0);
      stream << "{\"name\":\"" << JsonEscape(event.name) << "\",\"cat\":\"" << JsonEscape(event.category)
             << "\",\"ph\":\"X\",\"ts\":" << timestamp_us << ",\"dur\":" << duration_us
             << ",\"pid\":1,\"tid\":" << event.thread_id << ",\"args\":{\"frame\":" << frame.frame_index
             << ",\"thread\":\"" << JsonEscape(event.thread_name) << "\"}}";
    }
    frame_start_ms += frame.duration_ms;
  }
  stream << "]}";
  if (!stream.good()) {
    if (error) {
      *error = "Failed while writing profiler trace export file.";
    }
    return false;
  }
  return true;
}

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

void Profiler::ClearFrameHistory() {
  auto& state = State();
  std::lock_guard lock(state.mutex);
  state.completed_events.clear();
  state.frame_history.clear();
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

ProfilerFrameStats Profiler::GetLatestFrameStatsSnapshot() const {
  return BuildProfilerFrameStats(GetLatestFrameSnapshot());
}

std::vector<ProfilerFrameStats> Profiler::GetFrameStatsHistorySnapshot() const {
  return BuildProfilerFrameStatsHistory(GetFrameHistorySnapshot());
}

bool Profiler::ExportChromeTrace(const std::filesystem::path& path, std::string* error) const {
  return ExportProfilerChromeTrace(path, GetFrameHistorySnapshot(), error);
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
