#include "Profiler.hpp"
#include "ProfilerPanelModel.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>

using namespace evo_engine;

namespace {
using ProfilerClock = std::chrono::steady_clock;

struct ActiveProfilerScope {
  std::string name;
  std::string category;
  std::string stable_id;
  std::string owner_name;
  ProfilerClock::time_point start_time;
  ProfilerClock::time_point frame_start_time;
  uint64_t frame_index = 0;
  uint64_t thread_id = 0;
  uint64_t capture_session_index = 0;
  uint32_t depth = 0;
};

thread_local std::vector<ActiveProfilerScope> g_active_scopes;

uint64_t CurrentThreadId() {
  return static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
}

double MillisecondsBetween(const ProfilerClock::time_point start_time, const ProfilerClock::time_point end_time) {
  return std::chrono::duration<double, std::milli>(end_time - start_time).count();
}

double ValidDuration(const double duration_ms) {
  return std::isfinite(duration_ms) ? std::max(0.0, duration_ms) : 0.0;
}

double ValidStart(const double start_ms) {
  return std::isfinite(start_ms) ? start_ms : std::numeric_limits<double>::max();
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

ProfilerHierarchyNode& FindOrAddHierarchyNode(std::vector<ProfilerHierarchyNode>& nodes,
                                              const ProfilerScopeEvent& event) {
  const auto search = std::find_if(nodes.begin(), nodes.end(), [&](const ProfilerHierarchyNode& node) {
    return event.stable_id.empty()
               ? node.stable_id.empty() && node.name == event.name && node.category == event.category
               : node.stable_id == event.stable_id;
  });
  if (search != nodes.end()) {
    return *search;
  }
  auto& node = nodes.emplace_back();
  node.name = event.name;
  node.category = event.category;
  node.stable_id = event.stable_id;
  node.owner_name = event.owner_name;
  return node;
}

void FinishHierarchy(std::vector<ProfilerHierarchyNode>& nodes) {
  for (auto& node : nodes) {
    FinishHierarchy(node.children);
    double child_ms = 0.0;
    for (const auto& child : node.children) {
      child_ms += child.inclusive_ms;
    }
    node.self_ms = std::max(0.0, node.inclusive_ms - child_ms);
  }
}

void BuildHierarchy(ProfilerThreadLane& lane) {
  struct StackEntry {
    uint32_t depth = 0;
    double end_ms = 0.0;
    ProfilerHierarchyNode* node = nullptr;
  };
  std::vector<StackEntry> stack;
  for (const auto& event : lane.events) {
    const double start_ms = ValidStart(event.start_ms);
    const double duration_ms = ValidDuration(event.duration_ms);
    while (!stack.empty() && (stack.back().depth >= event.depth || stack.back().end_ms <= start_ms)) {
      stack.pop_back();
    }
    auto& siblings = stack.empty() ? lane.hierarchy : stack.back().node->children;
    auto& node = FindOrAddHierarchyNode(siblings, event);
    ++node.count;
    node.inclusive_ms += duration_ms;
    node.max_ms = std::max(node.max_ms, duration_ms);
    stack.push_back({event.depth, start_ms + duration_ms, &node});
  }
  FinishHierarchy(lane.hierarchy);
}

class ProfilerState {
 public:
  std::atomic_bool enabled{false};
  mutable std::mutex mutex;
  uint64_t active_frame_index = 0;
  uint64_t active_application_frame_index = 0;
  uint64_t capture_session_index = 0;
  bool frame_open = false;
  ProfilerClock::time_point frame_start_time = ProfilerClock::now();
  std::vector<ProfilerScopeEvent> completed_events;
  std::vector<ProfilerCounter> counters;
  uint32_t dropped_event_count = 0;
  std::deque<ProfilerFrameSnapshot> frame_history;
  std::unordered_map<uint64_t, std::string> thread_names;
  std::unordered_map<uint64_t, RegisteredProfilerItem> registered_items;
  std::unordered_map<std::string, uint64_t> registered_item_ids;
  uint64_t next_registered_item_handle = 1;
  uint64_t registration_revision = 0;
  size_t max_frame_history = 240;
  size_t max_completed_event_size = 65536;
};

ProfilerState& State() {
  static ProfilerState state;
  return state;
}
}  // namespace

double evo_engine::profiler_panel_detail::IntervalUnionMilliseconds(std::vector<TimingInterval> intervals) {
  intervals.erase(std::remove_if(intervals.begin(), intervals.end(),
                                 [](const auto& interval) {
                                   return !std::isfinite(interval.start_ms) || !std::isfinite(interval.duration_ms) ||
                                          interval.duration_ms <= 0.0;
                                 }),
                  intervals.end());
  if (intervals.empty())
    return 0.0;
  std::sort(intervals.begin(), intervals.end(), [](const auto& left, const auto& right) {
    return std::tie(left.start_ms, left.duration_ms) < std::tie(right.start_ms, right.duration_ms);
  });
  double total = 0.0;
  double start = intervals.front().start_ms;
  double end = start + intervals.front().duration_ms;
  for (size_t index = 1; index < intervals.size(); ++index) {
    const double next_start = intervals[index].start_ms;
    const double next_end = next_start + intervals[index].duration_ms;
    if (next_start <= end) {
      end = std::max(end, next_end);
    } else {
      total += end - start;
      start = next_start;
      end = next_end;
    }
  }
  return total + end - start;
}

evo_engine::profiler_panel_detail::FrameOverviewSample evo_engine::profiler_panel_detail::BuildFrameOverviewSample(
    const double cpu_wall_ms, const double synchronization_ms, const std::optional<double> gpu_ms) {
  FrameOverviewSample sample;
  sample.cpu_wall_ms = std::max(0.0, cpu_wall_ms);
  sample.synchronization_ms = std::clamp(synchronization_ms, 0.0, sample.cpu_wall_ms);
  sample.cpu_active_ms = sample.cpu_wall_ms - sample.synchronization_ms;
  sample.gpu_available = gpu_ms.has_value() && std::isfinite(*gpu_ms) && *gpu_ms >= 0.0;
  sample.gpu_ms = sample.gpu_available ? *gpu_ms : 0.0;
  sample.total_ms = sample.gpu_available ? std::max(sample.cpu_wall_ms, sample.gpu_ms) : sample.cpu_wall_ms;
  return sample;
}

evo_engine::profiler_panel_detail::CpuExecutorGroup evo_engine::profiler_panel_detail::ClassifyCpuExecutor(
    const std::string& thread_name) {
  if (thread_name == "MainThread")
    return CpuExecutorGroup::MainThread;
  if (thread_name.rfind("Worker-", 0) == 0)
    return CpuExecutorGroup::Worker;
  if (thread_name.rfind("AssetIo-", 0) == 0)
    return CpuExecutorGroup::AssetIo;
  if (thread_name.rfind("Gpu-", 0) == 0)
    return CpuExecutorGroup::GpuSubmission;
  if (thread_name.rfind("Render-", 0) == 0)
    return CpuExecutorGroup::Render;
  if (thread_name.rfind("Background-", 0) == 0)
    return CpuExecutorGroup::Background;
  return CpuExecutorGroup::Other;
}

const char* evo_engine::profiler_panel_detail::CpuExecutorGroupName(const CpuExecutorGroup group) {
  switch (group) {
    case CpuExecutorGroup::MainThread:
      return "Main Thread";
    case CpuExecutorGroup::Worker:
      return "Worker";
    case CpuExecutorGroup::AssetIo:
      return "Asset I/O";
    case CpuExecutorGroup::GpuSubmission:
      return "GPU Submission (CPU)";
    case CpuExecutorGroup::Render:
      return "Render";
    case CpuExecutorGroup::Background:
      return "Background";
    case CpuExecutorGroup::Other:
      return "Other";
  }
  return "Other";
}

std::string evo_engine::profiler_panel_detail::StableHierarchyKey(const CpuExecutorGroup group,
                                                                  const std::string& parent_path,
                                                                  const std::string& category,
                                                                  const std::string& name) {
  return std::to_string(static_cast<uint8_t>(group)) + "\x1f" + parent_path + "\x1f" + category + "\x1f" + name;
}

ProfilerFrameStats evo_engine::BuildProfilerFrameStats(const ProfilerFrameSnapshot& snapshot) {
  ProfilerFrameStats stats;
  stats.frame_index = snapshot.frame_index;
  stats.application_frame_index = snapshot.application_frame_index;
  stats.capture_session_index = snapshot.capture_session_index;
  stats.counters = snapshot.counters;
  stats.duration_ms = snapshot.duration_ms;
  stats.event_count = static_cast<uint32_t>(snapshot.events.size());
  stats.dropped_event_count = snapshot.dropped_event_count;

  std::unordered_map<uint64_t, size_t> lane_indices;
  std::unordered_map<std::string, size_t> category_indices;
  std::unordered_map<std::string, size_t> event_indices;
  for (const auto& event : snapshot.events) {
    const double duration_ms = ValidDuration(event.duration_ms);
    stats.total_event_ms += duration_ms;
    stats.max_event_ms = std::max(stats.max_event_ms, duration_ms);

    auto lane_search = lane_indices.find(event.thread_id);
    if (lane_search == lane_indices.end()) {
      lane_search = lane_indices.emplace(event.thread_id, stats.thread_lanes.size()).first;
      auto& lane = stats.thread_lanes.emplace_back();
      lane.thread_id = event.thread_id;
      lane.thread_name = event.thread_name;
    }
    auto& lane = stats.thread_lanes[lane_search->second];
    lane.total_ms += duration_ms;
    lane.events.emplace_back(event);

    AccumulateTotal(stats.category_totals, category_indices, event.category, event.category, event.category,
                    duration_ms);
    AccumulateTotal(stats.named_event_totals, event_indices, event.category + "\n" + event.name, event.name,
                    event.category, duration_ms);
  }

  for (auto& lane : stats.thread_lanes) {
    std::sort(lane.events.begin(), lane.events.end(), [](const ProfilerScopeEvent& a, const ProfilerScopeEvent& b) {
      if (ValidStart(a.start_ms) != ValidStart(b.start_ms)) {
        return ValidStart(a.start_ms) < ValidStart(b.start_ms);
      }
      return a.depth < b.depth;
    });
    BuildHierarchy(lane);
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

namespace {
struct HistoryNodeAccumulator {
  std::string name;
  std::string category;
  std::vector<double> inclusive;
  std::vector<double> self;
  size_t observed_frame_count = 0;
  size_t last_observed_frame = std::numeric_limits<size_t>::max();
  std::vector<HistoryNodeAccumulator> children;
};

struct ThreadHistoryAccumulator {
  uint64_t thread_id = 0;
  std::string thread_name;
  std::vector<double> root_work;
  size_t observed_frame_count = 0;
  std::vector<HistoryNodeAccumulator> hierarchy;
};

struct CounterHistoryAccumulator {
  std::string name;
  std::string category;
  std::string unit;
  std::vector<double> values;
  size_t observed_frame_count = 0;
  size_t last_observed_frame = std::numeric_limits<size_t>::max();
};

ProfilerDurationSummary SummarizeDurations(std::vector<double> values, const size_t frame_count,
                                           const size_t observed_frame_count, const size_t selected_frame_index) {
  ProfilerDurationSummary summary;
  summary.frame_count = frame_count;
  summary.observed_frame_count = observed_frame_count;
  if (frame_count == 0)
    return summary;
  for (const auto value : values) {
    summary.average_ms += value;
    summary.maximum_ms = std::max(summary.maximum_ms, value);
  }
  summary.average_ms /= static_cast<double>(frame_count);
  values.resize(frame_count, 0.0);
  summary.selected_ms = values[std::min(selected_frame_index, frame_count - 1)];
  std::sort(values.begin(), values.end());
  summary.median_ms = values[(frame_count - 1) / 2];
  summary.p95_ms = values[static_cast<size_t>(std::ceil(static_cast<double>(frame_count) * 0.95)) - 1];
  return summary;
}

void AccumulateHistoryNodes(std::vector<HistoryNodeAccumulator>& accumulators,
                            const std::vector<ProfilerHierarchyNode>& nodes, const size_t frame_index) {
  for (const auto& node : nodes) {
    const auto search = std::find_if(accumulators.begin(), accumulators.end(), [&](const auto& accumulator) {
      return accumulator.name == node.name && accumulator.category == node.category;
    });
    auto* accumulator = search == accumulators.end()
                            ? &accumulators.emplace_back(HistoryNodeAccumulator{node.name, node.category})
                            : &*search;
    accumulator->inclusive.resize(frame_index + 1, 0.0);
    accumulator->self.resize(frame_index + 1, 0.0);
    accumulator->inclusive[frame_index] += node.inclusive_ms;
    accumulator->self[frame_index] += node.self_ms;
    if (accumulator->last_observed_frame != frame_index) {
      accumulator->last_observed_frame = frame_index;
      accumulator->observed_frame_count++;
    }
    AccumulateHistoryNodes(accumulator->children, node.children, frame_index);
  }
}

std::vector<ProfilerHistoryNode> FinishHistoryNodes(std::vector<HistoryNodeAccumulator> accumulators,
                                                    const size_t frame_count, const size_t selected_frame_index) {
  std::vector<ProfilerHistoryNode> nodes;
  nodes.reserve(accumulators.size());
  for (auto& accumulator : accumulators) {
    nodes.push_back({std::move(accumulator.name), std::move(accumulator.category),
                     SummarizeDurations(std::move(accumulator.inclusive), frame_count, accumulator.observed_frame_count,
                                        selected_frame_index),
                     SummarizeDurations(std::move(accumulator.self), frame_count, accumulator.observed_frame_count,
                                        selected_frame_index),
                     FinishHistoryNodes(std::move(accumulator.children), frame_count, selected_frame_index)});
  }
  return nodes;
}
}  // namespace

ProfilerHistoryStats evo_engine::BuildProfilerHistoryStats(const std::vector<ProfilerFrameStats>& frames,
                                                           const size_t selected_frame_index) {
  ProfilerHistoryStats result;
  result.frame_count = frames.size();
  std::vector<double> frame_durations;
  std::vector<double> main_thread_wall;
  std::vector<double> worker_cpu_work;
  std::vector<ThreadHistoryAccumulator> thread_accumulators;
  std::vector<CounterHistoryAccumulator> counter_accumulators;
  frame_durations.reserve(frames.size());
  main_thread_wall.reserve(frames.size());
  worker_cpu_work.reserve(frames.size());
  for (size_t frame_index = 0; frame_index < frames.size(); frame_index++) {
    const auto& frame = frames[frame_index];
    frame_durations.emplace_back(frame.duration_ms);
    double frame_main_thread_wall = 0.0;
    double frame_worker_cpu_work = 0.0;
    for (const auto& lane : frame.thread_lanes) {
      const auto search = std::find_if(thread_accumulators.begin(), thread_accumulators.end(), [&](const auto& entry) {
        return entry.thread_id == lane.thread_id;
      });
      auto* accumulator =
          search == thread_accumulators.end()
              ? &thread_accumulators.emplace_back(ThreadHistoryAccumulator{lane.thread_id, lane.thread_name})
              : &*search;
      double root_work = 0.0;
      for (const auto& root : lane.hierarchy)
        root_work += root.inclusive_ms;
      accumulator->root_work.resize(frame_index + 1, 0.0);
      accumulator->root_work[frame_index] += root_work;
      accumulator->observed_frame_count++;
      AccumulateHistoryNodes(accumulator->hierarchy, lane.hierarchy, frame_index);
      if (lane.thread_name == "MainThread")
        frame_main_thread_wall += root_work;
      else
        frame_worker_cpu_work += root_work;
    }
    main_thread_wall.emplace_back(frame_main_thread_wall);
    worker_cpu_work.emplace_back(frame_worker_cpu_work);
    for (const auto& counter : frame.counters) {
      const auto search =
          std::find_if(counter_accumulators.begin(), counter_accumulators.end(), [&](const auto& entry) {
            return entry.name == counter.name && entry.category == counter.category && entry.unit == counter.unit;
          });
      auto* accumulator = search == counter_accumulators.end()
                              ? &counter_accumulators.emplace_back(
                                    CounterHistoryAccumulator{counter.name, counter.category, counter.unit})
                              : &*search;
      accumulator->values.resize(frame_index + 1, 0.0);
      accumulator->values[frame_index] = counter.value;
      if (accumulator->last_observed_frame != frame_index) {
        accumulator->last_observed_frame = frame_index;
        accumulator->observed_frame_count++;
      }
    }
  }
  result.frame_duration =
      SummarizeDurations(std::move(frame_durations), frames.size(), frames.size(), selected_frame_index);
  result.main_thread_wall =
      SummarizeDurations(std::move(main_thread_wall), frames.size(), frames.size(), selected_frame_index);
  result.worker_cpu_work =
      SummarizeDurations(std::move(worker_cpu_work), frames.size(), frames.size(), selected_frame_index);
  result.threads.reserve(thread_accumulators.size());
  for (auto& accumulator : thread_accumulators) {
    result.threads.push_back(
        {accumulator.thread_id, std::move(accumulator.thread_name),
         SummarizeDurations(std::move(accumulator.root_work), frames.size(), accumulator.observed_frame_count,
                            selected_frame_index),
         FinishHistoryNodes(std::move(accumulator.hierarchy), frames.size(), selected_frame_index)});
  }
  result.counters.reserve(counter_accumulators.size());
  for (auto& accumulator : counter_accumulators) {
    const auto summary = SummarizeDurations(std::move(accumulator.values), frames.size(),
                                            accumulator.observed_frame_count, selected_frame_index);
    result.counters.push_back({std::move(accumulator.name), std::move(accumulator.category),
                               std::move(accumulator.unit), summary.frame_count, summary.observed_frame_count,
                               summary.selected_ms, summary.average_ms, summary.median_ms, summary.p95_ms,
                               summary.maximum_ms});
  }
  return result;
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
  auto& state = State();
  std::lock_guard lock(state.mutex);
  if (state.enabled == enabled) {
    return;
  }
  if (enabled) {
    ++state.capture_session_index;
  } else {
    state.frame_open = false;
    state.completed_events.clear();
    state.counters.clear();
    state.dropped_event_count = 0;
  }
  state.enabled = enabled;
}

bool Profiler::IsEnabled() const {
  return State().enabled.load();
}

void Profiler::Reset() {
  auto& state = State();
  std::lock_guard lock(state.mutex);
  state.active_frame_index = 0;
  state.active_application_frame_index = 0;
  state.capture_session_index = state.enabled ? 1 : 0;
  state.frame_open = false;
  state.frame_start_time = ProfilerClock::now();
  state.completed_events.clear();
  state.counters.clear();
  state.dropped_event_count = 0;
  state.frame_history.clear();
  state.thread_names.clear();
  g_active_scopes.clear();
}

void Profiler::ClearFrameHistory() {
  auto& state = State();
  std::lock_guard lock(state.mutex);
  state.completed_events.clear();
  state.counters.clear();
  state.dropped_event_count = 0;
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

ProfilerItemHandle Profiler::RegisterItem(const std::string& owner_name, const ProfilerItemDescriptor& descriptor) {
  if (owner_name.empty() || descriptor.local_id.empty() || descriptor.display_name.empty() ||
      (!descriptor.cpu && !descriptor.gpu)) {
    return {};
  }
  const std::string stable_id = owner_name + "." + descriptor.local_id;
  auto& state = State();
  std::lock_guard lock(state.mutex);
  if (state.registered_item_ids.find(stable_id) != state.registered_item_ids.end()) {
    return {};
  }
  const ProfilerItemHandle handle{state.next_registered_item_handle++};
  state.registered_items.emplace(handle.value, RegisteredProfilerItem{handle, owner_name, stable_id, descriptor});
  state.registered_item_ids.emplace(stable_id, handle.value);
  ++state.registration_revision;
  return handle;
}

void Profiler::UnregisterOwner(const std::string& owner_name) {
  if (owner_name.empty())
    return;
  auto& state = State();
  std::lock_guard lock(state.mutex);
  bool removed = false;
  for (auto iterator = state.registered_items.begin(); iterator != state.registered_items.end();) {
    if (iterator->second.owner_name == owner_name) {
      state.registered_item_ids.erase(iterator->second.stable_id);
      iterator = state.registered_items.erase(iterator);
      removed = true;
    } else {
      ++iterator;
    }
  }
  const auto remove_owner_events = [&](auto& events) {
    events.erase(std::remove_if(events.begin(), events.end(),
                                [&](const auto& event) {
                                  return event.owner_name == owner_name;
                                }),
                 events.end());
  };
  remove_owner_events(state.completed_events);
  for (auto& frame : state.frame_history)
    remove_owner_events(frame.events);
  if (removed)
    ++state.registration_revision;
}

std::optional<RegisteredProfilerItem> Profiler::FindRegisteredItem(const ProfilerItemHandle handle) const {
  if (!handle)
    return std::nullopt;
  const auto& state = State();
  std::lock_guard lock(state.mutex);
  const auto search = state.registered_items.find(handle.value);
  return search == state.registered_items.end() ? std::nullopt : std::optional<RegisteredProfilerItem>{search->second};
}

std::vector<RegisteredProfilerItem> Profiler::GetRegisteredItemsSnapshot() const {
  const auto& state = State();
  std::lock_guard lock(state.mutex);
  std::vector<RegisteredProfilerItem> items;
  items.reserve(state.registered_items.size());
  for (const auto& [handle, item] : state.registered_items)
    items.emplace_back(item);
  std::sort(items.begin(), items.end(), [](const auto& left, const auto& right) {
    return std::tie(left.owner_name, left.stable_id) < std::tie(right.owner_name, right.stable_id);
  });
  return items;
}

uint64_t Profiler::GetRegistrationRevision() const {
  const auto& state = State();
  std::lock_guard lock(state.mutex);
  return state.registration_revision;
}

uint64_t Profiler::BeginFrame(const uint64_t application_frame_index) {
  if (!IsEnabled()) {
    return 0;
  }
  auto& state = State();
  std::lock_guard lock(state.mutex);
  if (!state.enabled) {
    return 0;
  }
  state.frame_open = true;
  state.active_application_frame_index = application_frame_index;
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
  frame.application_frame_index = state.active_application_frame_index;
  frame.capture_session_index = state.capture_session_index;
  frame.dropped_event_count = state.dropped_event_count;
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
  state.dropped_event_count = 0;
  frame.counters = std::move(state.counters);
  state.frame_history.emplace_back(std::move(frame));
  while (state.frame_history.size() > state.max_frame_history) {
    state.frame_history.pop_front();
  }
  state.frame_open = false;
}

void Profiler::RecordCounter(const std::string& name, const double value, const std::string& category,
                             const std::string& unit) {
  if (!IsEnabled())
    return;
  auto& state = State();
  std::lock_guard lock(state.mutex);
  if (!state.enabled || !state.frame_open)
    return;
  const auto search = std::find_if(state.counters.begin(), state.counters.end(), [&](const auto& counter) {
    return counter.name == name && counter.category == category && counter.unit == unit;
  });
  if (search == state.counters.end())
    state.counters.push_back({name, category, unit, value});
  else
    search->value = value;
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
    if (!state.frame_open) {
      return {};
    }
    scope.frame_index = state.active_frame_index;
    scope.capture_session_index = state.capture_session_index;
    scope.frame_start_time = state.frame_open ? state.frame_start_time : now;
  }
  const auto capture_session_index = scope.capture_session_index;
  g_active_scopes.emplace_back(std::move(scope));
  return {true, capture_session_index};
}

ProfilerScopeToken Profiler::BeginScope(const ProfilerItemHandle handle) {
  if (!IsEnabled())
    return {};
  const auto item = FindRegisteredItem(handle);
  if (!item || !item->descriptor.cpu)
    return {};
  auto token = BeginScope(item->descriptor.display_name, item->descriptor.cpu_category);
  if (token.active && !g_active_scopes.empty()) {
    g_active_scopes.back().stable_id = item->stable_id;
    g_active_scopes.back().owner_name = item->owner_name;
  }
  return token;
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
  if (token.capture_session_index != scope.capture_session_index) {
    return;
  }
  ProfilerScopeEvent event;
  event.frame_index = scope.frame_index;
  event.thread_id = scope.thread_id;
  event.name = std::move(scope.name);
  event.category = std::move(scope.category);
  event.stable_id = std::move(scope.stable_id);
  event.owner_name = std::move(scope.owner_name);
  event.depth = scope.depth;
  const auto end_time = ProfilerClock::now();
  event.start_ms = MillisecondsBetween(scope.frame_start_time, scope.start_time);
  event.duration_ms = MillisecondsBetween(scope.start_time, end_time);

  auto& state = State();
  std::lock_guard lock(state.mutex);
  if (!state.enabled || token.capture_session_index != state.capture_session_index) {
    return;
  }
  if (const auto search = state.thread_names.find(event.thread_id); search != state.thread_names.end()) {
    event.thread_name = search->second;
  } else {
    event.thread_name = DefaultThreadName(event.thread_id);
  }
  if (!state.frame_open || event.frame_index < state.active_frame_index) {
    const auto frame =
        std::find_if(state.frame_history.rbegin(), state.frame_history.rend(), [&](const auto& candidate) {
          return candidate.frame_index == event.frame_index;
        });
    if (frame != state.frame_history.rend()) {
      frame->events.emplace_back(std::move(event));
    }
    return;
  }
  state.completed_events.emplace_back(std::move(event));
  if (state.completed_events.size() > state.max_completed_event_size) {
    state.completed_events.erase(state.completed_events.begin());
    ++state.dropped_event_count;
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

ProfilerScope::ProfilerScope(const ProfilerItemHandle handle) : token_(Profiler::GetInstance().BeginScope(handle)) {
}

ProfilerScope::~ProfilerScope() {
  Profiler::GetInstance().EndScope(token_);
}

ProfilerFrameScope::ProfilerFrameScope(const uint64_t application_frame_index) {
  Profiler::GetInstance().BeginFrame(application_frame_index);
}

ProfilerFrameScope::~ProfilerFrameScope() {
  Profiler::GetInstance().EndFrame();
}
