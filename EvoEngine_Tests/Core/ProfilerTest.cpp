#include "Profiler.hpp"
#include "ProfilerPanelModel.hpp"
#include "TaskRuntime.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

using namespace evo_engine;

namespace {
bool HasEvent(const ProfilerFrameSnapshot& snapshot, const std::string& name, const std::string& category) {
  return std::any_of(snapshot.events.begin(), snapshot.events.end(), [&](const ProfilerScopeEvent& event) {
    return event.name == name && event.category == category;
  });
}

const ProfilerAggregateTotal* FindTotal(const std::vector<ProfilerAggregateTotal>& totals, const std::string& name,
                                        const std::string& category) {
  const auto search = std::find_if(totals.begin(), totals.end(), [&](const ProfilerAggregateTotal& total) {
    return total.name == name && total.category == category;
  });
  return search == totals.end() ? nullptr : &*search;
}

const ProfilerHierarchyNode* FindHierarchyNode(const std::vector<ProfilerHierarchyNode>& nodes,
                                               const std::string& name) {
  const auto search = std::find_if(nodes.begin(), nodes.end(), [&](const ProfilerHierarchyNode& node) {
    return node.name == name;
  });
  return search == nodes.end() ? nullptr : &*search;
}

TaskRuntimeSettings ProfilerRuntimeSettings() {
  TaskRuntimeSettings settings;
  settings.worker_thread_size = 1;
  settings.asset_io_thread_size = 0;
  settings.gpu_thread_size = 0;
  settings.render_thread_size = 0;
  settings.background_thread_size = 0;
  return settings;
}

class TempProfilerDirectory {
 public:
  TempProfilerDirectory() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::temp_directory_path() / ("EvoEngineProfilerTest_" + std::to_string(now));
    std::filesystem::create_directories(root_);
  }

  ~TempProfilerDirectory() {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] std::filesystem::path RootPath() const {
    return root_;
  }

 private:
  std::filesystem::path root_;
};
}  // namespace

TEST(Profiler, CapturesScopedEventsInFrame) {
  auto& profiler = Profiler::GetInstance();
  profiler.SetEnabled(true);
  profiler.Reset();
  profiler.RegisterThread("TestMain");

  {
    const ProfilerFrameScope frame(42);
    const ProfilerScope scope("UnitScope", "Unit");
  }

  const auto snapshot = profiler.GetLatestFrameSnapshot();
  ASSERT_EQ(snapshot.frame_index, 1);
  EXPECT_EQ(snapshot.application_frame_index, 42);
  ASSERT_EQ(snapshot.events.size(), 1);
  EXPECT_EQ(snapshot.events.front().name, "UnitScope");
  EXPECT_EQ(snapshot.events.front().category, "Unit");
  EXPECT_EQ(snapshot.events.front().thread_name, "TestMain");
  EXPECT_GE(snapshot.events.front().duration_ms, 0.0);
  EXPECT_GE(snapshot.events.front().start_ms, 0.0);
}

TEST(Profiler, DoesNotCaptureUntilEnabledAndRetainsSessionsAcrossStopRestart) {
  auto& profiler = Profiler::GetInstance();
  profiler.SetEnabled(false);
  profiler.Reset();

  {
    const ProfilerFrameScope frame;
    const ProfilerScope scope("Disabled", "Unit");
  }
  EXPECT_TRUE(profiler.GetFrameHistorySnapshot().empty());

  profiler.SetEnabled(true);
  {
    const ProfilerFrameScope frame;
    const ProfilerScope scope("FirstSession", "Unit");
  }
  profiler.SetEnabled(false);
  const auto frozen_history = profiler.GetFrameHistorySnapshot();
  ASSERT_EQ(frozen_history.size(), 1);

  profiler.SetEnabled(true);
  {
    const ProfilerFrameScope frame;
    const ProfilerScope scope("SecondSession", "Unit");
  }
  profiler.SetEnabled(false);

  const auto history = profiler.GetFrameHistorySnapshot();
  ASSERT_EQ(history.size(), 2);
  EXPECT_EQ(history.front().capture_session_index, frozen_history.front().capture_session_index);
  EXPECT_GT(history.back().capture_session_index, history.front().capture_session_index);
  EXPECT_TRUE(HasEvent(history.front(), "FirstSession", "Unit"));
  EXPECT_TRUE(HasEvent(history.back(), "SecondSession", "Unit"));
}

TEST(Profiler, DiscardsPartialFrameWhenCaptureStops) {
  auto& profiler = Profiler::GetInstance();
  profiler.SetEnabled(false);
  profiler.Reset();
  profiler.SetEnabled(true);
  {
    const ProfilerFrameScope frame;
    {
      const ProfilerScope completed_scope("Partial", "Unit");
    }
    profiler.SetEnabled(false);
  }
  EXPECT_TRUE(profiler.GetFrameHistorySnapshot().empty());
}

TEST(Profiler, AttachesScopesThatFinishLateToTheirOriginatingFrame) {
  auto& profiler = Profiler::GetInstance();
  profiler.SetEnabled(false);
  profiler.Reset();
  profiler.SetEnabled(true);
  profiler.BeginFrame();
  auto token = profiler.BeginScope("CrossFrame", "Unit");
  profiler.EndFrame();
  profiler.BeginFrame();
  profiler.EndScope(token);
  profiler.EndFrame();
  profiler.SetEnabled(false);

  const auto history = profiler.GetFrameHistorySnapshot();
  ASSERT_EQ(history.size(), 2);
  EXPECT_TRUE(HasEvent(history.front(), "CrossFrame", "Unit"));
  EXPECT_FALSE(HasEvent(history.back(), "CrossFrame", "Unit"));
}

TEST(Profiler, KeepsBoundedFrameHistory) {
  auto& profiler = Profiler::GetInstance();
  profiler.SetEnabled(true);
  profiler.Reset();
  profiler.SetMaxFrameHistory(2);

  {
    const ProfilerFrameScope frame;
  }
  {
    const ProfilerFrameScope frame;
  }
  {
    const ProfilerFrameScope frame;
  }

  const auto history = profiler.GetFrameHistorySnapshot();
  ASSERT_EQ(history.size(), 2);
  EXPECT_EQ(history.front().frame_index, 2);
  EXPECT_EQ(history.back().frame_index, 3);
}

TEST(Profiler, ClearsFrameHistoryWithoutDisablingCapture) {
  auto& profiler = Profiler::GetInstance();
  profiler.SetEnabled(true);
  profiler.Reset();

  {
    const ProfilerFrameScope frame;
    const ProfilerScope scope("BeforeClear", "Unit");
  }
  ASSERT_FALSE(profiler.GetFrameHistorySnapshot().empty());

  profiler.ClearFrameHistory();
  EXPECT_TRUE(profiler.GetFrameHistorySnapshot().empty());
  EXPECT_TRUE(profiler.IsEnabled());

  {
    const ProfilerFrameScope frame;
    const ProfilerScope scope("AfterClear", "Unit");
  }
  const auto snapshot = profiler.GetLatestFrameSnapshot();
  ASSERT_FALSE(snapshot.events.empty());
  EXPECT_EQ(snapshot.events.front().name, "AfterClear");
}

TEST(Profiler, CapturesAndOverwritesFrameCounters) {
  auto& profiler = Profiler::GetInstance();
  profiler.SetEnabled(true);
  profiler.Reset();
  {
    const ProfilerFrameScope frame;
    profiler.RecordCounter("Camera Count", 1.0, "Raster", "cameras");
    profiler.RecordCounter("Camera Count", 2.0, "Raster", "cameras");
    profiler.RecordCounter("Updated Probes", 64.0, "DDGI", "probes");
  }

  const auto snapshot = profiler.GetLatestFrameSnapshot();
  ASSERT_EQ(snapshot.counters.size(), 2);
  EXPECT_EQ(snapshot.counters[0].name, "Camera Count");
  EXPECT_DOUBLE_EQ(snapshot.counters[0].value, 2.0);
  EXPECT_EQ(snapshot.counters[1].category, "DDGI");
}

TEST(Profiler, CapturesTaskRuntimeExecutorScopes) {
  auto& profiler = Profiler::GetInstance();
  profiler.SetEnabled(true);
  profiler.Reset();
  profiler.SetMaxFrameHistory(8);

  TaskRuntime runtime;
  runtime.Initialize(ProfilerRuntimeSettings());

  TaskOptions options;
  options.executor = TaskExecutorType::Worker;
  options.debug_name = "ProfiledTask";
  {
    const ProfilerFrameScope frame;
    const auto task = runtime.Schedule({}, options, []() {
      const ProfilerScope nested_scope("NestedWork", "Unit");
    });
    runtime.Wait(task);
  }
  runtime.Shutdown();

  const auto snapshot = profiler.GetLatestFrameSnapshot();
  EXPECT_TRUE(HasEvent(snapshot, "ProfiledTask", "Task"));
  EXPECT_TRUE(HasEvent(snapshot, "NestedWork", "Unit"));
  EXPECT_TRUE(std::any_of(snapshot.events.begin(), snapshot.events.end(), [](const ProfilerScopeEvent& event) {
    return event.name == "ProfiledTask" && event.thread_name.rfind("Worker-", 0) == 0;
  }));
}

TEST(Profiler, BuildsFrameStatsForThreadLanesAndTotals) {
  ProfilerFrameSnapshot snapshot;
  snapshot.frame_index = 7;
  snapshot.duration_ms = 16.0;
  snapshot.events = {
      {7, 2, "Worker-0", "AssetFinalize", "Asset", 0, 4.0, 5.0},
      {7, 1, "MainThread", "Application::Update", "Frame", 0, 0.0, 7.0},
      {7, 1, "MainThread", "RenderSubmit", "Render", 0, 8.0, 3.0},
      {7, 1, "MainThread", "Application::Update", "Frame", 0, 12.0, 1.0},
  };

  const auto stats = BuildProfilerFrameStats(snapshot);
  EXPECT_EQ(stats.frame_index, 7);
  EXPECT_DOUBLE_EQ(stats.duration_ms, 16.0);
  EXPECT_EQ(stats.event_count, 4);
  EXPECT_DOUBLE_EQ(stats.total_event_ms, 16.0);
  EXPECT_DOUBLE_EQ(stats.max_event_ms, 7.0);

  ASSERT_EQ(stats.thread_lanes.size(), 2);
  EXPECT_EQ(stats.thread_lanes[0].thread_name, "MainThread");
  EXPECT_EQ(stats.thread_lanes[0].events.size(), 3);
  EXPECT_DOUBLE_EQ(stats.thread_lanes[0].total_ms, 11.0);
  EXPECT_EQ(stats.thread_lanes[0].events.front().name, "Application::Update");
  EXPECT_EQ(stats.thread_lanes[1].thread_name, "Worker-0");
  EXPECT_DOUBLE_EQ(stats.thread_lanes[1].total_ms, 5.0);

  const auto* frame_total = FindTotal(stats.category_totals, "Frame", "Frame");
  ASSERT_NE(frame_total, nullptr);
  EXPECT_EQ(frame_total->count, 2);
  EXPECT_DOUBLE_EQ(frame_total->total_ms, 8.0);
  EXPECT_DOUBLE_EQ(frame_total->average_ms, 4.0);
  EXPECT_DOUBLE_EQ(frame_total->max_ms, 7.0);

  const auto* update_total = FindTotal(stats.named_event_totals, "Application::Update", "Frame");
  ASSERT_NE(update_total, nullptr);
  EXPECT_EQ(update_total->count, 2);
  EXPECT_DOUBLE_EQ(update_total->total_ms, 8.0);
  EXPECT_DOUBLE_EQ(update_total->average_ms, 4.0);
  EXPECT_DOUBLE_EQ(update_total->max_ms, 7.0);
}

TEST(Profiler, BuildsAggregatedHierarchyWithInclusiveAndSelfTime) {
  ProfilerFrameSnapshot snapshot;
  snapshot.frame_index = 11;
  snapshot.capture_session_index = 4;
  snapshot.duration_ms = 12.0;
  snapshot.events = {
      {11, 1, "MainThread", "Application::Loop", "Frame", 0, 0.0, 10.0},
      {11, 1, "MainThread", "Application::PreUpdate", "Frame", 1, 0.5, 4.0},
      {11, 1, "MainThread", "Application::LayerPreUpdate", "Layer", 2, 1.0, 2.0},
      {11, 1, "MainThread", "RenderLayer", "Layer", 3, 1.1, 0.8},
      {11, 1, "MainThread", "EditorLayer", "Layer", 3, 2.0, 0.5},
      {11, 1, "MainThread", "Application::Update", "Frame", 1, 5.0, 3.0},
      {11, 1, "MainThread", "Application::LayerUpdate", "Layer", 2, 5.5, 2.0},
      {11, 1, "MainThread", "RenderLayer", "Layer", 3, 5.6, 0.6},
      {11, 1, "MainThread", "RenderLayer", "Layer", 3, 6.3, 0.4},
      {11, 2, "Worker-0", "WorkerTask", "Task", 0, 2.0, 6.0},
  };

  const auto stats = BuildProfilerFrameStats(snapshot);
  EXPECT_EQ(stats.capture_session_index, 4);
  ASSERT_EQ(stats.thread_lanes.size(), 2);
  const auto& main_lane = stats.thread_lanes.front();
  ASSERT_EQ(main_lane.thread_name, "MainThread");
  ASSERT_EQ(main_lane.hierarchy.size(), 1);
  const auto& loop = main_lane.hierarchy.front();
  EXPECT_EQ(loop.name, "Application::Loop");
  EXPECT_DOUBLE_EQ(loop.inclusive_ms, 10.0);
  EXPECT_DOUBLE_EQ(loop.self_ms, 3.0);

  const auto* pre_update = FindHierarchyNode(loop.children, "Application::PreUpdate");
  ASSERT_NE(pre_update, nullptr);
  EXPECT_DOUBLE_EQ(pre_update->self_ms, 2.0);
  const auto* pre_update_layers = FindHierarchyNode(pre_update->children, "Application::LayerPreUpdate");
  ASSERT_NE(pre_update_layers, nullptr);
  EXPECT_DOUBLE_EQ(pre_update_layers->self_ms, 0.7);

  const auto* update = FindHierarchyNode(loop.children, "Application::Update");
  ASSERT_NE(update, nullptr);
  const auto* update_layers = FindHierarchyNode(update->children, "Application::LayerUpdate");
  ASSERT_NE(update_layers, nullptr);
  const auto* render_layer = FindHierarchyNode(update_layers->children, "RenderLayer");
  ASSERT_NE(render_layer, nullptr);
  EXPECT_EQ(render_layer->count, 2);
  EXPECT_DOUBLE_EQ(render_layer->inclusive_ms, 1.0);
  EXPECT_DOUBLE_EQ(render_layer->max_ms, 0.6);

  const auto& worker_lane = stats.thread_lanes.back();
  ASSERT_EQ(worker_lane.thread_name, "Worker-0");
  ASSERT_EQ(worker_lane.hierarchy.size(), 1);
  EXPECT_EQ(worker_lane.hierarchy.front().name, "WorkerTask");
}

TEST(Profiler, PropagatesDroppedEventCountToFrameStats) {
  ProfilerFrameSnapshot snapshot;
  snapshot.dropped_event_count = 7;
  EXPECT_EQ(BuildProfilerFrameStats(snapshot).dropped_event_count, 7);
}

TEST(Profiler, ClampsMalformedHierarchySelfTime) {
  ProfilerFrameSnapshot snapshot;
  snapshot.events = {
      {1, 1, "MainThread", "Parent", "Unit", 0, 0.0, 1.0},
      {1, 1, "MainThread", "ChildA", "Unit", 1, 0.1, 0.8},
      {1, 1, "MainThread", "ChildB", "Unit", 1, 0.2, 0.8},
  };

  const auto stats = BuildProfilerFrameStats(snapshot);
  ASSERT_EQ(stats.thread_lanes.size(), 1);
  ASSERT_EQ(stats.thread_lanes.front().hierarchy.size(), 1);
  EXPECT_DOUBLE_EQ(stats.thread_lanes.front().hierarchy.front().self_ms, 0.0);
}

TEST(Profiler, PreservesProfilerPanelRowsInFirstSeenOrder) {
  struct Row {
    std::string key;
  };
  std::vector<Row> rows;
  const auto observe = [&](const std::string& key) -> Row& {
    return profiler_panel_detail::AppendFirstSeen(
        rows, key,
        [](const Row& row) -> const std::string& {
          return row.key;
        },
        Row{key});
  };

  EXPECT_EQ(&observe("Update"), &observe("Update"));
  observe("LateUpdate");
  observe("PreUpdate");
  ASSERT_EQ(rows.size(), 3);
  EXPECT_EQ(rows[0].key, "Update");
  EXPECT_EQ(rows[1].key, "LateUpdate");
  EXPECT_EQ(rows[2].key, "PreUpdate");
}

TEST(Profiler, TreatsMissingProfilerPanelSamplesAsZero) {
  const auto summary = profiler_panel_detail::SummarizeWithMissingZeros({4.0, 8.0}, 4);
  EXPECT_DOUBLE_EQ(summary.average, 3.0);
  EXPECT_DOUBLE_EQ(summary.maximum, 8.0);
  EXPECT_DOUBLE_EQ(summary.p95, 8.0);
  EXPECT_EQ(summary.observed_count, 2);

  const auto empty = profiler_panel_detail::SummarizeWithMissingZeros({}, 3);
  EXPECT_DOUBLE_EQ(empty.average, 0.0);
  EXPECT_DOUBLE_EQ(empty.p95, 0.0);
}

TEST(Profiler, UnionsOverlappingSynchronizationIntervals) {
  using profiler_panel_detail::TimingInterval;
  EXPECT_DOUBLE_EQ(
      profiler_panel_detail::IntervalUnionMilliseconds({{1.0, 4.0}, {2.0, 1.0}, {4.0, 3.0}, {9.0, 2.0}, {20.0, 0.0}}),
      8.0);
}

TEST(Profiler, BuildsOverlappingCpuGpuFrameOverview) {
  const auto sample = profiler_panel_detail::BuildFrameOverviewSample(12.0, 4.0, 14.0);
  EXPECT_DOUBLE_EQ(sample.cpu_active_ms, 8.0);
  EXPECT_DOUBLE_EQ(sample.synchronization_ms, 4.0);
  EXPECT_DOUBLE_EQ(sample.cpu_wall_ms, 12.0);
  EXPECT_DOUBLE_EQ(sample.gpu_ms, 14.0);
  EXPECT_DOUBLE_EQ(sample.total_ms, 14.0);
  EXPECT_TRUE(sample.gpu_available);

  const auto missing_gpu = profiler_panel_detail::BuildFrameOverviewSample(10.0, 20.0, std::nullopt);
  EXPECT_DOUBLE_EQ(missing_gpu.cpu_active_ms, 0.0);
  EXPECT_DOUBLE_EQ(missing_gpu.synchronization_ms, 10.0);
  EXPECT_DOUBLE_EQ(missing_gpu.total_ms, 10.0);
  EXPECT_FALSE(missing_gpu.gpu_available);
}

TEST(Profiler, ClassifiesLogicalCpuExecutorsAndBuildsStableKeys) {
  using profiler_panel_detail::ClassifyCpuExecutor;
  using profiler_panel_detail::CpuExecutorGroup;
  EXPECT_EQ(ClassifyCpuExecutor("MainThread"), CpuExecutorGroup::MainThread);
  EXPECT_EQ(ClassifyCpuExecutor("Worker-7"), CpuExecutorGroup::Worker);
  EXPECT_EQ(ClassifyCpuExecutor("AssetIo-0"), CpuExecutorGroup::AssetIo);
  EXPECT_EQ(ClassifyCpuExecutor("Gpu-0"), CpuExecutorGroup::GpuSubmission);
  EXPECT_EQ(ClassifyCpuExecutor("Render-1"), CpuExecutorGroup::Render);
  EXPECT_EQ(ClassifyCpuExecutor("Background-0"), CpuExecutorGroup::Background);
  EXPECT_EQ(ClassifyCpuExecutor("PluginThread"), CpuExecutorGroup::Other);
  const auto first = profiler_panel_detail::StableHierarchyKey(CpuExecutorGroup::Worker, "Root", "Task", "Update");
  EXPECT_EQ(first, profiler_panel_detail::StableHierarchyKey(CpuExecutorGroup::Worker, "Root", "Task", "Update"));
  EXPECT_NE(first, profiler_panel_detail::StableHierarchyKey(CpuExecutorGroup::Worker, "Other", "Task", "Update"));
}

TEST(Profiler, BuildsStableHistoryWithMissingZerosAndSeparateWorkerWork) {
  ProfilerFrameSnapshot first;
  first.frame_index = 1;
  first.duration_ms = 10.0;
  first.events = {
      {1, 1, "MainThread", "Application::Loop", "Frame", 0, 0.0, 8.0},
      {1, 1, "MainThread", "Application::Update", "Frame", 1, 1.0, 3.0},
      {1, 1, "MainThread", "Application::Update", "Frame", 1, 4.0, 1.0},
      {1, 2, "Worker-0", "Visibility", "Task", 0, 2.0, 6.0},
  };
  first.counters = {{"Camera Count", "Raster", "cameras", 2.0}};
  ProfilerFrameSnapshot second;
  second.frame_index = 2;
  second.duration_ms = 14.0;
  second.events = {
      {2, 1, "MainThread", "Application::Loop", "Frame", 0, 0.0, 12.0},
      {2, 1, "MainThread", "Application::LateUpdate", "Frame", 1, 5.0, 4.0},
  };

  const auto history = BuildProfilerHistoryStats(BuildProfilerFrameStatsHistory({first, second}));
  EXPECT_EQ(history.frame_count, 2);
  EXPECT_DOUBLE_EQ(history.frame_duration.average_ms, 12.0);
  EXPECT_DOUBLE_EQ(history.frame_duration.median_ms, 10.0);
  EXPECT_DOUBLE_EQ(history.frame_duration.p95_ms, 14.0);
  EXPECT_DOUBLE_EQ(history.frame_duration.selected_ms, 14.0);
  EXPECT_DOUBLE_EQ(history.main_thread_wall.average_ms, 10.0);
  EXPECT_DOUBLE_EQ(history.worker_cpu_work.average_ms, 3.0);
  ASSERT_EQ(history.threads.size(), 2);
  ASSERT_EQ(history.threads[0].hierarchy.size(), 1);
  const auto& loop = history.threads[0].hierarchy[0];
  ASSERT_EQ(loop.children.size(), 2);
  EXPECT_EQ(loop.children[0].name, "Application::Update");
  EXPECT_EQ(loop.children[1].name, "Application::LateUpdate");
  EXPECT_EQ(loop.children[0].inclusive.observed_frame_count, 1);
  EXPECT_DOUBLE_EQ(loop.children[0].inclusive.average_ms, 2.0);
  EXPECT_DOUBLE_EQ(loop.children[0].inclusive.maximum_ms, 4.0);
  ASSERT_EQ(history.counters.size(), 1);
  EXPECT_EQ(history.counters[0].observed_frame_count, 1);
  EXPECT_DOUBLE_EQ(history.counters[0].average, 1.0);
  EXPECT_DOUBLE_EQ(history.counters[0].selected, 0.0);
}

TEST(Profiler, ExportsChromeTraceJson) {
  ProfilerFrameSnapshot snapshot;
  snapshot.frame_index = 3;
  snapshot.duration_ms = 12.0;
  snapshot.events = {
      {3, 7, "MainThread", "Scene \"Update\"", "Scene", 0, 1.0, 2.5},
      {3, 8, "Worker-0", "Asset\nFinalize", "Asset Finalize", 1, 4.0, 3.0},
  };

  TempProfilerDirectory temp;
  const auto trace_path = temp.RootPath() / "trace.json";
  std::string error;
  ASSERT_TRUE(ExportProfilerChromeTrace(trace_path, {snapshot}, &error)) << error;

  std::ifstream stream(trace_path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  const auto json = buffer.str();
  EXPECT_NE(json.find("\"traceEvents\""), std::string::npos);
  EXPECT_NE(json.find("\"ph\":\"X\""), std::string::npos);
  EXPECT_NE(json.find("Scene \\\"Update\\\""), std::string::npos);
  EXPECT_NE(json.find("Asset\\nFinalize"), std::string::npos);
  EXPECT_NE(json.find("\"tid\":7"), std::string::npos);
  EXPECT_NE(json.find("\"frame\":3"), std::string::npos);
}
