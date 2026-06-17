#include "Profiler.hpp"
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
    const ProfilerFrameScope frame;
    const ProfilerScope scope("UnitScope", "Unit");
  }

  const auto snapshot = profiler.GetLatestFrameSnapshot();
  ASSERT_EQ(snapshot.frame_index, 1);
  ASSERT_EQ(snapshot.events.size(), 1);
  EXPECT_EQ(snapshot.events.front().name, "UnitScope");
  EXPECT_EQ(snapshot.events.front().category, "Unit");
  EXPECT_EQ(snapshot.events.front().thread_name, "TestMain");
  EXPECT_GE(snapshot.events.front().duration_ms, 0.0);
  EXPECT_GE(snapshot.events.front().start_ms, 0.0);
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
