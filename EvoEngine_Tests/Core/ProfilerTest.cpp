#include "Profiler.hpp"
#include "TaskRuntime.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <string>

using namespace evo_engine;

namespace {
bool HasEvent(const ProfilerFrameSnapshot& snapshot, const std::string& name, const std::string& category) {
  return std::any_of(snapshot.events.begin(), snapshot.events.end(), [&](const ProfilerScopeEvent& event) {
    return event.name == name && event.category == category;
  });
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
