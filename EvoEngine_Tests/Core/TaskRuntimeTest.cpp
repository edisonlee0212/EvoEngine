#include "TaskRuntime.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace evo_engine;

namespace {
TaskRuntimeSettings TestRuntimeSettings() {
  TaskRuntimeSettings settings;
  settings.worker_thread_size = 2;
  settings.asset_io_thread_size = 1;
  settings.gpu_thread_size = 1;
  settings.render_thread_size = 1;
  settings.background_thread_size = 1;
  return settings;
}
}  // namespace

TEST(TaskRuntime, ExecutesDependencyGraphInOrder) {
  TaskRuntime runtime;
  runtime.Initialize(TestRuntimeSettings());

  std::mutex order_mutex;
  std::vector<int> order;
  const auto dependency = runtime.Schedule([&]() {
    std::lock_guard lock(order_mutex);
    order.emplace_back(1);
  });
  const auto dependent = runtime.Schedule({dependency}, [&]() {
    std::lock_guard lock(order_mutex);
    order.emplace_back(2);
  });

  runtime.Wait(dependent);
  ASSERT_EQ(order.size(), 2);
  EXPECT_EQ(order[0], 1);
  EXPECT_EQ(order[1], 2);
}

TEST(TaskRuntime, RunsMainThreadTasksOnWaitingMainThread) {
  TaskRuntime runtime;
  runtime.Initialize(TestRuntimeSettings());

  const auto main_thread_id = std::this_thread::get_id();
  std::thread::id executed_thread_id;
  TaskOptions options;
  options.executor = TaskExecutorType::MainThread;
  options.affinity = ThreadAffinity::MainThread;

  const auto task = runtime.Schedule({}, options, [&]() {
    executed_thread_id = std::this_thread::get_id();
  });

  runtime.Wait(task);
  EXPECT_EQ(executed_thread_id, main_thread_id);
}

TEST(TaskRuntime, RunsNamedServiceExecutors) {
  TaskRuntime runtime;
  runtime.Initialize(TestRuntimeSettings());

  std::atomic_bool asset_io_executed = false;
  std::atomic_bool gpu_executed = false;
  std::atomic_bool gpu_is_not_render = false;
  std::atomic_bool render_executed = false;
  std::atomic_bool background_executed = false;

  TaskOptions asset_options;
  asset_options.executor = TaskExecutorType::AssetIo;
  const auto asset_task = runtime.Schedule({}, asset_options, [&]() {
    asset_io_executed = runtime.IsExecutorThread(TaskExecutorType::AssetIo);
  });

  TaskOptions gpu_options;
  gpu_options.executor = TaskExecutorType::Gpu;
  const auto gpu_task = runtime.Schedule({}, gpu_options, [&]() {
    gpu_executed = runtime.IsExecutorThread(TaskExecutorType::Gpu);
    gpu_is_not_render = !runtime.IsExecutorThread(TaskExecutorType::Render);
  });

  TaskOptions render_options;
  render_options.executor = TaskExecutorType::Render;
  const auto render_task = runtime.Schedule({}, render_options, [&]() {
    render_executed = runtime.IsExecutorThread(TaskExecutorType::Render);
  });

  TaskOptions background_options;
  background_options.executor = TaskExecutorType::Background;
  const auto background_task = runtime.Schedule({}, background_options, [&]() {
    background_executed = runtime.IsExecutorThread(TaskExecutorType::Background);
  });

  runtime.Wait(asset_task);
  runtime.Wait(gpu_task);
  runtime.Wait(render_task);
  runtime.Wait(background_task);

  EXPECT_TRUE(asset_io_executed);
  EXPECT_TRUE(gpu_executed);
  EXPECT_TRUE(gpu_is_not_render);
  EXPECT_TRUE(render_executed);
  EXPECT_TRUE(background_executed);

  const auto stats = runtime.GetStats();
  EXPECT_EQ(stats.gpu_thread_size, 1);
  EXPECT_EQ(stats.render_thread_size, 1);
}

TEST(TaskRuntime, PropagatesTaskExceptionsOnWait) {
  TaskRuntime runtime;
  runtime.Initialize(TestRuntimeSettings());

  const auto task = runtime.Schedule([]() {
    throw std::runtime_error("task failure");
  });

  EXPECT_THROW(runtime.Wait(task), std::runtime_error);
  const auto stats = runtime.GetStats();
  EXPECT_EQ(stats.failed_task_size, 1);
}
