#include "ShaderPreparationQueue.hpp"
#include "Shader.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <iterator>
#include <vector>

using namespace evo_engine;

struct ShaderPreparationQueue::State {
  struct Entry {
    std::string key;
    std::function<std::shared_ptr<void>()> factory;
    std::shared_ptr<void> result;
    std::string error;
    bool required = false;
    bool done = false;
  };
  std::vector<Entry> entries;
  struct Active {
    size_t index;
    std::future<std::shared_ptr<void>> future;
  };
  std::vector<Active> active;
  size_t worker_count = 2;
  bool optional = false;
  bool synchronous = false;

  void Finish(Entry& entry, std::shared_ptr<void> result) {
    entry.result = std::move(result);
    if (!entry.result && entry.error.empty())
      entry.error = "Pipeline initialization failed";
    entry.done = true;
    entry.factory = {};
  }
  void Poll() {
    for (auto it = active.begin(); it != active.end();) {
      if (it->future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        ++it;
        continue;
      }
      auto& entry = entries[it->index];
      try {
        Finish(entry, it->future.get());
      } catch (const std::exception& error) {
        entry.error = error.what();
        Finish(entry, {});
      }
      it = active.erase(it);
    }
  }
};

ShaderPreparationQueue::ShaderPreparationQueue(size_t worker_count) : state_(std::make_unique<State>()) {
  state_->worker_count =
      worker_count ? std::clamp(worker_count, size_t{1}, size_t{8}) : Shader::GetPreparationWorkerCount();
}
ShaderPreparationQueue::~ShaderPreparationQueue() {
  Wait();
}

void ShaderPreparationQueue::BeginRequests() {
  for (auto& entry : state_->entries)
    entry.required = false;
  state_->Poll();
}
void ShaderPreparationQueue::SetOptional(bool optional) {
  state_->optional = optional;
}
void ShaderPreparationQueue::SetSynchronous(bool synchronous) {
  state_->synchronous = synchronous;
}
void ShaderPreparationQueue::Require(const std::string& key) {
  for (auto& entry : state_->entries)
    if (entry.key == key)
      entry.required |= !state_->optional;
}
bool ShaderPreparationQueue::IsDone(const std::string& key) const {
  for (const auto& entry : state_->entries)
    if (entry.key == key)
      return entry.done;
  return false;
}
std::shared_ptr<void> ShaderPreparationQueue::GetResultImpl(const std::string& key) const {
  for (const auto& entry : state_->entries)
    if (entry.key == key)
      return entry.result;
  return {};
}
void ShaderPreparationQueue::ReleaseResult(const std::string& key) {
  for (auto& entry : state_->entries)
    if (entry.key == key)
      entry.result.reset();
}
std::shared_ptr<void> ShaderPreparationQueue::RequestImpl(const std::string& key,
                                                          std::function<std::shared_ptr<void>()> factory) {
  auto& entries = state_->entries;
  auto found = std::find_if(entries.begin(), entries.end(), [&](const auto& entry) {
    return entry.key == key;
  });
  if (found == entries.end()) {
    entries.push_back({key, std::move(factory)});
    found = std::prev(entries.end());
  } else if (found->done && !found->result && found->error.empty()) {
    found->done = false;
    found->factory = std::move(factory);
  }
  found->required |= !state_->optional;
  if (state_->synchronous && !found->done)
    state_->Finish(*found, found->factory());
  return found->result;
}
void ShaderPreparationQueue::Tick() {
  state_->Poll();
  for (bool required : {true, false}) {
    for (size_t i = 0; i < state_->entries.size(); ++i) {
      auto& entry = state_->entries[i];
      if (state_->active.size() == state_->worker_count)
        return;
      if (entry.done || entry.required != required ||
          std::any_of(state_->active.begin(), state_->active.end(), [i](const auto& job) {
            return job.index == i;
          }))
        continue;
      state_->active.push_back({i, std::async(std::launch::async, entry.factory)});
    }
    if (required && std::any_of(state_->entries.begin(), state_->entries.end(), [](const auto& entry) {
          return entry.required && !entry.done;
        }))
      return;
  }
}
bool ShaderPreparationQueue::RequiredReady() const {
  for (const auto& entry : state_->entries)
    if (entry.required && (!entry.done || !entry.error.empty()))
      return false;
  return true;
}
ShaderPreparationProgress ShaderPreparationQueue::GetProgress() const {
  ShaderPreparationProgress result;
  result.total = state_->entries.size();
  for (const auto& entry : state_->entries) {
    result.completed += entry.done;
    if (!entry.error.empty())
      result.error = entry.key + ": " + entry.error;
  }
  for (const auto& job : state_->active) {
    if (!result.current.empty())
      result.current += " | ";
    result.current += state_->entries[job.index].key;
  }
  return result;
}
void ShaderPreparationQueue::Wait() {
  for (auto& job : state_->active)
    job.future.wait();
  state_->Poll();
}
void ShaderPreparationQueue::Clear() {
  Wait();
  state_->entries.clear();
}
