#pragma once

#include <functional>
#include <memory>
#include <string>
#include "EvoEngineAPI.hpp"

namespace evo_engine {

struct ShaderPreparationProgress {
  size_t completed = 0;
  size_t total = 0;
  std::string current;
  std::string error;
};

class EVOENGINE_API ShaderPreparationQueue final {
 public:
  explicit ShaderPreparationQueue(size_t worker_count = 0);
  ~ShaderPreparationQueue();
  void BeginRequests();
  void SetOptional(bool optional);
  void SetSynchronous(bool synchronous);
  void Require(const std::string& key);
  [[nodiscard]] bool IsDone(const std::string& key) const;
  void ReleaseResult(const std::string& key);
  template <typename T>
  std::shared_ptr<T> GetResult(const std::string& key) const {
    return std::static_pointer_cast<T>(GetResultImpl(key));
  }
  template <typename T>
  std::shared_ptr<T> Request(const std::string& key, const std::function<std::shared_ptr<T>()>& factory) {
    return std::static_pointer_cast<T>(RequestImpl(key, [factory] {
      return factory();
    }));
  }
  void Tick();
  [[nodiscard]] bool RequiredReady() const;
  [[nodiscard]] ShaderPreparationProgress GetProgress() const;
  void Wait();
  void Clear();

 private:
  struct State;
  std::unique_ptr<State> state_;
  std::shared_ptr<void> RequestImpl(const std::string& key, std::function<std::shared_ptr<void>()> factory);
  std::shared_ptr<void> GetResultImpl(const std::string& key) const;
};
}  // namespace evo_engine
