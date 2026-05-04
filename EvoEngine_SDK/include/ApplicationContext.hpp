#pragma once

namespace evo_engine {
class Application;

class ApplicationContext final {
 public:
  static void Set(Application* application);
  static Application* TryGet();
  static Application& Get();
};

class ApplicationContextScope final {
  Application* previous_application_ = nullptr;

 public:
  explicit ApplicationContextScope(Application& application);
  ~ApplicationContextScope();
  ApplicationContextScope(const ApplicationContextScope&) = delete;
  ApplicationContextScope& operator=(const ApplicationContextScope&) = delete;
};
}  // namespace evo_engine
