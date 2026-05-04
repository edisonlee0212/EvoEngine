#include "ApplicationContext.hpp"

#include <stdexcept>

using namespace evo_engine;

namespace {
thread_local Application* g_current_application = nullptr;
}

void ApplicationContext::Set(Application* application) {
  g_current_application = application;
}

Application* ApplicationContext::TryGet() {
  return g_current_application;
}

Application& ApplicationContext::Get() {
  if (!g_current_application) {
    throw std::runtime_error("No EvoEngine Application is active on this thread.");
  }
  return *g_current_application;
}

ApplicationContextScope::ApplicationContextScope(Application& application) {
  previous_application_ = ApplicationContext::TryGet();
  ApplicationContext::Set(&application);
}

ApplicationContextScope::~ApplicationContextScope() {
  ApplicationContext::Set(previous_application_);
}
