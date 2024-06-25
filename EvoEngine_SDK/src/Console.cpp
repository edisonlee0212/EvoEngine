#include "Console.hpp"
#include "EditorLayer.hpp"
#include "Times.hpp"
using namespace evo_engine;

void Console::Log(const std::string& msg) {
  const auto editor_layer = Application::GetLayer<EditorLayer>();
  if (!editor_layer)
    return;
  std::lock_guard lock(editor_layer->console_message_mutex_);
  ConsoleMessage cm;
  cm.m_value = msg;
  cm.m_type = ConsoleMessageType::Log;
  cm.m_time = Times::Now();
  editor_layer->console_messages_.push_back(cm);
}
void Console::Error(const std::string& msg) {
  const auto editor_layer = Application::GetLayer<EditorLayer>();
  if (!editor_layer)
    return;
  std::lock_guard lock(editor_layer->console_message_mutex_);
  ConsoleMessage cm;
  cm.m_value = msg;
  cm.m_type = ConsoleMessageType::Error;
  cm.m_time = Times::Now();
  editor_layer->console_messages_.push_back(cm);
}

void Console::Warning(const std::string& msg) {
  const auto editor_layer = Application::GetLayer<EditorLayer>();
  if (!editor_layer)
    return;
  std::lock_guard lock(editor_layer->console_message_mutex_);
  ConsoleMessage cm;
  cm.m_value = msg;
  cm.m_type = ConsoleMessageType::Warning;
  cm.m_time = Times::Now();
  editor_layer->console_messages_.push_back(cm);
}
