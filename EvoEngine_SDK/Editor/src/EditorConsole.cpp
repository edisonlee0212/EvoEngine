#include "EditorLayer.hpp"

using namespace evo_engine;

void EditorLayer::PollConsoleMessages() {
  auto messages = Console::DrainPendingMessages();
  if (messages.empty())
    return;
  constexpr size_t kMaxConsoleMessages = 10000;
  std::lock_guard lock(console_message_mutex_);
  const auto overflow = console_messages_.size() + messages.size();
  if (overflow > kMaxConsoleMessages) {
    console_messages_.erase(console_messages_.begin(), console_messages_.begin() + (overflow - kMaxConsoleMessages));
  }
  console_messages_.insert(console_messages_.end(), std::make_move_iterator(messages.begin()),
                           std::make_move_iterator(messages.end()));
  ++console_message_revision_;
}
