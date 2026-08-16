#include "Console.hpp"
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "Times.hpp"

#include <ctime>
#include <mutex>
#include <vector>

using namespace evo_engine;

namespace {
constexpr size_t kMaxConsoleMessages = 10000;

struct PendingConsoleMessage {
  ConsoleMessageType type = ConsoleMessageType::Log;
  std::string value;
  double time = 0.0;
  std::time_t timestamp = 0;
};

std::mutex& PendingConsoleMessageMutex() {
  static std::mutex mutex;
  return mutex;
}

std::vector<PendingConsoleMessage>& PendingConsoleMessages() {
  static std::vector<PendingConsoleMessage> messages;
  return messages;
}

bool StartsWith(const std::string& value, const std::string& prefix) {
  return value.rfind(prefix, 0) == 0;
}

bool ShouldSkipRedirectedLine(const std::string& line) {
  return line.empty() || line == "==========" || StartsWith(line, "[EvoEngine]Log:") ||
         StartsWith(line, "[EvoEngine]Error:") || StartsWith(line, "[EvoEngine]Warning:");
}

class ConsoleStreamBuffer final : public std::streambuf {
  std::streambuf* original_ = nullptr;
  ConsoleMessageType type_ = ConsoleMessageType::Log;
  std::string line_;

 public:
  ConsoleStreamBuffer(std::streambuf* original, const ConsoleMessageType type) : original_(original), type_(type) {
  }

  ~ConsoleStreamBuffer() override {
    FlushLine();
  }

 protected:
  int overflow(const int character) override {
    if (character == traits_type::eof()) {
      return traits_type::not_eof(character);
    }
    if (original_) {
      original_->sputc(static_cast<char>(character));
    }
    if (character == '\n') {
      FlushLine();
    } else if (character != '\r') {
      line_.push_back(static_cast<char>(character));
    }
    return character;
  }

  int sync() override {
    if (original_) {
      original_->pubsync();
    }
    FlushLine();
    return 0;
  }

 private:
  void FlushLine() {
    if (!line_.empty()) {
      switch (type_) {
        case ConsoleMessageType::Error:
          Console::Error(line_);
          break;
        case ConsoleMessageType::Warning:
          Console::Warning(line_);
          break;
        default:
          Console::Log(line_);
          break;
      }
      line_.clear();
    }
  }
};
}  // namespace

struct Console::StreamRedirectState {
  std::streambuf* cout_buffer = nullptr;
  std::streambuf* cerr_buffer = nullptr;
  std::streambuf* clog_buffer = nullptr;
  std::unique_ptr<ConsoleStreamBuffer> cout_redirect;
  std::unique_ptr<ConsoleStreamBuffer> cerr_redirect;
  std::unique_ptr<ConsoleStreamBuffer> clog_redirect;
};

Console::Console() = default;

Console::~Console() {
  RestoreStandardStreamRedirectors();
}

void Console::InstallStandardStreamRedirectors() {
  if (stream_redirect_state_) {
    return;
  }

  auto state = std::make_unique<StreamRedirectState>();
  state->cout_buffer = std::cout.rdbuf();
  state->cerr_buffer = std::cerr.rdbuf();
  state->clog_buffer = std::clog.rdbuf();
  state->cout_redirect = std::make_unique<ConsoleStreamBuffer>(state->cout_buffer, ConsoleMessageType::Log);
  state->cerr_redirect = std::make_unique<ConsoleStreamBuffer>(state->cerr_buffer, ConsoleMessageType::Error);
  state->clog_redirect = std::make_unique<ConsoleStreamBuffer>(state->clog_buffer, ConsoleMessageType::Log);
  std::cout.rdbuf(state->cout_redirect.get());
  std::cerr.rdbuf(state->cerr_redirect.get());
  std::clog.rdbuf(state->clog_redirect.get());
  stream_redirect_state_ = std::move(state);
}

void Console::RestoreStandardStreamRedirectors() {
  if (!stream_redirect_state_) {
    return;
  }

  std::cout.rdbuf(stream_redirect_state_->cout_buffer);
  std::cerr.rdbuf(stream_redirect_state_->cerr_buffer);
  std::clog.rdbuf(stream_redirect_state_->clog_buffer);
  stream_redirect_state_.reset();
}

void Console::AppendMessageToEditor(const std::shared_ptr<EditorLayer>& editor_layer, const ConsoleMessageType type,
                                    const std::string& msg, const double time, const std::time_t timestamp) {
  std::lock_guard lock(editor_layer->console_message_mutex_);
  if (editor_layer->console_messages_.size() >= kMaxConsoleMessages) {
    editor_layer->console_messages_.erase(editor_layer->console_messages_.begin());
  }
  ConsoleMessage cm;
  cm.m_value = msg;
  cm.m_type = type;
  cm.m_time = time;
  cm.m_timestamp = timestamp;
  editor_layer->console_messages_.push_back(cm);
  ++editor_layer->console_message_revision_;
}

void Console::PushMessage(const ConsoleMessageType type, const std::string& msg) {
  if (ShouldSkipRedirectedLine(msg)) {
    return;
  }

  const auto application = ApplicationContext::TryGet();
  const double time = application ? application->GetTimes().Now() : 0.0;
  const std::time_t timestamp = std::time(nullptr);
  const auto editor_layer = application ? application->GetLayer<EditorLayer>() : nullptr;
  std::lock_guard pending_lock(PendingConsoleMessageMutex());
  if (!editor_layer) {
    PendingConsoleMessages().push_back({type, msg, time, timestamp});
    return;
  }

  for (const auto& pending_message : PendingConsoleMessages()) {
    AppendMessageToEditor(editor_layer, pending_message.type, pending_message.value, pending_message.time,
                          pending_message.timestamp);
  }
  PendingConsoleMessages().clear();
  AppendMessageToEditor(editor_layer, type, msg, time, timestamp);
}

void Console::Log(const std::string& msg) {
  PushMessage(ConsoleMessageType::Log, msg);
}

void Console::Error(const std::string& msg) {
  PushMessage(ConsoleMessageType::Error, msg);
}

void Console::Warning(const std::string& msg) {
  PushMessage(ConsoleMessageType::Warning, msg);
}
