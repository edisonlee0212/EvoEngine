#pragma once
#include <Windows.h>
#include <atomic>
#include <filesystem>
#include <future>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>

namespace evo_engine {
struct RuntimeConfiguration;
}
namespace Gdiplus {
class Image;
}

class RuntimeStartupScreen {
 public:
  explicit RuntimeStartupScreen(const evo_engine::RuntimeConfiguration& config, std::filesystem::path capture = {});
  ~RuntimeStartupScreen();
  void SetStatus(std::string_view status);
  void Close();
  [[nodiscard]] bool Cancelled() const;

 private:
  void Run(const evo_engine::RuntimeConfiguration& config, std::promise<void>& ready);
  void Paint(HWND window);
  static LRESULT CALLBACK WindowProc(HWND window, UINT message, WPARAM w_param, LPARAM l_param);
  std::atomic<HWND> window_ = nullptr;
  std::atomic_bool cancelled_ = false;
  std::mutex mutex_;
  std::wstring status_ = L"Starting...";
  Gdiplus::Image* logo_ = nullptr;
  std::filesystem::path capture_;
  std::thread thread_;
};
