#pragma once
#include <Windows.h>
#include <atomic>
#include <filesystem>
#include <functional>
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
  struct Progress {
    size_t scene_completed = 0;
    size_t scene_total = 1;
    size_t preparation_completed = 0;
    size_t preparation_total = 1;
    bool operator!=(const Progress& other) const {
      return scene_completed != other.scene_completed || scene_total != other.scene_total ||
             preparation_completed != other.preparation_completed || preparation_total != other.preparation_total;
    }
  };
  explicit RuntimeStartupScreen(const evo_engine::RuntimeConfiguration& config, std::filesystem::path capture = {});
  ~RuntimeStartupScreen();
  void SetStatus(std::string_view status);
  void SetProgressSource(std::function<Progress()> source);
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
  std::function<Progress()> progress_source_;
  Progress last_progress_;
  Gdiplus::Image* logo_ = nullptr;
  std::filesystem::path capture_;
  std::thread thread_;
};
