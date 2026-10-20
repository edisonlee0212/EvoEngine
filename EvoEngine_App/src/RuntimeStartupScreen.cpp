#include "RuntimeStartupScreen.hpp"
#include <gdiplus.h>
#include <algorithm>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
#include "RuntimeConfiguration.hpp"
#include "RuntimePaths.hpp"
#include "WindowModePolicy.hpp"

namespace {
std::wstring Wide(const std::string_view value) {
  const int size = MultiByteToWideChar(CP_UTF8, 0, value.data(), int(value.size()), nullptr, 0);
  std::wstring result(size, L'\0');
  MultiByteToWideChar(CP_UTF8, 0, value.data(), int(value.size()), result.data(), size);
  return result;
}
constexpr UINT kClose = WM_APP + 1;
}  // namespace

RuntimeStartupScreen::RuntimeStartupScreen(const evo_engine::RuntimeConfiguration& config,
                                           std::filesystem::path capture)
    : capture_(std::move(capture)) {
  std::promise<void> ready;
  auto result = ready.get_future();
  thread_ = std::thread([this, config, ready = std::move(ready)]() mutable {
    try {
      Run(config, ready);
    } catch (...) {
      ready.set_exception(std::current_exception());
    }
  });
  try {
    result.get();
  } catch (...) {
    thread_.join();
    throw;
  }
}

RuntimeStartupScreen::~RuntimeStartupScreen() {
  Close();
}

void RuntimeStartupScreen::Close() {
  if (const auto window = window_.load())
    PostMessageW(window, kClose, 0, 0);
  if (thread_.joinable())
    thread_.join();
}

bool RuntimeStartupScreen::Cancelled() const {
  return cancelled_;
}

void RuntimeStartupScreen::SetProgressSource(std::function<Progress()> source) {
  const std::lock_guard lock(mutex_);
  progress_source_ = std::move(source);
}

void RuntimeStartupScreen::SetStatus(const std::string_view status) {
  const auto text = Wide(status);
  const std::lock_guard lock(mutex_);
  if (status_ != text) {
    status_ = text;
    std::cout << "RUNTIME_STARTUP_STATUS " + std::string(status) + "\n";
    if (const auto window = window_.load())
      InvalidateRect(window, nullptr, FALSE);
  }
}

void RuntimeStartupScreen::Run(const evo_engine::RuntimeConfiguration& config, std::promise<void>& ready) {
  Gdiplus::GdiplusStartupInput input;
  ULONG_PTR token;
  if (Gdiplus::GdiplusStartup(&token, &input, nullptr) != Gdiplus::Ok)
    throw std::runtime_error("Could not initialize the runtime startup screen.");
  struct Shutdown {
    ULONG_PTR token;
    ~Shutdown() {
      Gdiplus::GdiplusShutdown(token);
    }
  } shutdown{token};
  const std::unique_ptr<Gdiplus::Image> logo(
      Gdiplus::Image::FromFile(evo_engine::runtime_paths::Resolve(config.startup_logo).c_str()));
  if (!logo || logo->GetLastStatus() != Gdiplus::Ok)
    throw std::runtime_error("Could not load the runtime startup logo.");
  logo_ = logo.get();
  const auto instance = GetModuleHandleW(nullptr);
  WNDCLASSW window_class{};
  window_class.lpfnWndProc = WindowProc;
  window_class.hInstance = instance;
  window_class.hIcon = LoadIconW(instance, L"IDI_ICON1");
  window_class.hCursor = LoadCursor(nullptr, IDC_ARROW);
  window_class.hbrBackground = static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH));
  window_class.lpszClassName = L"EvoEngineRuntimeStartup";
  if (!RegisterClassW(&window_class))
    throw std::runtime_error("Could not register the runtime startup window.");
  const bool fullscreen = evo_engine::WindowModePolicy::IsFullscreen(config.window_mode);
  const DWORD style = config.window_mode == evo_engine::WindowDisplayMode::Windowed
                          ? WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX
                          : WS_POPUP;
  MONITORINFO monitor{sizeof(monitor)};
  GetMonitorInfoW(MonitorFromPoint({0, 0}, MONITOR_DEFAULTTOPRIMARY), &monitor);
  RECT bounds{0, 0, config.window_size.x, config.window_size.y};
  AdjustWindowRect(&bounds, style, FALSE);
  int width = bounds.right - bounds.left, height = bounds.bottom - bounds.top;
  const auto& area = fullscreen ? monitor.rcMonitor : monitor.rcWork;
  if (fullscreen) {
    width = area.right - area.left;
    height = area.bottom - area.top;
  }
  const auto title = Wide(config.application_name);
  const auto window = CreateWindowExW(
      0, window_class.lpszClassName, title.c_str(), style, area.left + (area.right - area.left - width) / 2,
      area.top + (area.bottom - area.top - height) / 2, width, height, nullptr, nullptr, instance, this);
  if (!window) {
    UnregisterClassW(window_class.lpszClassName, instance);
    throw std::runtime_error("Could not create the runtime startup window.");
  }
  window_ = window;
  SetTimer(window, 1, 33, nullptr);
  ShowWindow(window, SW_SHOW);
  UpdateWindow(window);
  std::cout << "RUNTIME_STARTUP_VISIBLE" << std::endl;
  ready.set_value();
  MSG message;
  while (GetMessageW(&message, nullptr, 0, 0) > 0) {
    TranslateMessage(&message);
    DispatchMessageW(&message);
  }
  window_ = nullptr;
  logo_ = nullptr;
  UnregisterClassW(window_class.lpszClassName, instance);
}

LRESULT CALLBACK RuntimeStartupScreen::WindowProc(HWND window, UINT message, WPARAM w_param, LPARAM l_param) {
  auto* screen = reinterpret_cast<RuntimeStartupScreen*>(GetWindowLongPtrW(window, GWLP_USERDATA));
  if (message == WM_NCCREATE) {
    screen = static_cast<RuntimeStartupScreen*>(reinterpret_cast<CREATESTRUCTW*>(l_param)->lpCreateParams);
    SetWindowLongPtrW(window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(screen));
  }
  if (screen) {
    switch (message) {
      case WM_TIMER:
        InvalidateRect(window, nullptr, FALSE);
        return 0;
      case WM_PAINT:
        screen->Paint(window);
        return 0;
      case WM_ERASEBKGND:
        return 1;
      case WM_CLOSE:
        screen->cancelled_ = true;
        [[fallthrough]];
      case kClose:
        InvalidateRect(window, nullptr, FALSE);
        UpdateWindow(window);
        DestroyWindow(window);
        return 0;
      case WM_DESTROY:
        screen->window_ = nullptr;
        PostQuitMessage(0);
        return 0;
    }
  }
  return DefWindowProcW(window, message, w_param, l_param);
}

void RuntimeStartupScreen::Paint(HWND window) {
  PAINTSTRUCT paint;
  const auto dc = BeginPaint(window, &paint);
  RECT rect;
  GetClientRect(window, &rect);
  const float width = float(rect.right), height = float(rect.bottom);
  Gdiplus::Bitmap frame(rect.right, rect.bottom, PixelFormat32bppARGB);
  Gdiplus::Graphics graphics(&frame);
  graphics.Clear(Gdiplus::Color::Black);
  graphics.SetInterpolationMode(Gdiplus::InterpolationModeHighQualityBicubic);
  const float dpi = float(GetDpiForWindow(window)) / 96;
  const float side = std::min({width * 0.15f, height * 0.175f, 128 * dpi});
  const float scale = side / float(std::max(logo_->GetWidth(), logo_->GetHeight()));
  const float logo_width = logo_->GetWidth() * scale, logo_height = logo_->GetHeight() * scale;
  const float top = std::max(8 * dpi, std::min((height - logo_height) * 0.5f, height - logo_height - 156 * dpi));
  graphics.DrawImage(logo_, (width - logo_width) * 0.5f, top, logo_width, logo_height);
  Gdiplus::Font font(L"Segoe UI", 16 * dpi, Gdiplus::FontStyleRegular, Gdiplus::UnitPixel);
  Gdiplus::SolidBrush brush(Gdiplus::Color(220, 220, 220));
  Gdiplus::StringFormat format;
  format.SetAlignment(Gdiplus::StringAlignmentCenter);
  format.SetTrimming(Gdiplus::StringTrimmingEllipsisCharacter);
  format.SetFormatFlags(Gdiplus::StringFormatFlagsNoWrap);
  const Gdiplus::RectF text_bounds(16 * dpi, top + logo_height + 28 * dpi, width - 32 * dpi, 28 * dpi);
  std::wstring status;
  std::function<Progress()> progress_source;
  {
    const std::lock_guard lock(mutex_);
    status = status_;
    progress_source = progress_source_;
  }
  graphics.DrawString(status.c_str(), -1, &font, text_bounds, &format, &brush);
  const auto progress = progress_source ? progress_source() : Progress{};
  if (progress != last_progress_) {
    std::cout << "RUNTIME_STARTUP_PROGRESS scene=" + std::to_string(progress.scene_completed) + "/" +
                     std::to_string(progress.scene_total) +
                     " preparations=" + std::to_string(progress.preparation_completed) + "/" +
                     std::to_string(progress.preparation_total) + "\n";
    last_progress_ = progress;
  }
  const float bar_width = std::min(width * 0.5f, 240 * dpi);
  const float bar_left = (width - bar_width) * 0.5f;
  Gdiplus::SolidBrush track(Gdiplus::Color(48, 48, 48));
  const auto draw_progress = [&](const wchar_t* label, const size_t completed, const size_t total, const float y) {
    const float fraction = total == 0 ? 0 : std::min(1.0f, float(completed) / float(total));
    const auto text = std::wstring(label) + L"  " + std::to_wstring(completed) + L"/" + std::to_wstring(total) +
                      L"  (" + std::to_wstring(int(fraction * 100)) + L"%)";
    graphics.DrawString(text.c_str(), -1, &font, Gdiplus::RectF(0, y, width, 24 * dpi), &format, &brush);
    graphics.FillRectangle(&track, bar_left, y + 26 * dpi, bar_width, 4 * dpi);
    graphics.FillRectangle(&brush, bar_left, y + 26 * dpi, bar_width * fraction, 4 * dpi);
  };
  draw_progress(L"Scene loading", progress.scene_completed, progress.scene_total, text_bounds.GetBottom() + 8 * dpi);
  draw_progress(L"Shaders / pipelines", progress.preparation_completed, progress.preparation_total,
                text_bounds.GetBottom() + 54 * dpi);
  Gdiplus::Graphics output(dc);
  output.DrawImage(&frame, 0, 0);
  if (!capture_.empty()) {
    UINT count = 0, size = 0;
    Gdiplus::GetImageEncodersSize(&count, &size);
    std::vector<unsigned char> bytes(size);
    auto* codecs = reinterpret_cast<Gdiplus::ImageCodecInfo*>(bytes.data());
    if (Gdiplus::GetImageEncoders(count, size, codecs) == Gdiplus::Ok) {
      for (UINT index = 0; index < count; ++index)
        if (std::wstring_view(codecs[index].MimeType) == L"image/png")
          frame.Save(capture_.c_str(), &codecs[index].Clsid);
    }
    capture_.clear();
  }
  EndPaint(window, &paint);
}
