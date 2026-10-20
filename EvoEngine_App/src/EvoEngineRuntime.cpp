#include "Application.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "ImGuiLayer.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "RuntimeConfiguration.hpp"
#include "RuntimeGuiLayer.hpp"
#include "RuntimePaths.hpp"
#include "RuntimeStartupScreen.hpp"
#include "Scene.hpp"
#include "Shader.hpp"
#include "WindowLayer.hpp"

#include <Windows.h>
#include <io.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

using namespace evo_engine;

namespace {
class RuntimeConsole final : public std::streambuf {
  HANDLE console_ = INVALID_HANDLE_VALUE;
  std::streambuf* cout_ = std::cout.rdbuf();
  std::streambuf* cerr_ = std::cerr.rdbuf();
  std::streambuf* clog_ = std::clog.rdbuf();
  std::mutex mutex_;

 public:
  RuntimeConsole() {
    if (!GetConsoleWindow() && !AllocConsole())
      throw std::runtime_error("Could not open the runtime console.");
    console_ =
        CreateFileW(L"CONOUT$", GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
    if (console_ == INVALID_HANDLE_VALUE)
      throw std::runtime_error("Could not write to the runtime console.");
    SetConsoleOutputCP(CP_UTF8);
    std::cout.rdbuf(this);
    std::cerr.rdbuf(this);
    std::clog.rdbuf(this);
  }

  ~RuntimeConsole() override {
    pubsync();
    std::cout.rdbuf(cout_);
    std::cerr.rdbuf(cerr_);
    std::clog.rdbuf(clog_);
    CloseHandle(console_);
  }

 protected:
  std::streamsize xsputn(const char* text, const std::streamsize size) override {
    const std::lock_guard lock(mutex_);
    const auto written = cout_->sputn(text, size);
    DWORD console_written = 0;
    WriteFile(console_, text, static_cast<DWORD>(size), &console_written, nullptr);
    return written;
  }
  int_type overflow(const int_type character) override {
    if (traits_type::eq_int_type(character, traits_type::eof()))
      return traits_type::not_eof(character);
    const char value = traits_type::to_char_type(character);
    return xsputn(&value, 1) == 1 ? character : traits_type::eof();
  }
  int sync() override {
    const std::lock_guard lock(mutex_);
    return cout_->pubsync();
  }
};
}  // namespace

int main(const int argc, char** argv) {
  bool error_dialog = true;
  bool logging_ready = false;
  uint64_t frame_limit = 0;
  std::filesystem::path capture_path;
  std::filesystem::path startup_capture_path;
  std::unique_ptr<RuntimeConsole> console;
  std::unique_ptr<Application> application;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string argument = argv[i];
      if (argument == "--no-error-dialog") {
        error_dialog = false;
      } else if (argument == "--frames" && i + 1 < argc) {
        const std::string value = argv[++i];
        size_t consumed = 0;
        frame_limit = std::stoull(value, &consumed);
        if (frame_limit == 0 || value.front() == '-' || consumed != value.size()) {
          throw std::runtime_error("--frames must be positive.");
        }
      } else if (argument == "--capture" && i + 1 < argc) {
        capture_path = runtime_paths::Resolve(std::filesystem::u8path(argv[++i]));
      } else if (argument == "--capture-startup" && i + 1 < argc) {
        startup_capture_path = runtime_paths::Resolve(std::filesystem::u8path(argv[++i]));
      } else {
        throw std::runtime_error("Unknown or incomplete runtime argument: " + argument);
      }
    }
    if (!capture_path.empty() && frame_limit < 2) {
      throw std::runtime_error("--capture requires --frames of at least two.");
    }
    runtime_paths::PrepareWritableDirectories();
    std::filesystem::current_path(runtime_paths::Root());
    const auto log_path = runtime_paths::Resolve("Logs/runtime.log");
    FILE* stream = nullptr;
    if (_wfreopen_s(&stream, log_path.c_str(), L"a", stdout) != 0 || _wfreopen_s(&stream, L"NUL", L"w", stderr) != 0 ||
        _dup2(_fileno(stdout), _fileno(stderr)) != 0) {
      throw std::runtime_error("Cannot open the runtime log in the application directory.");
    }
    logging_ready = true;
    std::cout << std::unitbuf << "Loading runtime configuration." << std::endl;
    const auto scratch = runtime_paths::Resolve("Cache/Scratch").wstring();
    SetEnvironmentVariableW(L"TEMP", scratch.c_str());
    SetEnvironmentVariableW(L"TMP", scratch.c_str());
    const auto config = RuntimeConfiguration::Load(runtime_paths::Resolve("runtime.yaml"));
    if (!startup_capture_path.empty())
      std::filesystem::create_directories(startup_capture_path.parent_path());
    std::atomic_bool scene_loading = false;
    std::atomic_bool scene_ready = false;
    std::atomic_bool preparation_ready = false;
    std::atomic<size_t> preparation_jobs_total = 0;
    std::atomic<size_t> preparation_jobs_completed = 0;
    RuntimeStartupScreen startup(config, startup_capture_path);
    if (config.show_console) {
      console = std::make_unique<RuntimeConsole>();
      std::cout << "Runtime console enabled. Logs: " << log_path.string() << std::endl;
    }
    std::cout << "Initializing runtime application." << std::endl;
    application = std::make_unique<Application>();
    const ApplicationContextScope application_scope(*application);
    startup.SetProgressSource([&] {
      const ApplicationContextScope scope(*application);
      const auto assets = scene_loading ? AssetManager::GetAssetLoadSnapshot() : AssetManager::AssetLoadSnapshot{};
      const auto shaders = Shader::GetPreparationProgress();
      RuntimeStartupScreen::Progress progress;
      progress.scene_total = assets.cumulative_total + 1;
      progress.scene_completed = assets.cumulative_completed + (scene_ready ? 1 : 0);
      progress.preparation_completed = shaders.completed + preparation_jobs_completed.load();
      progress.preparation_total = shaders.total + preparation_jobs_total.load() + 1;
      if (preparation_ready)
        progress.preparation_completed = progress.preparation_total;
      return progress;
    });
    auto renderer = application->PushLayer<RenderLayer>("Runtime rendering");
    auto window = application->PushLayer<WindowLayer>("Runtime window");
    application->PushLayer<ImGuiLayer>("Runtime GUI");
    application->PushLayer<RuntimeGuiLayer>("Runtime GUI");
    auto settings = config.ApplicationSettings();
    if (config.window_mode == WindowDisplayMode::ExclusiveFullscreen)
      settings.window_mode = WindowDisplayMode::BorderlessFullscreen;
    settings.startup_status = [&startup](const std::string_view status) {
      startup.SetStatus(status);
    };
    application->Initialize(settings);
    scene_loading = true;
    if (application->GetApplicationStatus() == Application::ExecutionStatus::Uninitialized) {
      throw std::runtime_error("Runtime initialization failed.");
    }
    config.ValidateLoadedPackages();
    bool started = false;
    bool visible = false;
    bool capture_requested = false;
    uint64_t frames = 0;
    while (!startup.Cancelled() && application->Loop()) {
      if (startup.Cancelled())
        break;
      if (ProjectManager::GetProjectState() == ProjectState::Failed) {
        throw std::runtime_error(ProjectManager::GetProjectFailure());
      }
      const auto scene = application->GetActiveScene();
      scene_ready = ProjectManager::IsProjectIdle();
      const auto shader_progress = renderer->GetShaderPreparationProgress();
      // Publish the total first so the screen cannot observe more completed jobs than discovered jobs.
      preparation_jobs_total = shader_progress.total;
      preparation_jobs_completed = shader_progress.completed;
      if (const auto error = renderer->GetSceneShaderPrewarmError(scene); !error.empty())
        throw std::runtime_error("Runtime shader preparation failed: " + error);
      if (!started && ProjectManager::GetProjectState() == ProjectState::Loaded && renderer->HasPresentedScene(scene)) {
        startup.SetStatus("Preparing the first playable frame...");
        application->Start(true);
        started = true;
        std::cout << "RUNTIME_STARTED scene="
                  << static_cast<uint64_t>(ProjectManager::GetStartScene().lock()->GetHandle())
                  << " packages=" << config.packages.size() << std::endl;
        continue;
      }
      if (!started && ProjectManager::GetProjectState() == ProjectState::Loaded)
        startup.SetStatus("Preparing shaders and pipelines...");
      if (!started || application->GetApplicationStatus() != Application::ExecutionStatus::Playing) {
        continue;
      }
      if (!visible) {
        if (!renderer->HasPresentedScene(scene))
          continue;
        Platform::WaitForFrameSubmissions("Runtime Startup Frame Ready");
        preparation_ready = true;
        window->ShowWindow();
        if (config.window_mode == WindowDisplayMode::ExclusiveFullscreen)
          window->SetDisplayMode(config.window_mode);
        startup.Close();
        visible = true;
        std::cout << "RUNTIME_READY" << std::endl;
      }
      ++frames;
      if (!capture_path.empty() && !capture_requested && frames + 1 >= frame_limit) {
        std::filesystem::create_directories(capture_path.parent_path());
        window->RequestScreenshot(capture_path);
        capture_requested = true;
      }
      if (frame_limit != 0 && frames >= frame_limit) {
        const auto camera = application->GetActiveScene()->main_camera.Get<Camera>();
        if (!camera || camera->GetFrameCount() == 0) {
          throw std::runtime_error("Runtime main camera did not render.");
        }
        if (capture_requested) {
          std::string error;
          if (!window->StoreCompletedScreenshot(error)) {
            if (!error.empty()) {
              throw std::runtime_error(error);
            }
            continue;
          }
        }
        std::cout << "RUNTIME_SMOKE_COMPLETE frames=" << frames << " camera=" << camera->GetSize().x << "x"
                  << camera->GetSize().y << std::endl;
        break;
      }
    }
    window.reset();
    renderer.reset();
    application->Terminate();
    return 0;
  } catch (const std::exception& error) {
    if (logging_ready) {
      std::cerr << "RUNTIME_FATAL: " << error.what() << std::endl;
    }
    std::fflush(nullptr);
    if (error_dialog) {
      MessageBoxA(nullptr, error.what(), "EvoEngine Runtime", MB_OK | MB_ICONERROR);
    }
    // DLL detach can wait on GPU workers already stopped by ExitProcess after a partial startup.
    TerminateProcess(GetCurrentProcess(), EXIT_FAILURE);
    std::_Exit(EXIT_FAILURE);
  }
}
