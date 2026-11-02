#include "EcoSysLabEditorLayer.hpp"

#include <cstdlib>
#include <fstream>
#include <iomanip>

#include "BranchRenderer.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "EcoSysLabSystem.hpp"
#include "GpuTreeMeshRenderer.hpp"
#include "GpuTreePhysics.hpp"
#include "Profiler.hpp"
#include "ProjectManager.hpp"
#include "WindParticleField.hpp"

using namespace eco_sys_lab_package;

void EcoSysLabEditorLayer::ProfileWindyTreeDemo() {
  const char* directory = std::getenv("EVOENGINE_WINDY_TREE_PROFILE");
  if (!directory || !*directory || ProjectManager::GetProjectPath().filename() != "WindyTree.eveproj" ||
      GetApplication().GetApplicationStatus() != Application::ExecutionStatus::Playing)
    return;
  const auto scene = GetScene();
  const auto system = scene->GetSystem<EcoSysLabSystem>();
  if (!system || system->growth.simulated_time < 5.0f * 365.0f || system->growth.grew_this_frame ||
      system->growth.remaining_time > 0.0f || !system->dynamics.tree_physics || !ProjectManager::IsProjectIdle())
    return;
  const auto runtime = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
  const bool full_shadows = std::getenv("EVOENGINE_FULL_BRANCH_SHADOWS") != nullptr;
  if (runtime->gpu_tree_mesh_renderer_)
    runtime->gpu_tree_mesh_renderer_->directional_shadow_impostors = !full_shadows;
  const bool gather = std::getenv("EVOENGINE_ROD_AIR_GATHER") != nullptr;
  if (system->dynamics.wind)
    system->dynamics.wind->rod_driven_coupling = !gather;
  struct Phase {
    const char* name;
    bool plant_visual, branches;
  };
  static constexpr Phase phases[] = {
      {"full", true, true}, {"without_plant_visual", false, true}, {"physics_without_branches", false, false}};
  struct Capture {
    std::weak_ptr<Scene> scene;
    size_t phase = 0, frame = 0;
    bool overlay = true;
    std::vector<std::pair<std::shared_ptr<DynamicTreeSkeleton>, bool>> physics;
    std::vector<std::pair<std::shared_ptr<BranchRenderer>, bool>> branches;
  };
  static Capture capture;
  if (capture.scene.lock() != scene) {
    capture = {};
    capture.scene = scene;
    capture.overlay = tree_visualization_settings_.enable;
    for (const auto& entry : system->dynamics.entries)
      if (const auto physics = entry.physics.lock())
        capture.physics.emplace_back(physics, physics->physics_running);
    if (const auto entities = scene->UnsafeGetPrivateComponentOwnersList<BranchRenderer>())
      for (const auto entity : *entities) {
        const auto renderer = scene->GetOrSetPrivateComponent<BranchRenderer>(entity).lock();
        capture.branches.emplace_back(renderer, renderer->IsEnabled());
      }
    std::filesystem::create_directories(directory);
    std::ofstream config(std::filesystem::path(directory) / "configuration.txt");
    config << "branch_shadows=" << (full_shadows ? "geometry" : "impostor") << "\n"
           << "rod_air=" << (gather ? "gather" : "scatter") << "\n"
           << "device=" << Platform::GetSelectedPhysicalDevice()->properties.deviceName
           << "\ntrees=" << capture.physics.size() << "\nrods=" << system->dynamics.tree_physics->GetRodCount()
           << "\nair_cells=" << (system->dynamics.wind ? system->dynamics.wind->GetAirCellCount() : 0)
           << "\nair_cell_size=" << system->environment.air_cell_size << '\n';
    if (!capture.physics.empty()) {
      const auto& settings = capture.physics.front().first->physics_settings;
      config << "time_step=" << settings.time_step << "\nsubsteps=" << settings.substeps
             << "\niterations=" << settings.iterations << '\n';
      ApplicationContext::Get().GetLayer<EditorLayer>()->SetSelectedEntity(capture.physics.front().first->GetOwner());
    }
    Platform::SetGpuTimestampCaptureEnabled(true);
    Profiler::GetInstance().SetEnabled(true);
    Profiler::GetInstance().SetMaxFrameHistory(64);
  }
  if (capture.phase >= std::size(phases))
    return;
  const auto& phase = phases[capture.phase];
  if (capture.frame == 0) {
    tree_visualization_settings_.enable = phase.plant_visual;
    for (const auto& entry : capture.branches)
      entry.first->SetEnabled(phase.branches && entry.second);
    EVOENGINE_LOG("Windy Tree profile phase: " + std::string(phase.name))
  }
  constexpr size_t warmup_frames = 32, measured_frames = 60;
  if (capture.frame == warmup_frames) {
    Profiler::GetInstance().ClearFrameHistory();
    Platform::ResetGpuTimestampStats();
  }
  if (++capture.frame <= warmup_frames + measured_frames)
    return;
  Platform::WaitForFrameSubmissions("Windy Tree profiling readback");
  const auto frames = Profiler::GetInstance().GetFrameHistorySnapshot();
  const auto begin = frames.size() > measured_frames ? frames.size() - measured_frames : 0;
  std::ofstream output(std::filesystem::path(directory) / (std::string(phase.name) + ".csv"));
  output << std::setprecision(12) << "domain,name,frame,calls,ms\n";
  for (size_t i = begin; i < frames.size(); ++i) {
    const auto& frame = frames[i];
    output << "frame,Wall," << frame.application_frame_index << ",1," << frame.duration_ms << '\n';
    for (const auto& total : BuildProfilerFrameStats(frame).named_event_totals)
      output << "cpu," << std::quoted(total.name) << ',' << frame.application_frame_index << ',' << total.count << ','
             << total.total_ms << '\n';
  }
  for (const auto& frame : Platform::GetGpuTimestampFrameHistory()) {
    if (!frame.results_available || frames.empty() ||
        frame.application_frame_index < frames[begin].application_frame_index ||
        frame.application_frame_index > frames.back().application_frame_index)
      continue;
    output << "gpu_span,Span," << frame.application_frame_index << ",1," << frame.span_milliseconds << '\n';
    for (const auto& pass : BuildGpuTimestampPassAggregates(frame))
      output << "gpu," << std::quoted(pass.metadata.display_name) << ',' << frame.application_frame_index << ','
             << pass.call_count << ',' << pass.total_milliseconds << '\n';
  }
  (void)ExportProfilerChromeTrace(std::filesystem::path(directory) / (std::string(phase.name) + ".trace.json"), frames);
  ApplicationContext::Get().GetLayer<EditorLayer>()->GetSceneCamera()->GetRenderTexture()->StoreToPng(
      std::filesystem::path(directory) / (std::string(phase.name) + ".png"));
  capture.frame = 0;
  if (++capture.phase == std::size(phases)) {
    tree_visualization_settings_.enable = capture.overlay;
    for (const auto& entry : capture.branches)
      entry.first->SetEnabled(entry.second);
    EVOENGINE_LOG("Windy Tree profiling complete: " + std::string(directory))
    GetApplication().End();
  }
}
