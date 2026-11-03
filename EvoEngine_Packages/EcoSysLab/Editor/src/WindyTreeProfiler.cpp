#include "EcoSysLabEditorLayer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>

#include "BranchRenderer.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "EcoSysLabSystem.hpp"
#include "GpuTreeClusterPhysics.hpp"
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
  const bool profile_growth = std::getenv("EVOENGINE_WINDY_TREE_GROWTH_PROFILE") != nullptr;
  static std::weak_ptr<Scene> growth_capture_scene, growth_ready_scene;
  if (system && profile_growth && growth_capture_scene.lock() != scene) {
    growth_capture_scene = scene;
    auto& profiler = Profiler::GetInstance();
    profiler.SetEnabled(true);
    profiler.SetMaxFrameHistory(1024);
    profiler.ClearFrameHistory();
  }
  if (system) {
    if (const auto order = std::getenv("EVOENGINE_TREE_CLUSTER_ORDER")) {
      system->dynamics.use_subtree_clusters = true;
      system->dynamics.cluster_lod.enabled_policies = 2;
      system->dynamics.cluster_lod.rules[1].z = glm::clamp(std::strtof(order, nullptr), 0.0f, 64.0f);
    }
    if (const auto tolerance = std::getenv("EVOENGINE_TREE_PHYSICS_TOLERANCE"))
      system->dynamics.physics_approximation_tolerance = glm::clamp(std::strtof(tolerance, nullptr), 0.0f, 2.0f);
  }
  if (!system || system->growth.simulated_time < 5.0f * 365.0f || system->growth.grew_this_frame ||
      system->growth.remaining_time > 0.0f || !system->dynamics.tree_physics || !ProjectManager::IsProjectIdle())
    return;
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeSkeleton>())
    for (const auto entity : *owners) {
      const auto physics = scene->GetOrSetPrivateComponent<DynamicTreeSkeleton>(entity).lock();
      if (scene->IsEntityEnabled(entity) && physics->IsEnabled() && !physics->GetGpuPhysicsState())
        return;
    }
  // Wait one frame so the completed capture includes the final solver preparation.
  if (profile_growth && growth_ready_scene.lock() != scene) {
    growth_ready_scene = scene;
    return;
  }
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
    if (profile_growth) {
      auto frames = Profiler::GetInstance().GetFrameHistorySnapshot();
      const auto growth_frame = [](const auto& frame) {
        return std::any_of(frame.events.begin(), frame.events.end(), [](const auto& event) {
          return event.name == "EcoSysLab / Growth step";
        });
      };
      const auto write_frames = [&](const char* name) {
        std::ofstream output(std::filesystem::path(directory) / name);
        output << std::setprecision(12) << "domain,name,frame,calls,ms\n";
        for (const auto& frame : frames) {
          output << "frame,Wall," << frame.application_frame_index << ",1," << frame.duration_ms << '\n';
          for (const auto& total : BuildProfilerFrameStats(frame).named_event_totals)
            output << "cpu," << std::quoted(total.name) << ',' << frame.application_frame_index << ',' << total.count
                   << ',' << total.total_ms << '\n';
        }
      };
      frames.erase(frames.begin(), std::find_if(frames.begin(), frames.end(), growth_frame));
      write_frames("growth-session.csv");
      frames.erase(std::remove_if(frames.begin(), frames.end(),
                                  [&](const auto& frame) {
                                    return !growth_frame(frame);
                                  }),
                   frames.end());
      write_frames("growth.csv");
      (void)ExportProfilerChromeTrace(std::filesystem::path(directory) / "growth.trace.json", frames);
      std::ofstream skeletons(std::filesystem::path(directory) / "growth-skeletons.csv");
      skeletons << std::setprecision(9) << "tree,node,parent,x,y,z,length,thickness\n";
      if (const auto trees = scene->UnsafeGetPrivateComponentOwnersList<Tree>())
        for (size_t i = 0; i < trees->size(); ++i) {
          const auto tree = scene->GetOrSetPrivateComponent<Tree>(trees->at(i)).lock();
          const auto& skeleton = tree->shoot_model.PeekShootSkeleton();
          for (const auto handle : skeleton.PeekSortedNodeList()) {
            const auto& node = skeleton.PeekNode(handle);
            skeletons << i << ',' << handle << ',' << node.GetParentHandle() << ',' << node.info.global_position.x
                      << ',' << node.info.global_position.y << ',' << node.info.global_position.z << ','
                      << node.info.length << ',' << node.info.thickness << '\n';
          }
        }
    }
    std::ofstream config(std::filesystem::path(directory) / "configuration.txt");
    config << "branch_shadows=" << (full_shadows ? "geometry" : "impostor") << "\n"
           << "rod_air=" << (gather ? "gather" : "scatter") << "\n"
           << "device=" << Platform::GetSelectedPhysicalDevice()->properties.deviceName
           << "\ntrees=" << capture.physics.size() << "\nrods=" << system->dynamics.tree_physics->GetRodCount()
           << "\nair_cells=" << (system->dynamics.wind ? system->dynamics.wind->GetAirCellCount() : 0)
           << "\nair_cell_size=" << system->environment.air_cell_size << '\n';
    config << "subtree_clusters=" << system->dynamics.use_subtree_clusters
           << "\ncluster_order=" << system->dynamics.cluster_lod.rules[1].z << '\n';
    config << "physics_tolerance=" << system->dynamics.physics_approximation_tolerance << '\n';
    for (const auto id : system->dynamics.tree_physics->GetActiveTreeIds()) {
      const auto& state = system->dynamics.tree_physics->GetTreeState(id);
      config << "tree_" << id << "_level_rods=" << state->rod_count;
      if (state->lod)
        for (const auto& level : state->lod->templates)
          config << ',' << level->rod_count;
      config << '\n';
      if (state->lod) {
        std::vector<GpuTreePhysicsLodLevel> levels;
        state->lod->levels->DownloadVector(levels, state->lod->level_count);
        config << "tree_" << id << "_level_min_tolerance=0";
        for (size_t i = 1; i < levels.size(); ++i)
          config << ',' << std::max(levels[i].metrics.x, levels[i].metrics.z);
        config << '\n';
      }
    }
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
  if (std::getenv("EVOENGINE_TREE_CLUSTER_VALIDATE") && system->dynamics.use_subtree_clusters && capture.phase == 0 &&
      capture.frame < warmup_frames) {
    if (capture.frame < 8)
      system->dynamics.cluster_lod.rules[1].z = 3;
    else if (capture.frame < 16)
      system->dynamics.cluster_lod.rules[1].z = 1;
    else if (capture.frame < 24)
      system->dynamics.cluster_lod.enabled_policies = 0;
  }
  if (capture.frame == warmup_frames) {
    Profiler::GetInstance().ClearFrameHistory();
    Platform::ResetGpuTimestampStats();
  }
  if (++capture.frame <= warmup_frames + measured_frames)
    return;
  Platform::WaitForFrameSubmissions("Windy Tree profiling readback");
  {
    std::ofstream levels(std::filesystem::path(directory) / (std::string(phase.name) + ".lod.txt"));
    uint32_t rods = 0;
    for (const auto id : system->dynamics.tree_physics->GetActiveTreeIds()) {
      const auto& state = system->dynamics.tree_physics->GetTreeState(id);
      uint32_t selected = 0;
      if (state->lod) {
        std::vector<uint32_t> values;
        state->lod->state->DownloadVector(values, 1);
        selected = values.front();
      }
      const auto count = selected ? state->lod->templates[selected - 1]->rod_count : state->rod_count;
      levels << "tree=" << id << " level=" << selected << " rods=" << count << '\n';
      rods += count;
    }
    levels << "active_rods=" << rods << '\n';
    if (system->dynamics.cluster_forest) {
      uint32_t bodies = 0, invalid = 0;
      for (const auto& entry : system->dynamics.entries)
        if (entry.cluster) {
          std::vector<glm::uvec4> status;
          entry.cluster->GetLodStateBuffer()->DownloadVector(status, 6);
          bodies += status[0].x;
          levels << "cluster_tree=" << entry.gpu_id << " bodies=" << status[0].x << " depth=" << status[1].z
                 << " transitions=" << status[0].w << '\n';
          std::vector<glm::vec4> values;
          const auto& state = entry.cluster->GetTreeState();
          state->vertices[0]->DownloadVector(values, state->vertex_count * 4);
          for (uint32_t i = 0; i < state->vertex_count; ++i)
            if (!std::isfinite(glm::length(glm::vec3(values[i]))))
              ++invalid;
        }
      levels << "active_cluster_bodies=" << bodies << "\ninvalid_cluster_vertices=" << invalid << '\n';
    }
  }
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
