#include "LSystemLayer.hpp"

#include "Application.hpp"
#include "Camera.hpp"
#include "EditorLayer.hpp"
#include "Jobs.hpp"
#include "LSystemInspectionAdapters.hpp"
#include "LSystemSerializationAdapters.hpp"
#include "Platform.hpp"
#include "Scene.hpp"
#include "ScotsPine.hpp"
#include "ScotsPineDescriptor.hpp"
#include "SorghumLS.hpp"
#include "SorghumLSDescriptor.hpp"
#include "Times.hpp"
#include "TransformGraph.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <random>
#include <unordered_map>

using namespace l_system_package;
using namespace evo_engine;

namespace {

template <typename Selector>
double AverageProfileMetric(const std::vector<LSystemLayer::ProfileFrame>& frames, Selector selector) {
  if (frames.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (const auto& frame : frames) {
    sum += selector(frame);
  }
  return sum / static_cast<double>(frames.size());
}

template <typename Selector>
double MaxProfileMetric(const std::vector<LSystemLayer::ProfileFrame>& frames, Selector selector) {
  double max_value = 0.0;
  for (const auto& frame : frames) {
    max_value = std::max(max_value, selector(frame));
  }
  return max_value;
}

float SampleDescriptorTargetGddForPine(ScotsPine& pine) {
  auto descriptor = pine.descriptor_ref.Get<ScotsPineDescriptor>();
  if (pine.enable_repot_profile_switch && pine.growth_model.IsPostRepotProfileActive()) {
    if (const auto post_descriptor = pine.post_repot_descriptor_ref.Get<ScotsPineDescriptor>()) {
      descriptor = post_descriptor;
    }
  }
  if (!descriptor) {
    return -1.0f;
  }

  std::mt19937 rng(pine.seed);
  return std::max(0.0f, SampleDistribution(descriptor->target_gdd, rng));
}

float SampleDescriptorTargetGddForSorghum(SorghumLS& sorghum) {
  const auto descriptor = sorghum.descriptor_ref.Get<SorghumLSDescriptor>();
  if (!descriptor) {
    return -1.0f;
  }

  std::mt19937 rng(sorghum.seed);
  return std::max(0.0f, SampleDistribution(descriptor->target_gdd, rng));
}

float NormalizeDayOfYear(float day) {
  if (!std::isfinite(day)) {
    return 0.0f;
  }
  day = std::fmod(day, 365.0f);
  if (day < 0.0f) {
    day += 365.0f;
  }
  return day;
}

bool IsInActiveSeason(const float simulation_day_of_year, const int season_start_day, const int season_end_day) {
  const int day = static_cast<int>(std::floor(NormalizeDayOfYear(simulation_day_of_year)));
  const int start = std::clamp(season_start_day, 0, 364);
  const int end = std::clamp(season_end_day, 0, 364);
  if (start <= end) {
    return day >= start && day <= end;
  }
  return day >= start || day <= end;
}

int ClampColorModeIndex(const int mode) {
  return std::clamp(mode, 0, 6);
}

int ResolveEffectiveColorMode(const int selected_mode, const bool scene_plant_view_tint_enabled) {
  return scene_plant_view_tint_enabled ? ClampColorModeIndex(selected_mode) : 0;
}

void ApplyGlobalPlantColorMode(const int selected_mode, const bool scene_plant_view_tint_enabled) {
  const int effective_mode = ResolveEffectiveColorMode(selected_mode, scene_plant_view_tint_enabled);
  ScotsPine::SetGlobalColorMode(static_cast<ScotsPine::ColorMode>(effective_mode));
  SorghumLS::SetGlobalColorMode(static_cast<SorghumLS::ColorMode>(std::clamp(effective_mode, 0, 4)));
}

void ApplyPineStemOnlyMode(const std::shared_ptr<Scene>& scene, const bool stem_only_mode,
                           const bool regenerate_existing_pines) {
  ScotsPine::SetGenerateNeedleTopologyEnabled(!stem_only_mode);

  if (!regenerate_existing_pines || !scene) {
    return;
  }

  if (const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>()) {
    const std::vector<Entity> pine_entities = *pine_entities_ptr;
    for (const auto& entity : pine_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      if (const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock()) {
        pine->GrowToTargetGDD();
      }
    }
  }
}

template <typename GrowthStepProfile>
void AccumulateGrowthProfile(LSystemLayer::ProfileFrame& frame, const GrowthStepProfile& profile) {
  frame.growth_rules_ms += profile.apply_growth_rules_seconds * 1000.0;
  frame.topology_rules_ms += profile.apply_topology_rules_seconds * 1000.0;
  frame.sort_lists_ms += profile.sort_lists_seconds * 1000.0;
  frame.update_node_info_ms += profile.update_node_info_seconds * 1000.0;
  frame.propagate_geometry_ms += profile.propagate_geometry_seconds * 1000.0;
  frame.topology_scan_ms += profile.topology_scan_seconds * 1000.0;
}

void RebuildAllPlantGeometry(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return;
  }

  if (const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>()) {
    const std::vector<Entity> pine_entities = *pine_entities_ptr;
    for (const auto& entity : pine_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      if (const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock()) {
        pine->RebuildGeometry();
      }
    }
  }

  if (const auto* sorghum_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>()) {
    const std::vector<Entity> sorghum_entities = *sorghum_entities_ptr;
    for (const auto& entity : sorghum_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      if (const auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock()) {
        sorghum->RebuildGeometry();
      }
    }
  }
}

struct PineTemporalSample {
  float gdd_per_day = 2.0f;
  int season_start_day = 60;
  int season_end_day = 334;
};

PineTemporalSample SamplePineTemporalParameters(ScotsPine& pine) {
  PineTemporalSample sample;
  auto descriptor = pine.descriptor_ref.Get<ScotsPineDescriptor>();
  if (pine.enable_repot_profile_switch && pine.growth_model.IsPostRepotProfileActive()) {
    if (const auto post_descriptor = pine.post_repot_descriptor_ref.Get<ScotsPineDescriptor>()) {
      descriptor = post_descriptor;
    }
  }
  if (!descriptor) {
    return sample;
  }

  std::mt19937 rng(static_cast<uint32_t>(pine.seed) ^ 0x5f3759dfu);
  sample.gdd_per_day = std::max(0.0f, SampleDistribution(descriptor->gdd_per_day, rng));

  const auto sample_day = [&](const SingleDistribution<float>& distribution) {
    const float sampled_day = std::clamp(SampleDistribution(distribution, rng), 0.0f, 365.0f);
    return static_cast<int>(std::floor(NormalizeDayOfYear(std::round(sampled_day))));
  };

  sample.season_start_day = sample_day(descriptor->growing_season_start_day);
  sample.season_end_day = sample_day(descriptor->growing_season_end_day);
  return sample;
}

}  // namespace

namespace {
enum class SorghumGeometryPass { Full, Incremental, Preview };

size_t PublishSorghumGeometry(const std::shared_ptr<Scene>& scene,
                              const std::vector<std::shared_ptr<SorghumLS>>& plants, const bool update_render_geometry,
                              const SorghumGeometryPass pass = SorghumGeometryPass::Full,
                              const float preview_target_gdd = 1000.0f, const uint32_t preview_max_growth_steps = 64u) {
  if (plants.empty()) {
    return 0;
  }
  std::vector<std::shared_ptr<const SorghumGeometrySnapshot>> snapshots(plants.size());
  Jobs::RunParallelFor(plants.size(), [&](const size_t index) {
    switch (pass) {
      case SorghumGeometryPass::Incremental:
        snapshots[index] = plants[index]->AdvanceGeometrySnapshot(true);
        break;
      case SorghumGeometryPass::Preview:
        snapshots[index] = plants[index]->GeneratePreviewGeometrySnapshot(preview_target_gdd, preview_max_growth_steps);
        break;
      case SorghumGeometryPass::Full:
        snapshots[index] = plants[index]->GenerateGeometrySnapshot(true);
        break;
    }
  });
  for (size_t index = 0; index < plants.size(); ++index) {
    plants[index]->PublishGeometrySnapshot(snapshots[index], update_render_geometry);
  }
  TransformGraph::CalculateTransformGraphs(scene);
  return plants.size();
}
}  // namespace

void LSystemLayer::OnCreate() {
  simulation_day_of_year = NormalizeDayOfYear(static_cast<float>(std::clamp(season_start_day, 0, 364)));
  ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
  ApplyPineStemOnlyMode(GetScene(), pine_stem_only_mode, true);
  enable_inspection = GetApplication().GetLayer<EditorLayer>() != nullptr;
}

void LSystemLayer::OnDestroy() {
}

void LSystemLayer::PreUpdate() {
  RestoreSorghumScene();
}

size_t LSystemLayer::RegenerateSorghumScene(const float evaluation_gdd, const int seed_base,
                                            const bool update_render_geometry) const {
  const auto scene = GetScene();
  if (!scene) {
    return 0;
  }
  std::vector<std::shared_ptr<SorghumLS>> plants;
  uint32_t seed_offset = 0;
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>()) {
    for (const auto& entity : *owners) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      const auto plant = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
      if (!plant || !plant->descriptor_ref.Get<SorghumLSDescriptor>()) {
        continue;
      }
      if (seed_base >= 0) {
        plant->seed = static_cast<uint32_t>(seed_base) + seed_offset;
      }
      plant->target_gdd = std::max(0.0f, evaluation_gdd);
      plants.emplace_back(plant);
      ++seed_offset;
    }
  }
  return PublishSorghumGeometry(scene, plants, update_render_geometry);
}

size_t LSystemLayer::AdvanceSorghumScene(const float evaluation_gdd, const int seed_base,
                                         const bool update_render_geometry) const {
  const auto scene = GetScene();
  if (!scene) {
    return 0;
  }
  std::vector<std::shared_ptr<SorghumLS>> plants;
  uint32_t seed_offset = 0;
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>()) {
    for (const auto& entity : *owners) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      const auto plant = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
      if (!plant || !plant->descriptor_ref.Get<SorghumLSDescriptor>()) {
        continue;
      }
      if (seed_base >= 0) {
        plant->seed = static_cast<uint32_t>(seed_base) + seed_offset;
      }
      plant->target_gdd = std::max(0.0f, evaluation_gdd);
      plants.emplace_back(plant);
      ++seed_offset;
    }
  }
  return PublishSorghumGeometry(scene, plants, update_render_geometry, SorghumGeometryPass::Incremental);
}

size_t LSystemLayer::RestoreSorghumScene(const bool update_render_geometry) const {
  const auto scene = GetScene();
  if (!scene) {
    return 0;
  }
  std::vector<std::shared_ptr<SorghumLS>> plants;
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>()) {
    for (const auto& entity : *owners) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      const auto plant = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
      if (plant && !plant->growth_model.IsInitialized() && plant->target_gdd > 0.0f &&
          plant->descriptor_ref.Get<SorghumLSDescriptor>()) {
        plants.emplace_back(plant);
      }
    }
  }
  return PublishSorghumGeometry(scene, plants, update_render_geometry);
}

size_t LSystemLayer::RegenerateSorghumDescriptor(const SorghumLSDescriptor& descriptor, const bool representative_only,
                                                 const bool preview, const float preview_target_gdd,
                                                 const uint32_t preview_max_growth_steps) const {
  const auto scene = GetScene();
  if (!scene) {
    return 0;
  }

  std::vector<std::shared_ptr<SorghumLS>> plants;
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>()) {
    for (const auto& entity : *owners) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      const auto plant = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
      if (!plant || plant->descriptor_ref.Get<SorghumLSDescriptor>().get() != &descriptor) {
        continue;
      }
      plant->target_gdd = SampleDescriptorTargetGddForSorghum(*plant);
      plants.emplace_back(plant);
      if (representative_only) {
        break;
      }
    }
  }

  return PublishSorghumGeometry(scene, plants, true, preview ? SorghumGeometryPass::Preview : SorghumGeometryPass::Full,
                                preview_target_gdd, preview_max_growth_steps);
}

void LSystemLayer::PushProfileFrame(const ProfileFrame& frame) {
  if (profiling_history_size <= 0) {
    return;
  }
  if (profiling_history.size() >= static_cast<size_t>(profiling_history_size)) {
    profiling_history.erase(profiling_history.begin());
  }
  profiling_history.push_back(frame);
}

void LSystemLayer::ExportProfileCsv(const std::string& path) const {
  std::error_code filesystem_error;
  const std::filesystem::path output_path(path);
  const auto parent_path = output_path.parent_path();
  if (!parent_path.empty()) {
    std::filesystem::create_directories(parent_path, filesystem_error);
    if (filesystem_error) {
      return;
    }
  }

  std::ofstream out(path, std::ios::trunc);
  if (!out.is_open()) {
    return;
  }

  out << "frame,update_ms,grow_ms,growth_rules_ms,topology_rules_ms,sort_lists_ms,update_node_info_ms,"
         "propagate_geometry_ms,topology_scan_ms,rebuild_ms,internode_rebuild_ms,leaf_spline_ms,leaf_mesh_ms,"
         "mesh_upload_ms,pines,sorghums,growth_steps,nodes,internodes,needles,leaves,"
         "live_leaves,invalid_instances\n";
  for (size_t i = 0; i < profiling_history.size(); i++) {
    const auto& f = profiling_history[i];
    out << i << "," << std::fixed << std::setprecision(4) << f.update_ms << "," << f.grow_ms << "," << f.growth_rules_ms
        << "," << f.topology_rules_ms << "," << f.sort_lists_ms << "," << f.update_node_info_ms << ","
        << f.propagate_geometry_ms << "," << f.topology_scan_ms << "," << f.rebuild_ms << "," << f.internode_rebuild_ms
        << "," << f.leaf_spline_ms << "," << f.leaf_mesh_ms << "," << f.mesh_upload_ms << "," << f.pine_count << ","
        << f.sorghum_count << "," << f.growth_steps << "," << f.node_count << "," << f.internode_count << ","
        << f.needle_count << "," << f.leaf_count << "," << f.live_leaf_count << "," << f.invalid_instance_count << "\n";
  }
}

void LSystemLayer::Update() {
  auto& times = GetApplication().GetTimes();
  const double update_start = times.Now();
  ProfileFrame frame{};

  if (!auto_grow) {
    return;
  }

  const auto scene = GetScene();
  if (!scene) {
    return;
  }

  const float dt = static_cast<float>(times.DeltaTime());
  if (auto_grow_fps_failsafe_enabled && dt > 0.0f) {
    const float fps = 1.0f / dt;
    if (fps < kAutoGrowFailsafeMinFps) {
      auto_grow = false;
      fps_failsafe_tripped_ = true;
      last_failsafe_fps_ = fps;
      return;
    }
  }

  const float growth_dt = auto_grow_max_delta_time > 0.0f ? std::min(dt, auto_grow_max_delta_time) : dt;
  const float delta_days = std::max(0.0f, chronological_days_per_second) * std::max(0.0f, growth_dt);
  if (seasonality_enabled && delta_days > 0.0f) {
    simulation_day_of_year = NormalizeDayOfYear(simulation_day_of_year + delta_days);
  }
  const float delta_years = seasonality_enabled ? (delta_days / 365.0f) : 0.0f;

  if (const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>()) {
    const std::vector<Entity> pine_entities = *pine_entities_ptr;
    for (const auto& entity : pine_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }

      auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
      if (!pine) {
        continue;
      }
      frame.pine_count++;

      const PineTemporalSample pine_temporal = SamplePineTemporalParameters(*pine);
      const bool pine_in_active_season =
          !seasonality_enabled ||
          IsInActiveSeason(simulation_day_of_year, pine_temporal.season_start_day, pine_temporal.season_end_day);
      const float pine_delta_gdd =
          pine_in_active_season ? std::max(0.0f, pine_temporal.gdd_per_day) * delta_days : 0.0f;

      const float pine_season_days =
          (pine_temporal.season_end_day >= pine_temporal.season_start_day)
              ? static_cast<float>(pine_temporal.season_end_day - pine_temporal.season_start_day + 1)
              : static_cast<float>(365 - pine_temporal.season_start_day + pine_temporal.season_end_day + 1);
      const float pine_season_length_years = std::max(1e-3f, pine_season_days / 365.0f);

      pine->SetSeasonalChronologicalMode(seasonality_enabled);

      if (seasonality_enabled && delta_years > 0.0f) {
        if (pine_in_active_season) {
          if (!pine->growth_model.IsInitialized()) {
            if (const auto descriptor = pine->descriptor_ref.Get<ScotsPineDescriptor>()) {
              std::shared_ptr<ScotsPineDescriptor> post_descriptor = nullptr;
              float repot_switch_gdd = -1.0f;
              if (pine->enable_repot_profile_switch) {
                post_descriptor = pine->post_repot_descriptor_ref.Get<ScotsPineDescriptor>();
                if (post_descriptor) {
                  repot_switch_gdd = std::max(0.0f, pine->repot_switch_gdd);
                }
              }

              pine->growth_model.Initialize(*descriptor, pine->seed, glm::vec3(0), kDefaultRootRotation,
                                            post_descriptor.get(), repot_switch_gdd,
                                            ScotsPine::IsGenerateNeedleTopologyEnabled());
            }
          }
          pine->growth_model.AdvanceChronologicalYears(delta_years);
        } else {
          pine->AdvanceChronologicalAging(delta_years);
        }
      }

      if (!pine_in_active_season) {
        if (pine->growth_model.IsInitialized()) {
          pine->growth_model.graph.data.clock.SyncSeasonalState(seasonality_enabled, pine_in_active_season,
                                                                pine_season_length_years);
        }
        continue;
      }

      if (pine->growth_model.IsInitialized()) {
        pine->growth_model.graph.data.clock.SyncSeasonalState(seasonality_enabled, pine_in_active_season,
                                                              pine_season_length_years);
      }

      const float descriptor_target_gdd = SampleDescriptorTargetGddForPine(*pine);
      const float next_target_gdd = std::max(0.0f, pine->target_gdd + pine_delta_gdd);
      pine->target_gdd =
          descriptor_target_gdd >= 0.0f ? std::min(next_target_gdd, descriptor_target_gdd) : next_target_gdd;

      pine->GrowToTargetGDD();

      if (profiling_enabled) {
        frame.grow_ms += pine->last_grow_seconds * 1000.0;
        AccumulateGrowthProfile(frame, pine->growth_model.last_grow_to_gdd_profile);
        frame.rebuild_ms += pine->last_rebuild_seconds * 1000.0;
        frame.growth_steps += pine->growth_model.last_growth_steps;
        frame.node_count += pine->last_node_count;
        frame.internode_count += pine->last_internode_count;
        frame.needle_count += pine->last_needle_count;
        frame.invalid_instance_count += pine->last_invalid_instance_count;
      }
    }
  }

  if (const auto* sorghum_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>()) {
    const std::vector<Entity> sorghum_entities = *sorghum_entities_ptr;
    for (const auto& entity : sorghum_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }

      auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
      if (!sorghum) {
        continue;
      }
      frame.sorghum_count++;

      const bool sorghum_in_active_season =
          !seasonality_enabled || IsInActiveSeason(simulation_day_of_year, season_start_day, season_end_day);
      float sorghum_delta_gdd = 0.0f;
      if (sorghum_in_active_season) {
        if (const auto descriptor = sorghum->descriptor_ref.Get<SorghumLSDescriptor>()) {
          std::mt19937 rng(static_cast<uint32_t>(sorghum->seed) ^ 0x51e9a5afu);
          sorghum_delta_gdd = std::max(0.0f, SampleDistribution(descriptor->gdd_per_day, rng)) * delta_days;
        }
      }

      sorghum->SetSeasonalChronologicalMode(seasonality_enabled);
      if (seasonality_enabled && delta_years > 0.0f && !sorghum_in_active_season) {
        sorghum->AdvanceChronologicalAging(delta_years);
        continue;
      }

      const float descriptor_target_gdd = SampleDescriptorTargetGddForSorghum(*sorghum);
      const float next_target_gdd = std::max(0.0f, sorghum->target_gdd + sorghum_delta_gdd);
      sorghum->target_gdd =
          descriptor_target_gdd >= 0.0f ? std::min(next_target_gdd, descriptor_target_gdd) : next_target_gdd;
      sorghum->GrowToTargetGDD(false, sorghum_growth_step_cap_per_update);

      if (profiling_enabled) {
        frame.grow_ms += sorghum->last_grow_seconds * 1000.0;
        AccumulateGrowthProfile(frame, sorghum->growth_model.last_grow_to_gdd_profile);
        frame.rebuild_ms += sorghum->last_rebuild_seconds * 1000.0;
        frame.internode_rebuild_ms += sorghum->last_rebuild_internode_seconds * 1000.0;
        frame.leaf_spline_ms += sorghum->last_leaf_spline_seconds * 1000.0;
        frame.leaf_mesh_ms += sorghum->last_leaf_mesh_seconds * 1000.0;
        frame.mesh_upload_ms += sorghum->last_mesh_upload_seconds * 1000.0;
        frame.growth_steps += sorghum->growth_model.last_growth_steps;
        frame.node_count += sorghum->last_node_count;
        frame.internode_count += sorghum->last_internode_count;
        frame.leaf_count += sorghum->last_leaf_count;
        frame.live_leaf_count += sorghum->last_live_leaf_count;
        frame.invalid_instance_count += sorghum->last_invalid_instance_count;
      }
    }
  }

  if (profiling_enabled) {
    frame.update_ms = (times.Now() - update_start) * 1000.0;
    last_profile_frame = frame;
    PushProfileFrame(frame);
  }
}

bool l_system_package::InspectLSystemLayer(InspectorContext& context, LSystemLayer& layer) {
  auto& auto_grow = layer.auto_grow;
  auto& auto_grow_fps_failsafe_enabled = layer.auto_grow_fps_failsafe_enabled;
  auto& reseed_on_reset = layer.reseed_on_reset;
  auto& seasonality_enabled = layer.seasonality_enabled;
  auto& season_start_day = layer.season_start_day;
  auto& season_end_day = layer.season_end_day;
  auto& chronological_days_per_second = layer.chronological_days_per_second;
  auto& auto_grow_max_delta_time = layer.auto_grow_max_delta_time;
  auto& simulation_day_of_year = layer.simulation_day_of_year;
  auto& sorghum_growth_step_cap_per_update = layer.sorghum_growth_step_cap_per_update;
  auto& tassel_color_mode = layer.tassel_color_mode;
  auto& scene_plant_view_tint_enabled = layer.scene_plant_view_tint_enabled;
  auto& pine_stem_only_mode = layer.pine_stem_only_mode;
  auto& profiling_enabled = layer.profiling_enabled;
  auto& profiling_history_size = layer.profiling_history_size;
  auto& profiling_export_path = layer.profiling_export_path;
  auto& last_profile_frame = layer.last_profile_frame;
  auto& profiling_history = layer.profiling_history;
  auto& fps_failsafe_tripped_ = layer.fps_failsafe_tripped_;
  auto& last_failsafe_fps_ = layer.last_failsafe_fps_;

  auto reset_all_lsystems = [&]() {
    const auto scene = layer.GetScene();
    if (!scene) {
      return;
    }

    unsigned int base_seed = 0u;
    if (reseed_on_reset) {
      base_seed = static_cast<unsigned int>(std::chrono::steady_clock::now().time_since_epoch().count() & 0xffffffffu);
    }
    unsigned int seed_offset = 0u;

    if (const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>()) {
      const std::vector<Entity> pine_entities = *pine_entities_ptr;
      for (const auto& entity : pine_entities) {
        if (!scene->IsEntityValid(entity)) {
          continue;
        }
        auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
        if (!pine) {
          continue;
        }

        if (reseed_on_reset) {
          pine->seed = base_seed + seed_offset++;
        }
        // Hard reset for Ctrl+W: clear all generated geometry and leave the
        // plant at zero thermal target (no warm-start regrowth).
        pine->target_gdd = 0.0f;
        pine->growth_model.Reset();
        pine->ClearGeometryEntities();
      }
    }

    if (const auto* sorghum_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>()) {
      const std::vector<Entity> sorghum_entities = *sorghum_entities_ptr;
      for (const auto& entity : sorghum_entities) {
        if (!scene->IsEntityValid(entity)) {
          continue;
        }
        auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
        if (!sorghum) {
          continue;
        }

        if (reseed_on_reset) {
          sorghum->seed = base_seed + seed_offset++;
        }
        sorghum->target_gdd = 0.0f;
        sorghum->growth_model.Reset();
        sorghum->ClearGeometryEntities();
      }
    }

    simulation_day_of_year = NormalizeDayOfYear(static_cast<float>(std::clamp(season_start_day, 0, 364)));
  };

  const auto left_ctrl_state = EditorLayer::GetKey(GLFW_KEY_LEFT_CONTROL);
  const auto right_ctrl_state = EditorLayer::GetKey(GLFW_KEY_RIGHT_CONTROL);
  const bool ctrl_down =
      left_ctrl_state == Input::KeyActionType::Hold || left_ctrl_state == Input::KeyActionType::Press ||
      right_ctrl_state == Input::KeyActionType::Hold || right_ctrl_state == Input::KeyActionType::Press;

  if (ctrl_down) {
    if (EditorLayer::GetKey(GLFW_KEY_F) == Input::KeyActionType::Press) {
      auto_grow = !auto_grow;
      if (auto_grow) {
        fps_failsafe_tripped_ = false;
        last_failsafe_fps_ = 0.0f;
      }
    }
    if (EditorLayer::GetKey(GLFW_KEY_W) == Input::KeyActionType::Press) {
      auto& app = layer.GetApplication();
      if (app.IsPlaying()) {
        app.Stop();
      }
      auto_grow = false;
      reset_all_lsystems();
    }
  }

  const auto scene = layer.GetScene();
  const bool has_sorghum = scene && scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>() != nullptr;
  const std::string window_title = has_sorghum ? "Sorghum Genotype Lab" : layer.GetLayerName();
  bool open = layer.enable_inspection;
  if (!ImGui::Begin(window_title.c_str(), &open)) {
    ImGui::End();
    layer.enable_inspection = open;
    return false;
  }

  struct DescriptorGroup {
    std::shared_ptr<SorghumLSDescriptor> descriptor;
    size_t plant_count = 0;
  };
  std::vector<DescriptorGroup> descriptor_groups;
  std::unordered_map<SorghumLSDescriptor*, size_t> descriptor_group_indices;
  if (scene) {
    if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>()) {
      for (const auto& entity : *owners) {
        if (!scene->IsEntityValid(entity)) {
          continue;
        }
        const auto plant = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
        const auto descriptor = plant ? plant->descriptor_ref.Get<SorghumLSDescriptor>() : nullptr;
        if (!descriptor) {
          continue;
        }
        const auto [it, inserted] = descriptor_group_indices.emplace(descriptor.get(), descriptor_groups.size());
        if (inserted) {
          descriptor_groups.push_back({descriptor, 0});
        }
        descriptor_groups[it->second].plant_count++;
      }
    }
  }
  std::sort(descriptor_groups.begin(), descriptor_groups.end(), [](const DescriptorGroup& a, const DescriptorGroup& b) {
    return a.descriptor->GetTitle() < b.descriptor->GetTitle();
  });

  if (!descriptor_groups.empty()) {
    size_t total_plants = 0;
    bool live_preview = true;
    bool preview_whole_genotype = true;
    for (const auto& group : descriptor_groups) {
      total_plants += group.plant_count;
      live_preview &= group.descriptor->live_preview;
      preview_whole_genotype &= !group.descriptor->live_preview_representative_only;
    }

    ImGui::Text("%zu plants | %zu genotype descriptors", total_plants, descriptor_groups.size());
    ImGui::TextDisabled("One CPU geometry path feeds rasterization, Vulkan ray tracing, and ray query.");

    if (const auto camera = scene->main_camera.Get<Camera>()) {
      int render_mode = static_cast<int>(camera->camera_render_mode);
      const char* render_modes[] = {"Rasterization (interactive)", "Vulkan Ray Tracing", "Vulkan Ray Query"};
      if (ImGui::Combo("Viewport", &render_mode, render_modes, IM_ARRAYSIZE(render_modes))) {
        camera->camera_render_mode = Camera::NormalizeCameraRenderMode(static_cast<uint32_t>(render_mode));
      }
      const auto resolved_mode = Camera::ResolveCameraRenderMode(camera->camera_render_mode);
      ImGui::TextDisabled("Active: %s | RT %s | Ray Query %s", Camera::GetCameraRenderModeName(resolved_mode),
                          Platform::RayTracingEnabled() ? "available" : "unavailable",
                          Platform::RayQueryEnabled() ? "available" : "unavailable");
    }

    if (ImGui::Checkbox("Live preview", &live_preview)) {
      for (const auto& group : descriptor_groups) {
        group.descriptor->live_preview = live_preview;
        group.descriptor->SetUnsaved();
      }
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("Update whole genotype while dragging", &preview_whole_genotype)) {
      for (const auto& group : descriptor_groups) {
        group.descriptor->live_preview_representative_only = !preview_whole_genotype;
        group.descriptor->SetUnsaved();
      }
    }
    if (preview_whole_genotype) {
      ImGui::TextDisabled("Dragging previews every plant; releasing publishes full-quality geometry.");
    } else {
      ImGui::TextDisabled("Dragging previews one representative smoothly; releasing rebuilds every plant.");
    }

    for (const auto& group : descriptor_groups) {
      ImGui::PushID(group.descriptor.get());
      const auto& descriptor = *group.descriptor;
      ImGui::SeparatorText(descriptor.GetTitle().c_str());
      ImGui::Text("%zu plants | phytomers %.1f | tillers %.1f | panicle %s", group.plant_count,
                  descriptor.total_phytomer_count.mean, descriptor.tiller_count.mean,
                  descriptor.enable_panicle ? "enabled" : "disabled");
      if (context.editor_layer && ImGui::Button("Edit parameters")) {
        context.editor_layer->OpenAssetInspector(group.descriptor);
      }
      ImGui::SameLine();
      if (ImGui::Button("Rebuild genotype")) {
        layer.RegenerateSorghumDescriptor(descriptor, false, false);
      }
      ImGui::PopID();
    }

    if (ImGui::Button("Rebuild all genotypes")) {
      for (const auto& group : descriptor_groups) {
        layer.RegenerateSorghumDescriptor(*group.descriptor, false, false);
      }
    }
    ImGui::Separator();
  }

  if (ImGui::TreeNode("Advanced L-system Simulation")) {
    if (ImGui::Checkbox("Auto-Grow (Ctrl+F)", &auto_grow) && auto_grow) {
      fps_failsafe_tripped_ = false;
      last_failsafe_fps_ = 0.0f;
    }
    ImGui::Checkbox("Enable Auto-Grow FPS Failsafe", &auto_grow_fps_failsafe_enabled);
    ImGui::TextDisabled("Thermal rates are descriptor-owned and sampled per plant.");
    int growth_step_cap = static_cast<int>(sorghum_growth_step_cap_per_update);
    if (ImGui::DragInt("Sorghum Growth Steps/Plant/Frame", &growth_step_cap, 1.0f, 0, 1000)) {
      sorghum_growth_step_cap_per_update = static_cast<uint32_t>(std::max(0, growth_step_cap));
    }
    ImGui::TextDisabled("0 means uncapped growth for baseline/profiling comparisons.");

    ImGui::SeparatorText("Seasonality");
    ImGui::Checkbox("Enable Calendar Seasonality", &seasonality_enabled);
    if (ImGui::DragInt("Season Start Day", &season_start_day, 1.0f, 0, 364)) {
      season_start_day = std::clamp(season_start_day, 0, 364);
      simulation_day_of_year = NormalizeDayOfYear(static_cast<float>(season_start_day));
    }
    ImGui::DragInt("Season End Day", &season_end_day, 1.0f, 0, 364);
    ImGui::DragFloat("Calendar Days/sec (Chronology)", &chronological_days_per_second, 0.25f, 0.0f, 365.0f, "%.2f");
    ImGui::DragFloat("Max Auto-Grow Delta Time", &auto_grow_max_delta_time, 0.001f, 0.0f, 1.0f, "%.3f");
    if (ImGui::DragFloat("Simulation Day Of Year", &simulation_day_of_year, 0.25f, 0.0f, 364.999f, "%.2f")) {
      simulation_day_of_year = NormalizeDayOfYear(simulation_day_of_year);
    }
    const bool inspector_active_season = IsInActiveSeason(simulation_day_of_year, season_start_day, season_end_day);
    ImGui::Text("Season State: %s", inspector_active_season ? "Active" : "Dormant");

    ImGui::Checkbox("Reseed on Reset (Ctrl+W when stopped)", &reseed_on_reset);
    if (fps_failsafe_tripped_) {
      ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "Auto-grow stopped by 1 FPS failsafe (last: %.2f FPS).",
                         last_failsafe_fps_);
      ImGui::TextDisabled("Re-enable Auto-Grow to resume growth.");
    }

    if (ImGui::Checkbox("Scots Pine Stem-Only Mode", &pine_stem_only_mode)) {
      ApplyPineStemOnlyMode(layer.GetScene(), pine_stem_only_mode, true);
    }

    if (ImGui::Checkbox("Enable Scene/Plant View Tint", &scene_plant_view_tint_enabled)) {
      ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
      RebuildAllPlantGeometry(layer.GetScene());
    }
    if (!scene_plant_view_tint_enabled) {
      ImGui::TextDisabled("Tint disabled: effective mode forced to Shaded.");
    }

    {
      const char* color_mode_items[] = {
          "Shaded",       "By Type", "By Instance", "By Node", "Needle Lignification", "Needle Stripe Proxy",
          "Needle Sheath"};
      if (ImGui::Combo("Plant Color Mode", &tassel_color_mode, color_mode_items, IM_ARRAYSIZE(color_mode_items))) {
        tassel_color_mode = ClampColorModeIndex(tassel_color_mode);
        ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
        RebuildAllPlantGeometry(layer.GetScene());
      }
    }

    if (ImGui::Button("Reset Plant Color View (Shaded)")) {
      tassel_color_mode = 0;
      scene_plant_view_tint_enabled = false;
      ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
      RebuildAllPlantGeometry(layer.GetScene());
    }

    if (ImGui::Button("Reset All LSystems (Ctrl+W)")) {
      reset_all_lsystems();
    }

    ImGui::Separator();
    ImGui::Checkbox("Enable LSystem Profiling", &profiling_enabled);
    ImGui::DragInt("Profile History Size", &profiling_history_size, 1.0f, 30, 4000);

    static char export_path_buffer[260] = "lsystem_profile.csv";
    static std::string last_loaded_export_path;
    if (last_loaded_export_path != profiling_export_path && profiling_export_path.size() < sizeof(export_path_buffer)) {
      std::snprintf(export_path_buffer, sizeof(export_path_buffer), "%s", profiling_export_path.c_str());
      last_loaded_export_path = profiling_export_path;
    }
    if (ImGui::InputText("Profile CSV Path", export_path_buffer, sizeof(export_path_buffer))) {
      profiling_export_path = export_path_buffer;
      last_loaded_export_path = profiling_export_path;
    }

    if (ImGui::Button("Export Profile CSV")) {
      layer.ExportProfileCsv(profiling_export_path);
    }
    if (ImGui::Button("Clear Profile History")) {
      profiling_history.clear();
    }

    if (profiling_enabled) {
      const double avg_update_ms = AverageProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
        return f.update_ms;
      });
      const double avg_grow_ms = AverageProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
        return f.grow_ms;
      });
      const double avg_rebuild_ms = AverageProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
        return f.rebuild_ms;
      });

      const double max_update_ms = MaxProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
        return f.update_ms;
      });
      const double max_grow_ms = MaxProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
        return f.grow_ms;
      });
      const double max_rebuild_ms = MaxProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
        return f.rebuild_ms;
      });

      ImGui::SeparatorText("LSystem Profiling (Rolling)");
      ImGui::Text("History Frames: %d", static_cast<int>(profiling_history.size()));
      ImGui::Text("Last Update: %.3f ms", last_profile_frame.update_ms);
      ImGui::Text("Last Grow: %.3f ms", last_profile_frame.grow_ms);
      ImGui::Text("Last Growth Rules/Topology/Sort: %.3f / %.3f / %.3f ms", last_profile_frame.growth_rules_ms,
                  last_profile_frame.topology_rules_ms, last_profile_frame.sort_lists_ms);
      ImGui::Text("Last Node/Propagate/Scan: %.3f / %.3f / %.3f ms", last_profile_frame.update_node_info_ms,
                  last_profile_frame.propagate_geometry_ms, last_profile_frame.topology_scan_ms);
      ImGui::Text("Last Rebuild: %.3f ms", last_profile_frame.rebuild_ms);
      ImGui::Text("Last Internodes/Leaf Spline/Leaf Mesh/Upload: %.3f / %.3f / %.3f / %.3f ms",
                  last_profile_frame.internode_rebuild_ms, last_profile_frame.leaf_spline_ms,
                  last_profile_frame.leaf_mesh_ms, last_profile_frame.mesh_upload_ms);
      ImGui::Text("Last Pines/Sorghums: %u / %u", last_profile_frame.pine_count, last_profile_frame.sorghum_count);
      ImGui::Text("Last Growth Steps: %u", last_profile_frame.growth_steps);
      ImGui::Text("Last Nodes/Internodes/Needles: %u / %u / %u", last_profile_frame.node_count,
                  last_profile_frame.internode_count, last_profile_frame.needle_count);
      ImGui::Text("Last Leaves: %u (live %u)", last_profile_frame.leaf_count, last_profile_frame.live_leaf_count);
      ImGui::Text("Last Invalid Instances: %u", last_profile_frame.invalid_instance_count);

      ImGui::SeparatorText("Averages / Maxima");
      ImGui::Text("Update ms avg/max: %.3f / %.3f", avg_update_ms, max_update_ms);
      ImGui::Text("Grow ms avg/max: %.3f / %.3f", avg_grow_ms, max_grow_ms);
      ImGui::Text("Rebuild ms avg/max: %.3f / %.3f", avg_rebuild_ms, max_rebuild_ms);
    }
    ImGui::TreePop();
  }
  ImGui::End();
  layer.enable_inspection = open;
  return false;
}

void l_system_package::SerializeLSystemLayer(YAML::Emitter& out, const LSystemLayer& target) {
  out << YAML::Key << "auto_grow" << YAML::Value << target.auto_grow;
  out << YAML::Key << "auto_grow_fps_failsafe_enabled" << YAML::Value << target.auto_grow_fps_failsafe_enabled;
  out << YAML::Key << "seasonality_enabled" << YAML::Value << target.seasonality_enabled;
  out << YAML::Key << "season_start_day" << YAML::Value << target.season_start_day;
  out << YAML::Key << "season_end_day" << YAML::Value << target.season_end_day;
  out << YAML::Key << "chronological_days_per_second" << YAML::Value << target.chronological_days_per_second;
  out << YAML::Key << "auto_grow_max_delta_time" << YAML::Value << target.auto_grow_max_delta_time;
  out << YAML::Key << "simulation_day_of_year" << YAML::Value << NormalizeDayOfYear(target.simulation_day_of_year);
  out << YAML::Key << "sorghum_growth_step_cap_per_update" << YAML::Value << target.sorghum_growth_step_cap_per_update;
  out << YAML::Key << "reseed_on_reset" << YAML::Value << target.reseed_on_reset;
  out << YAML::Key << "tassel_color_mode" << YAML::Value << target.tassel_color_mode;
  out << YAML::Key << "scene_plant_view_tint_enabled" << YAML::Value << target.scene_plant_view_tint_enabled;
  out << YAML::Key << "pine_stem_only_mode" << YAML::Value << target.pine_stem_only_mode;
  out << YAML::Key << "profiling_enabled" << YAML::Value << target.profiling_enabled;
  out << YAML::Key << "profiling_history_size" << YAML::Value << target.profiling_history_size;
  out << YAML::Key << "profiling_export_path" << YAML::Value << target.profiling_export_path;
}

void l_system_package::DeserializeLSystemLayer(const YAML::Node& in, LSystemLayer& target) {
  if (in["auto_grow"]) {
    target.auto_grow = in["auto_grow"].as<bool>();
  }
  if (in["auto_grow_fps_failsafe_enabled"]) {
    target.auto_grow_fps_failsafe_enabled = in["auto_grow_fps_failsafe_enabled"].as<bool>();
  }
  if (in["seasonality_enabled"]) {
    target.seasonality_enabled = in["seasonality_enabled"].as<bool>();
  }
  if (in["season_start_day"]) {
    target.season_start_day = std::clamp(in["season_start_day"].as<int>(), 0, 364);
  }
  if (in["season_end_day"]) {
    target.season_end_day = std::clamp(in["season_end_day"].as<int>(), 0, 364);
  }
  if (in["chronological_days_per_second"]) {
    target.chronological_days_per_second = std::max(0.0f, in["chronological_days_per_second"].as<float>());
  }
  if (in["auto_grow_max_delta_time"]) {
    target.auto_grow_max_delta_time = std::max(0.0f, in["auto_grow_max_delta_time"].as<float>());
  }
  target.simulation_day_of_year = NormalizeDayOfYear(static_cast<float>(target.season_start_day));
  if (in["simulation_day_of_year"]) {
    target.simulation_day_of_year = NormalizeDayOfYear(in["simulation_day_of_year"].as<float>());
  }
  if (in["sorghum_growth_step_cap_per_update"]) {
    target.sorghum_growth_step_cap_per_update =
        static_cast<uint32_t>(std::max(0, in["sorghum_growth_step_cap_per_update"].as<int>()));
  }
  if (in["reseed_on_reset"]) {
    target.reseed_on_reset = in["reseed_on_reset"].as<bool>();
  }
  if (in["tassel_color_mode"]) {
    target.tassel_color_mode = ClampColorModeIndex(in["tassel_color_mode"].as<int>());
  }
  if (in["scene_plant_view_tint_enabled"]) {
    target.scene_plant_view_tint_enabled = in["scene_plant_view_tint_enabled"].as<bool>();
  }
  if (in["pine_stem_only_mode"]) {
    target.pine_stem_only_mode = in["pine_stem_only_mode"].as<bool>();
  }
  if (in["profiling_enabled"]) {
    target.profiling_enabled = in["profiling_enabled"].as<bool>();
  }
  if (in["profiling_history_size"]) {
    target.profiling_history_size = std::max(30, in["profiling_history_size"].as<int>());
  }
  if (in["profiling_export_path"]) {
    target.profiling_export_path = in["profiling_export_path"].as<std::string>();
  }
}
