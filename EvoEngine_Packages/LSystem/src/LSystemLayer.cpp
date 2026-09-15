#include "LSystemLayer.hpp"
#include "LSystemSimulation.hpp"

#include "Application.hpp"
#include "LSystemSerializationAdapters.hpp"
#include "Scene.hpp"
#include "ScotsPine.hpp"
#include "ScotsPineDescriptor.hpp"
#include "Times.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <random>

using namespace l_system_package;
using namespace l_system_package::simulation;
using namespace evo_engine;

namespace {

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

  return descriptor->SampleTargetGdd(pine.seed);
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

void LSystemLayer::OnCreate() {
  simulation_day_of_year = NormalizeDayOfYear(static_cast<float>(std::clamp(season_start_day, 0, 364)));
  ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
  ApplyPineStemOnlyMode(GetScene(), pine_stem_only_mode, true);
}

void LSystemLayer::OnDestroy() {
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
  std::ofstream out(path, std::ios::trunc);
  if (!out.is_open()) {
    return;
  }

  out << "frame,update_ms,grow_ms,rebuild_ms,pines,growth_steps,nodes,internodes,needles,invalid_instances\n";
  for (size_t i = 0; i < profiling_history.size(); i++) {
    const auto& f = profiling_history[i];
    out << i << "," << std::fixed << std::setprecision(4) << f.update_ms << "," << f.grow_ms << "," << f.rebuild_ms
        << "," << f.pine_count << "," << f.growth_steps << "," << f.node_count << "," << f.internode_count << ","
        << f.needle_count << "," << f.invalid_instance_count << "\n";
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
  if (dt > 0.0f) {
    const float fps = 1.0f / dt;
    if (fps < kAutoGrowFailsafeMinFps) {
      auto_grow = false;
      fps_failsafe_tripped_ = true;
      last_failsafe_fps_ = fps;
      return;
    }
  }

  const float delta_days = std::max(0.0f, chronological_days_per_second) * std::max(0.0f, dt);
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
        frame.rebuild_ms += pine->last_rebuild_seconds * 1000.0;
        frame.growth_steps += pine->growth_model.last_growth_steps;
        frame.node_count += pine->last_node_count;
        frame.internode_count += pine->last_internode_count;
        frame.needle_count += pine->last_needle_count;
        frame.invalid_instance_count += pine->last_invalid_instance_count;
      }
    }
  }

  if (profiling_enabled) {
    frame.update_ms = (times.Now() - update_start) * 1000.0;
    last_profile_frame = frame;
    PushProfileFrame(frame);
  }
}

void l_system_package::SerializeLSystemLayer(YAML::Emitter& out, const LSystemLayer& target) {
  out << YAML::Key << "auto_grow" << YAML::Value << target.auto_grow;
  out << YAML::Key << "seasonality_enabled" << YAML::Value << target.seasonality_enabled;
  out << YAML::Key << "season_start_day" << YAML::Value << target.season_start_day;
  out << YAML::Key << "season_end_day" << YAML::Value << target.season_end_day;
  out << YAML::Key << "chronological_days_per_second" << YAML::Value << target.chronological_days_per_second;
  out << YAML::Key << "simulation_day_of_year" << YAML::Value << NormalizeDayOfYear(target.simulation_day_of_year);
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
  target.simulation_day_of_year = NormalizeDayOfYear(static_cast<float>(target.season_start_day));
  if (in["simulation_day_of_year"]) {
    target.simulation_day_of_year = NormalizeDayOfYear(in["simulation_day_of_year"].as<float>());
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
