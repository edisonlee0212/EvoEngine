#include "LSystemLayer.hpp"
#include "ClassRegistry.hpp"
#include "EditorLayer.hpp"
#include "LSystemDescriptor.hpp"
#include "MaizeTassel.hpp"
#include "MaizeTasselDescriptor.hpp"
#include "ScotsPine.hpp"
#include "ScotsPineDescriptor.hpp"
#include "Scene.hpp"
#include "Application.hpp"
#include "Times.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <random>

#ifdef LSYSTEM_GPU_PIPELINE
#include "Core/Math/Transform.hpp"
#include "Core/Console.hpp"
#include "Layers/RenderLayer.hpp"
#include "Platform/Platform.hpp"
#include "Rendering/Platform/GraphicsPipeline.hpp"
#include "Rendering/Platform/GraphicsResources.hpp"
#include "Rendering/Platform/Shader.hpp"
#include "gpu/LSystemGPUEngine.hpp"
#include <filesystem>
#endif

using namespace l_system_plugin;
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

float SampleDescriptorTargetGdd(MaizeTassel& tassel) {
  const auto descriptor = tassel.descriptor_ref.Get<MaizeTasselDescriptor>();
  if (!descriptor) {
    return -1.0f;
  }

  std::mt19937 rng(tassel.seed);
  return std::max(0.0f, SampleDistribution(descriptor->target_gdd, rng));
}

float SampleDescriptorTargetGddForPine(ScotsPine& pine) {
  auto descriptor = pine.descriptor_ref.Get<ScotsPineDescriptor>();
  if (pine.enable_repot_profile_switch &&
      pine.growth_model.IsPostRepotProfileActive()) {
    if (const auto post_descriptor =
            pine.post_repot_descriptor_ref.Get<ScotsPineDescriptor>()) {
      descriptor = post_descriptor;
    }
  }
  if (!descriptor) {
    return -1.0f;
  }
  std::mt19937 rng(pine.seed);
  return std::max(0.0f, SampleDistribution(descriptor->target_gdd, rng));
}

float NormalizeDayOfYear(float day) {
  if (!std::isfinite(day)) return 0.0f;
  day = std::fmod(day, 365.0f);
  if (day < 0.0f) day += 365.0f;
  return day;
}

bool IsInActiveSeason(const float simulation_day_of_year,
                      const int season_start_day,
                      const int season_end_day) {
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

int ResolveEffectiveColorMode(const int selected_mode,
                              const bool scene_plant_view_tint_enabled) {
  return scene_plant_view_tint_enabled ? ClampColorModeIndex(selected_mode) : 0;
}

void ApplyGlobalPlantColorMode(const int selected_mode,
                               const bool scene_plant_view_tint_enabled) {
  const int effective_mode =
      ResolveEffectiveColorMode(selected_mode, scene_plant_view_tint_enabled);
  const int maize_mode = std::clamp(effective_mode, 0, 3);
  const int pine_mode = ClampColorModeIndex(effective_mode);
  MaizeTassel::SetGlobalColorMode(static_cast<MaizeTassel::ColorMode>(maize_mode));
  ScotsPine::SetGlobalColorMode(static_cast<ScotsPine::ColorMode>(pine_mode));
}

void RebuildAllPlantGeometry(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return;
  }

  if (const auto* tassel_entities_ptr =
          scene->UnsafeGetPrivateComponentOwnersList<MaizeTassel>()) {
    const std::vector<Entity> tassel_entities = *tassel_entities_ptr;
    for (const auto& entity : tassel_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      auto tassel = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
      if (tassel) {
        tassel->RebuildGeometry();
      }
    }
  }

  if (const auto* pine_entities_ptr =
          scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>()) {
    const std::vector<Entity> pine_entities = *pine_entities_ptr;
    for (const auto& entity : pine_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
      if (pine) {
        pine->RebuildGeometry();
      }
    }
  }
}

struct PineTemporalSample {
  float gdd_per_day = 2.0f;
  int season_start_day = 60;
  int season_end_day = 334;
};

PineTemporalSample SamplePineTemporalParameters(const ScotsPine& pine) {
  PineTemporalSample sample;
  auto descriptor = pine.descriptor_ref.Get<ScotsPineDescriptor>();
  if (pine.enable_repot_profile_switch &&
      pine.growth_model.IsPostRepotProfileActive()) {
    if (const auto post_descriptor =
            pine.post_repot_descriptor_ref.Get<ScotsPineDescriptor>()) {
      descriptor = post_descriptor;
    }
  }
  if (!descriptor) {
    return sample;
  }

  std::mt19937 rng(static_cast<uint32_t>(pine.seed) ^ 0x5f3759dfu);
  sample.gdd_per_day = std::max(0.0f, SampleDistribution(descriptor->gdd_per_day, rng));

  const auto sample_day = [&](const evo_engine::SingleDistribution<float>& distribution) {
    const float sampled_day =
        std::clamp(SampleDistribution(distribution, rng), 0.0f, 365.0f);
    return static_cast<int>(
        std::floor(NormalizeDayOfYear(std::round(sampled_day))));
  };
  sample.season_start_day = sample_day(descriptor->growing_season_start_day);
  sample.season_end_day = sample_day(descriptor->growing_season_end_day);
  return sample;
}
}  // namespace

AssetRegistration<LSystemDescriptor> lsys_desc_registry("LSystemDescriptor", {".lsys"});
AssetRegistration<MaizeTasselDescriptor> maize_tassel_desc_registry("MaizeTasselDescriptor", {".mtassel"});
PrivateComponentRegistration<MaizeTassel> maize_tassel_component_registry("MaizeTassel");
AssetRegistration<ScotsPineDescriptor> scots_pine_desc_registry("ScotsPineDescriptor", {".spine"});
PrivateComponentRegistration<ScotsPine> scots_pine_component_registry("ScotsPine");

void LSystemLayer::OnCreate() {
  simulation_day_of_year =
      NormalizeDayOfYear(static_cast<float>(std::clamp(season_start_day, 0, 364)));

  ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);

#ifdef LSYSTEM_GPU_PIPELINE
  // -------------------------------------------------------------------------
  // Phase 1b: register the tassel_internode mesh-shader pipeline.
  //
  // Mirrors the EcoSysLab DsKineticVoronoiMeshing segment-meshlet pattern:
  //   * set 0 = RenderLayer::per_frame_layout (camera matrices, materials)
  //   * set 1 = tassel_internode_layout      (TasselInternodeInstance SSBO)
  //   * push constant = TasselInternodePushConstant (uint count)
  //
  // No draw call is recorded yet — Phase 1b-final wires the indirect
  // draw + VMA buffer upload. This step proves the SPIR-V cross-compiles
  // and the descriptor / push-constant ranges validate at PSO creation.
  // -------------------------------------------------------------------------
  if (Platform::Constants::support_mesh_shader) {
    if (!tassel_internode_layout) {
      tassel_internode_layout = std::make_shared<DescriptorSetLayout>();
      // (set=1, binding=0) — the TasselInternodeBuffer SSBO read by both
      // tassel_internode.task and tassel_internode.mesh.
      tassel_internode_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
      tassel_internode_layout->Initialize();
    }

    // Hand the layout to the GPU engine so it can lazily create per-instance
    // descriptor sets when UploadTasselInternodes is first called for an
    // instance. Calling this before the engine has any instances is safe.
    gpu::LSystemGPUEngine::Get().SetTasselInternodeDescriptorLayout(tassel_internode_layout);

    if (!tassel_internode_pipeline) {
      tassel_internode_pipeline = std::make_shared<GraphicsPipeline>();
      tassel_internode_pipeline->task_shader = Shader::CreateTemporary(
          ShaderType::Task, Platform::GetShaderGlobalDefines(),
          std::filesystem::path("./LSystemResources") / "Shaders/Graphics/Mesh/Tassel/tassel_internode.task");
      tassel_internode_pipeline->mesh_shader = Shader::CreateTemporary(
          ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
          std::filesystem::path("./LSystemResources") / "Shaders/Graphics/Mesh/Tassel/tassel_internode.mesh");
      tassel_internode_pipeline->fragment_shader = Shader::CreateTemporary(
          ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
          std::filesystem::path("./LSystemResources") / "Shaders/Graphics/Fragment/Tassel/tassel_internode.frag");
      tassel_internode_pipeline->geometry_type = GeometryType::Mesh;
      tassel_internode_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
      tassel_internode_pipeline->descriptor_set_layouts.emplace_back(tassel_internode_layout);
      tassel_internode_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
      tassel_internode_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
      // 2 G-buffer color attachments — must match outNormal / outMaterial in tassel_internode.frag.
      tassel_internode_pipeline->color_attachment_formats = {2, Platform::Constants::g_buffer_color};
      auto& push_constant_range = tassel_internode_pipeline->push_constant_ranges.emplace_back();
      push_constant_range.size = sizeof(TasselInternodePushConstant);
      push_constant_range.offset = 0;
      push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
      tassel_internode_pipeline->Initialize();
    }
  }

#endif
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

  out << "frame,update_ms,grow_ms,rebuild_ms,"
      << "apply_growth_rules_ms,apply_topology_rules_ms,sort_lists_ms,update_node_info_ms,"
      << "propagate_geometry_ms,topology_scan_ms,"
      << "rebuild_internode_collect_ms,rebuild_internode_upload_ms,rebuild_spikelet_collect_ms,rebuild_spikelet_upload_ms,"
      << "tassels,growth_steps,nodes,internodes,spikelets,invalid_instances\n";
  for (size_t i = 0; i < profiling_history.size(); i++) {
    const auto& f = profiling_history[i];
    out << i << ","
        << std::fixed << std::setprecision(4)
        << f.update_ms << ","
        << f.grow_ms << ","
        << f.rebuild_ms << ","
        << f.apply_growth_rules_ms << ","
        << f.apply_topology_rules_ms << ","
        << f.sort_lists_ms << ","
        << f.update_node_info_ms << ","
        << f.propagate_geometry_ms << ","
        << f.topology_scan_ms << ","
        << f.rebuild_internode_collect_ms << ","
        << f.rebuild_internode_upload_ms << ","
        << f.rebuild_spikelet_collect_ms << ","
        << f.rebuild_spikelet_upload_ms << ","
        << f.tassel_count << ","
        << f.growth_steps << ","
        << f.node_count << ","
        << f.internode_count << ","
        << f.spikelet_count << ","
        << f.invalid_instance_count << "\n";
  }
}

void LSystemLayer::Update() {
  const double update_start = Times::Now();
  ProfileFrame frame{};

#ifdef LSYSTEM_GPU_PIPELINE
  // -------------------------------------------------------------------------
  // Phase 1b-final: register one deferred-rendering callback per frame that
  // draws every MaizeTassel that has internodes uploaded to the GPU
  // engine. Registration happens unconditionally — auto_grow only gates
  // CPU-side growth ticking, not draw recording. The engine clears the
  // callback list every frame, so this MUST run each frame.
  if (tassel_internode_pipeline && tassel_internode_pipeline->Initialized()) {
    if (const auto render_layer = Application::GetLayer<RenderLayer>()) {
      // Snapshot tassel entities before registration so the closure does
      // not iterate a live scene structure during command-buffer
      // recording (which runs deeper in the engine after Update).
      struct DrawEntry {
        uint32_t gpu_instance_id;
        glm::mat4 model;
      };
      std::vector<DrawEntry> live_draws;
      live_draws.reserve(8);
      if (const auto scene = GetScene()) {
        if (const auto* tassels = scene->UnsafeGetPrivateComponentOwnersList<MaizeTassel>()) {
          for (const auto& entity : *tassels) {
            if (!scene->IsEntityValid(entity)) continue;
            const auto comp = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
            if (!comp) continue;
            if (comp->gpu_instance_id == 0u) continue;
            if (gpu::LSystemGPUEngine::Get().GetTasselInternodeCount(comp->gpu_instance_id) == 0u) continue;
            const auto gt = scene->GetDataComponent<GlobalTransform>(entity);
            live_draws.push_back({comp->gpu_instance_id, gt.value});
          }
        }
      }

      if (!live_draws.empty()) {
        const auto pipeline = tassel_internode_pipeline;  // shared_ptr by value into closure
        const std::vector<DrawEntry> draws = std::move(live_draws);
        render_layer->DeferredRenderingAllCameras(
            [pipeline, draws](VkCommandBuffer vk_command_buffer,
                              const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                              const RenderLayer::DeferredRenderingView& view) -> uint32_t {
              auto& engine = gpu::LSystemGPUEngine::Get();
              const uint32_t task_work_group_invocations =
                  Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

              pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
              pipeline->states.SetViewportScissor(view.viewport);
              pipeline->states.polygon_mode = VK_POLYGON_MODE_FILL;
              pipeline->states.ApplyAllStates(vk_command_buffer);
              pipeline->Bind(vk_command_buffer);
              pipeline->BindDescriptorSet(
                  vk_command_buffer, 0,
                  RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());

              uint32_t total_prims = 0;
              for (const auto& draw : draws) {
                const VkDescriptorSet set = engine.GetTasselInternodeDescriptorSet(draw.gpu_instance_id);
                if (set == VK_NULL_HANDLE) continue;
                const uint32_t count = engine.GetTasselInternodeCount(draw.gpu_instance_id);
                if (count == 0u) continue;

                pipeline->BindDescriptorSet(vk_command_buffer, 1, set);

                TasselInternodePushConstant pc{};
                pc.camera_index = view.camera_index;
                pc.tassel_internode_count = count;
                pc.model = draw.model;
                pipeline->PushConstant(vk_command_buffer, 0, pc);

                const uint32_t group_count = Platform::DivUp(count, task_work_group_invocations);
                vkCmdDrawMeshTasksEXT(vk_command_buffer, group_count, 1u, 1u);

                // 12 triangles per cylinder (RADIAL=6, two tris per sector).
                total_prims += count * 12u;
              }
              return total_prims;
            });
      }
    }
  }
#endif

  if (!auto_grow)
    return;

  const auto scene = GetScene();
  if (!scene)
    return;

  const float dt = static_cast<float>(Times::DeltaTime());
  if (dt > 0.0f) {
    const float fps = 1.0f / dt;
    if (fps < kAutoGrowFailsafeMinFps) {
      auto_grow = false;
      fps_failsafe_tripped_ = true;
      last_failsafe_fps_ = fps;
      return;
    }
  }

    // Calendar progression for season window, chronological aging, and
    // winter-count dormancy progression.
    const float delta_days =
      std::max(0.0f, chronological_days_per_second) * std::max(0.0f, dt);

    if (seasonality_enabled && delta_days > 0.0f) {
      simulation_day_of_year = NormalizeDayOfYear(simulation_day_of_year + delta_days);
    }

    const bool in_active_season =
      !seasonality_enabled ||
      IsInActiveSeason(simulation_day_of_year, season_start_day, season_end_day);

    // Phenology DOY-based GDD Curve (Sine wave approximating temperature)
    // Assume peak summer is DOY 200, coldest winter is DOY 20.
    float daily_gdd_rate = 0.0f;
    if (seasonality_enabled) {
      // rough approximation of a temperature curve
      const float doy_offset = simulation_day_of_year - 20.0f;
      const float temp_curve = -std::cos(doy_offset * 3.14159265359f / 182.5f);
      // Only accumulate GDD if temp_curve is positive (approx spring/summer)
      daily_gdd_rate = std::max(0.0f, temp_curve * 15.0f); // arbitrarily max 15 GDD/day
    } else {
      daily_gdd_rate = gdd_per_second; // fallback to continuous
    }

    const float raw_delta_gdd = seasonality_enabled ? (daily_gdd_rate * delta_days) : (gdd_per_second * dt);
    const float capped_delta_gdd = max_gdd_per_frame > 0.0f
                     ? std::min(raw_delta_gdd, max_gdd_per_frame)
                     : raw_delta_gdd;

    const float delta_gdd = in_active_season ? capped_delta_gdd : 0.0f;
    const float delta_years = seasonality_enabled ? (delta_days / 365.0f) : 0.0f;

  const auto* tassel_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<MaizeTassel>();

  // Copy before iterating: GrowToTargetGDD() can create geometry entities with new component types
  // (MeshRenderer, Particles) for the first time, causing p_owners_collections_list_ to reallocate
  // and invalidating the raw pointer returned by UnsafeGetPrivateComponentOwnersList.
  const std::vector<Entity> tassel_entities = tassel_entities_ptr ? *tassel_entities_ptr : std::vector<Entity>{};

  for (const auto& entity : tassel_entities) {
    if (!scene->IsEntityValid(entity))
      continue;
    auto tassel = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
    if (!tassel)
      continue;
    frame.tassel_count++;

    tassel->SetSeasonalChronologicalMode(seasonality_enabled);

    bool age_only_changed = false;
    if (seasonality_enabled && delta_years > 0.0f) {
      if (in_active_season) {
        if (!tassel->growth_model.IsInitialized()) {
          if (const auto descriptor =
                  tassel->descriptor_ref.Get<MaizeTasselDescriptor>()) {
            tassel->growth_model.Initialize(*descriptor, tassel->seed);
          }
        }
        tassel->growth_model.AdvanceChronologicalYears(delta_years);
      } else {
        age_only_changed = tassel->AdvanceChronologicalAging(delta_years);
      }
    }

    if (!in_active_season) {
      if (profiling_enabled && age_only_changed) {
        frame.rebuild_ms += tassel->last_rebuild_seconds * 1000.0;
        frame.node_count += tassel->last_node_count;
        frame.internode_count += tassel->last_internode_count;
        frame.spikelet_count += tassel->last_spikelet_count;
      }
      continue;
    }

    const float descriptor_target_gdd = SampleDescriptorTargetGdd(*tassel);
    const float next_target_gdd = std::max(0.0f, tassel->target_gdd + delta_gdd);
    tassel->target_gdd = descriptor_target_gdd >= 0.0f
      ? std::min(next_target_gdd, descriptor_target_gdd)
      : next_target_gdd;

    tassel->max_growth_steps_per_frame =
        static_cast<uint32_t>(std::max(0, max_growth_steps_per_frame));
    tassel->GrowToTargetGDD();

    if (profiling_enabled) {
      const auto& grow_profile = tassel->growth_model.last_grow_to_gdd_profile;
      frame.grow_ms += tassel->last_grow_seconds * 1000.0;
      frame.rebuild_ms += tassel->last_rebuild_seconds * 1000.0;
      frame.apply_growth_rules_ms += grow_profile.apply_growth_rules_seconds * 1000.0;
      frame.apply_topology_rules_ms += grow_profile.apply_topology_rules_seconds * 1000.0;
      frame.sort_lists_ms += grow_profile.sort_lists_seconds * 1000.0;
      frame.update_node_info_ms += grow_profile.update_node_info_seconds * 1000.0;
      frame.propagate_geometry_ms += grow_profile.propagate_geometry_seconds * 1000.0;
      frame.topology_scan_ms += grow_profile.topology_scan_seconds * 1000.0;
      frame.rebuild_internode_collect_ms +=
        tassel->last_rebuild_internode_collect_seconds * 1000.0;
      frame.rebuild_internode_upload_ms +=
        tassel->last_rebuild_internode_upload_seconds * 1000.0;
      frame.rebuild_spikelet_collect_ms +=
        tassel->last_rebuild_spikelet_collect_seconds * 1000.0;
      frame.rebuild_spikelet_upload_ms +=
        tassel->last_rebuild_spikelet_upload_seconds * 1000.0;
      frame.growth_steps += tassel->growth_model.last_growth_steps;
      frame.node_count += tassel->last_node_count;
      frame.internode_count += tassel->last_internode_count;
      frame.spikelet_count += tassel->last_spikelet_count;
      frame.invalid_instance_count += tassel->last_invalid_instance_count;
    }
  }

  // ---- Pine auto-grow ----
  // Pine vigor and calendar windows are sampled per plant from the
  // ScotsPineDescriptor. LSystemLayer provides only day progression speed.
  if (const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>()) {
    const std::vector<Entity> pine_entities = *pine_entities_ptr;
    for (const auto& entity : pine_entities) {
      if (!scene->IsEntityValid(entity))
        continue;
      auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
      if (!pine)
        continue;

      const PineTemporalSample pine_temporal =
          SamplePineTemporalParameters(*pine);
      const bool pine_in_active_season =
          !seasonality_enabled ||
          IsInActiveSeason(simulation_day_of_year,
                           pine_temporal.season_start_day,
                           pine_temporal.season_end_day);
      const float pine_raw_delta_gdd =
          std::max(0.0f, pine_temporal.gdd_per_day) * delta_days;
      const float pine_capped_delta_gdd = max_gdd_per_frame > 0.0f
          ? std::min(pine_raw_delta_gdd, max_gdd_per_frame)
          : pine_raw_delta_gdd;
      const float pine_delta_gdd =
          pine_in_active_season ? pine_capped_delta_gdd : 0.0f;

      const float pine_season_days =
          (pine_temporal.season_end_day >= pine_temporal.season_start_day)
              ? static_cast<float>(pine_temporal.season_end_day -
                                   pine_temporal.season_start_day + 1)
              : static_cast<float>(365 - pine_temporal.season_start_day +
                                   pine_temporal.season_end_day + 1);
      const float pine_season_length_years =
          std::max(1e-3f, pine_season_days / 365.0f);

      pine->SetSeasonalChronologicalMode(seasonality_enabled);

      if (seasonality_enabled && delta_years > 0.0f) {
        if (pine_in_active_season) {
          if (!pine->growth_model.IsInitialized()) {
            if (const auto descriptor =
                    pine->descriptor_ref.Get<ScotsPineDescriptor>()) {
              std::shared_ptr<ScotsPineDescriptor> post_descriptor = nullptr;
              float repot_switch_gdd = -1.0f;
              if (pine->enable_repot_profile_switch) {
                post_descriptor =
                    pine->post_repot_descriptor_ref.Get<ScotsPineDescriptor>();
                if (post_descriptor) {
                  repot_switch_gdd = std::max(0.0f, pine->repot_switch_gdd);
                }
              }
              pine->growth_model.Initialize(*descriptor, pine->seed,
                                            glm::vec3(0), kDefaultRootRotation,
                                            post_descriptor.get(), repot_switch_gdd);
            }
          }
          pine->growth_model.AdvanceChronologicalYears(delta_years);
        } else {
          pine->AdvanceChronologicalAging(delta_years);
        }
      }

      if (!pine_in_active_season) {
        // Even when dormant, keep the clock's in_active_season_ flag in sync
        // so the next dormant->active edge bumps the year exactly once.
        if (pine->growth_model.IsInitialized()) {
          pine->growth_model.graph.data.clock.SyncSeasonalState(
              seasonality_enabled, pine_in_active_season,
              pine_season_length_years);
        }
        continue;
      }

      // Forward seasonal state to the clock BEFORE growing. R0 (year
      // rollover) and R1 (active-season gate) read these fields when the
      // derivation engine fires production rules below.
      if (pine->growth_model.IsInitialized()) {
        pine->growth_model.graph.data.clock.SyncSeasonalState(
            seasonality_enabled, pine_in_active_season,
            pine_season_length_years);
      }

      const float descriptor_target_gdd = SampleDescriptorTargetGddForPine(*pine);
      const float next_target_gdd = std::max(0.0f, pine->target_gdd + pine_delta_gdd);
      pine->target_gdd = descriptor_target_gdd >= 0.0f
        ? std::min(next_target_gdd, descriptor_target_gdd)
        : next_target_gdd;

      pine->max_growth_steps_per_frame =
          static_cast<uint32_t>(std::max(0, max_growth_steps_per_frame));
      pine->GrowToTargetGDD();
    }
  }

  if (profiling_enabled) {
    frame.update_ms = (Times::Now() - update_start) * 1000.0;
    last_profile_frame = frame;
    PushProfileFrame(frame);
  }
}

void LSystemLayer::OnInspect(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer) {
  auto reset_all_tassels = [this]() {
    const auto scene = GetScene();
    if (!scene)
      return;

    const auto* tassel_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<MaizeTassel>();
    const std::vector<Entity> tassel_entities = tassel_entities_ptr ? *tassel_entities_ptr : std::vector<Entity>{};
    unsigned int base_seed = 0u;
    if (reseed_on_reset) {
      base_seed = static_cast<unsigned int>(
          std::chrono::steady_clock::now().time_since_epoch().count() & 0xFFFFFFFFu);
    }

    unsigned int seed_offset = 0u;
    for (const auto& entity : tassel_entities) {
      if (!scene->IsEntityValid(entity))
        continue;
      auto tassel = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
      if (!tassel)
        continue;

      if (reseed_on_reset) {
        tassel->seed = base_seed + seed_offset;
        seed_offset++;
      }

      tassel->target_gdd = 0.0f;
      tassel->GenerateGeometryEntities();
    }

    // ---- Pine reset (mirrors maize loop above) ----
    if (const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>()) {
      const std::vector<Entity> pine_entities = *pine_entities_ptr;
      for (const auto& entity : pine_entities) {
        if (!scene->IsEntityValid(entity))
          continue;
        auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
        if (!pine)
          continue;

        if (reseed_on_reset) {
          pine->seed = base_seed + seed_offset;
          seed_offset++;
        }

        pine->target_gdd = 0.0f;
        pine->GenerateGeometryEntities();
      }
    }

    auto_grow = false;
    simulation_day_of_year =
      NormalizeDayOfYear(static_cast<float>(std::clamp(season_start_day, 0, 364)));
  };

  // --- Keyboard shortcuts (polling, matching EcoSysLab pattern) ---
  if (EditorLayer::GetKey(GLFW_KEY_LEFT_CONTROL) == Input::KeyActionType::Hold ||
      EditorLayer::GetKey(GLFW_KEY_RIGHT_CONTROL) == Input::KeyActionType::Hold) {
    if (EditorLayer::GetKey(GLFW_KEY_F) == Input::KeyActionType::Press) {
      auto_grow = !auto_grow;
      if (auto_grow) {
        fps_failsafe_tripped_ = false;
        last_failsafe_fps_ = 0.0f;
      }
    }
    if (EditorLayer::GetKey(GLFW_KEY_W) == Input::KeyActionType::Press) {
      reset_all_tassels();
    }
  }

  // --- UI ---
  if (ImGui::Checkbox("Auto-Grow (Ctrl+F)", &auto_grow) && auto_grow) {
    fps_failsafe_tripped_ = false;
    last_failsafe_fps_ = 0.0f;
  }
  ImGui::DragFloat("Thermal GDD/sec (Vigor)", &gdd_per_second, 1.0f, 1.0f, 500.0f);
  ImGui::TextDisabled("Maize uses layer GDD/sec; Scots pine uses descriptor GDD/day.");
  ImGui::DragFloat("Max GDD/Frame (0=Unlimited)", &max_gdd_per_frame, 0.1f, 0.0f, 500.0f);
  ImGui::DragInt("Max Growth Steps/Frame (0=Unlimited)",
                 &max_growth_steps_per_frame,
                 1.0f,
                 0,
                 5000);
  ImGui::TextDisabled("Use both caps to smooth thermal growth after long frames.");

  ImGui::SeparatorText("Seasonality");
  ImGui::Checkbox("Enable Calendar Seasonality", &seasonality_enabled);
  if (ImGui::DragInt("Season Start Day", &season_start_day, 1.0f, 0, 364)) {
    season_start_day = std::clamp(season_start_day, 0, 364);
    simulation_day_of_year =
        NormalizeDayOfYear(static_cast<float>(season_start_day));
  }
  ImGui::DragInt("Season End Day", &season_end_day, 1.0f, 0, 364);
  ImGui::DragFloat("Calendar Days/sec (Chronology)", &chronological_days_per_second,
                   0.25f, 0.0f, 365.0f, "%.2f");
  ImGui::TextDisabled(
      "Thermal GDD drives development; calendar days drive season and aging.");
  if (ImGui::DragFloat("Simulation Day Of Year", &simulation_day_of_year,
                       0.25f, 0.0f, 364.999f, "%.2f")) {
    simulation_day_of_year = NormalizeDayOfYear(simulation_day_of_year);
  }
  const bool inspector_active_season =
      IsInActiveSeason(simulation_day_of_year, season_start_day, season_end_day);
  ImGui::Text("Season State: %s", inspector_active_season ? "Active" : "Dormant");

  ImGui::Checkbox("Reseed on Reset (Ctrl+W)", &reseed_on_reset);
  if (fps_failsafe_tripped_) {
    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f),
                       "Auto-grow stopped by 5 FPS failsafe (last: %.2f FPS).",
                       last_failsafe_fps_);
    ImGui::TextDisabled("Re-enable Auto-Grow to resume growth.");
  }

  if (ImGui::Checkbox("Enable Scene/Plant View Tint", &scene_plant_view_tint_enabled)) {
    ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
    RebuildAllPlantGeometry(GetScene());
  }
  if (!scene_plant_view_tint_enabled) {
    ImGui::TextDisabled("Tint disabled: effective mode forced to Shaded.");
  }

  {
    const char* color_mode_items[] = {
      "Shaded",
      "By Type",
      "By Instance",
      "By Node",
      "Pine: Needle Lignification",
      "Pine: Needle Stripe Proxy",
      "Pine: Needle Sheath"
    };
    if (ImGui::Combo("Plant Color Mode", &tassel_color_mode, color_mode_items, IM_ARRAYSIZE(color_mode_items))) {
      tassel_color_mode = ClampColorModeIndex(tassel_color_mode);
      ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
      RebuildAllPlantGeometry(GetScene());
    }
    ImGui::TextDisabled("Modes 4-6 are pine-only needle diagnostics.");
  }

  if (ImGui::Button("Reset All Tassels (Ctrl+W)")) {
    reset_all_tassels();
  }

  ImGui::Separator();
  ImGui::Checkbox("Enable LSystem Profiling", &profiling_enabled);
  ImGui::DragInt("Profile History Size", &profiling_history_size, 1.0f, 30, 4000);

  static char export_path_buffer[260] = "lsystem_profile.csv";
  static std::string last_loaded_export_path;
  if (last_loaded_export_path != profiling_export_path &&
      profiling_export_path.size() < sizeof(export_path_buffer)) {
    std::snprintf(export_path_buffer, sizeof(export_path_buffer), "%s", profiling_export_path.c_str());
    last_loaded_export_path = profiling_export_path;
  }
  if (ImGui::InputText("Profile CSV Path", export_path_buffer, sizeof(export_path_buffer))) {
    profiling_export_path = export_path_buffer;
    last_loaded_export_path = profiling_export_path;
  }

  if (ImGui::Button("Export Profile CSV")) {
    ExportProfileCsv(profiling_export_path);
  }

  if (ImGui::Button("Clear Profile History")) {
    profiling_history.clear();
  }

  if (profiling_enabled) {
    const double avg_update_ms = AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.update_ms;
    });
    const double avg_grow_ms = AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.grow_ms;
    });
    const double avg_rebuild_ms = AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.rebuild_ms;
    });
    const double avg_apply_growth_rules_ms = AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.apply_growth_rules_ms;
    });
    const double avg_apply_topology_rules_ms = AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.apply_topology_rules_ms;
    });
    const double avg_sort_lists_ms = AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.sort_lists_ms;
    });
    const double avg_propagate_geometry_ms = AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.propagate_geometry_ms;
    });
    const double avg_rebuild_internode_collect_ms =
        AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
          return f.rebuild_internode_collect_ms;
        });
    const double avg_rebuild_spikelet_collect_ms =
        AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
          return f.rebuild_spikelet_collect_ms;
        });

    const double max_update_ms = MaxProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.update_ms;
    });
    const double max_grow_ms = MaxProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.grow_ms;
    });
    const double max_rebuild_ms = MaxProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.rebuild_ms;
    });

    ImGui::SeparatorText("LSystem Profiling (Rolling)");
    ImGui::Text("History Frames: %d", static_cast<int>(profiling_history.size()));
    ImGui::Text("Last Update: %.3f ms", last_profile_frame.update_ms);
    ImGui::Text("Last Grow: %.3f ms", last_profile_frame.grow_ms);
    ImGui::Text("Last Rebuild: %.3f ms", last_profile_frame.rebuild_ms);
    ImGui::Text("Last Tassels: %u", last_profile_frame.tassel_count);
    ImGui::Text("Last Growth Steps: %u", last_profile_frame.growth_steps);
    ImGui::Text("Last Nodes/Internodes/Spikelets: %u / %u / %u",
                last_profile_frame.node_count,
                last_profile_frame.internode_count,
                last_profile_frame.spikelet_count);
    ImGui::Text("Last Invalid Instances: %u", last_profile_frame.invalid_instance_count);
    ImGui::Text("Last Growth phases ms (rules/topology/sort/prop): %.3f / %.3f / %.3f / %.3f",
                last_profile_frame.apply_growth_rules_ms,
                last_profile_frame.apply_topology_rules_ms,
                last_profile_frame.sort_lists_ms,
                last_profile_frame.propagate_geometry_ms);
    ImGui::Text("Last Rebuild phases ms (internode collect/upload, spikelet collect/upload): %.3f / %.3f, %.3f / %.3f",
                last_profile_frame.rebuild_internode_collect_ms,
                last_profile_frame.rebuild_internode_upload_ms,
                last_profile_frame.rebuild_spikelet_collect_ms,
                last_profile_frame.rebuild_spikelet_upload_ms);

    ImGui::SeparatorText("Averages / Maxima");
    ImGui::Text("Update ms avg/max: %.3f / %.3f", avg_update_ms, max_update_ms);
    ImGui::Text("Grow ms avg/max: %.3f / %.3f", avg_grow_ms, max_grow_ms);
    ImGui::Text("Rebuild ms avg/max: %.3f / %.3f", avg_rebuild_ms, max_rebuild_ms);
    ImGui::Text("Avg Growth phases ms (rules/topology/sort/prop): %.3f / %.3f / %.3f / %.3f",
                avg_apply_growth_rules_ms,
                avg_apply_topology_rules_ms,
                avg_sort_lists_ms,
                avg_propagate_geometry_ms);
    ImGui::Text("Avg Rebuild collect ms (internode/spikelet): %.3f / %.3f",
                avg_rebuild_internode_collect_ms,
                avg_rebuild_spikelet_collect_ms);
  }
}

void LSystemLayer::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "auto_grow" << YAML::Value << auto_grow;
  out << YAML::Key << "gdd_per_second" << YAML::Value << gdd_per_second;
  out << YAML::Key << "max_gdd_per_frame" << YAML::Value << max_gdd_per_frame;
  out << YAML::Key << "max_growth_steps_per_frame" << YAML::Value << max_growth_steps_per_frame;
  out << YAML::Key << "seasonality_enabled" << YAML::Value << seasonality_enabled;
  out << YAML::Key << "season_start_day" << YAML::Value << season_start_day;
  out << YAML::Key << "season_end_day" << YAML::Value << season_end_day;
  out << YAML::Key << "chronological_days_per_second" << YAML::Value
      << chronological_days_per_second;
  out << YAML::Key << "simulation_day_of_year" << YAML::Value
      << NormalizeDayOfYear(simulation_day_of_year);
  out << YAML::Key << "reseed_on_reset" << YAML::Value << reseed_on_reset;
  out << YAML::Key << "tassel_color_mode" << YAML::Value << tassel_color_mode;
  out << YAML::Key << "scene_plant_view_tint_enabled" << YAML::Value
      << scene_plant_view_tint_enabled;
  out << YAML::Key << "profiling_enabled" << YAML::Value << profiling_enabled;
  out << YAML::Key << "profiling_history_size" << YAML::Value << profiling_history_size;
  out << YAML::Key << "profiling_export_path" << YAML::Value << profiling_export_path;
}

void LSystemLayer::Deserialize(const YAML::Node& in) {
  if (in["auto_grow"])
    auto_grow = in["auto_grow"].as<bool>();
  if (in["gdd_per_second"])
    gdd_per_second = in["gdd_per_second"].as<float>();
  if (in["max_gdd_per_frame"])
    max_gdd_per_frame = std::max(0.0f, in["max_gdd_per_frame"].as<float>());
  if (in["max_growth_steps_per_frame"])
    max_growth_steps_per_frame = std::max(0, in["max_growth_steps_per_frame"].as<int>());
  if (in["seasonality_enabled"]) {
    seasonality_enabled = in["seasonality_enabled"].as<bool>();
  }
  if (in["season_start_day"]) {
    season_start_day = std::clamp(in["season_start_day"].as<int>(), 0, 364);
  }
  if (in["season_end_day"]) {
    season_end_day = std::clamp(in["season_end_day"].as<int>(), 0, 364);
  }
  if (in["chronological_days_per_second"]) {
    chronological_days_per_second =
        std::max(0.0f, in["chronological_days_per_second"].as<float>());
  }
  simulation_day_of_year =
      NormalizeDayOfYear(static_cast<float>(season_start_day));
  if (in["reseed_on_reset"])
    reseed_on_reset = in["reseed_on_reset"].as<bool>();
  if (in["tassel_color_mode"])
    tassel_color_mode = ClampColorModeIndex(in["tassel_color_mode"].as<int>());
  if (in["scene_plant_view_tint_enabled"])
    scene_plant_view_tint_enabled = in["scene_plant_view_tint_enabled"].as<bool>();
  ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
  RebuildAllPlantGeometry(GetScene());
  if (in["profiling_enabled"])
    profiling_enabled = in["profiling_enabled"].as<bool>();
  if (in["profiling_history_size"])
    profiling_history_size = std::max(30, in["profiling_history_size"].as<int>());
  if (in["profiling_export_path"])
    profiling_export_path = in["profiling_export_path"].as<std::string>();
}
