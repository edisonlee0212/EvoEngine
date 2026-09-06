// Godot debug integration adapter, 34d06658a85845111a50db9e485ec4a0701d4298.
// See docs/licenses/Godot-MIT.txt. Session controls and evidence export are host-only.
#include "SdfgiDebug.hpp"
#include "Camera.hpp"
#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "RenderTexture.hpp"
#include "SdfgiLight.hpp"
#include "Serialization.hpp"
#include "Texture2D.hpp"

#include <cmath>
#include <fstream>
#include <set>

using namespace evo_engine;

const char* evo_engine::GetSdfgiDebugViewName(const SdfgiDebugView view) {
  static const char* names[]{"Beauty",
                             "Cascades",
                             "SDF",
                             "Probes",
                             "Probe visibility",
                             "Dirty regions",
                             "Distance slice",
                             "SDFGI diffuse",
                             "SDFGI specular",
                             "Environment fallback",
                             "Contributors / receivers"};
  const auto index = static_cast<uint32_t>(view);
  return index < std::size(names) ? names[index] : "Unknown";
}

bool SdfgiDebugState::BeginFrame(const uint32_t scene_frame) {
  if (last_boundary == scene_frame)
    return false;
  last_boundary = scene_frame;
  if (frozen && !single_step && !full_redraw && !reset_history)
    return false;
  single_step = false;
  return true;
}

void SdfgiDebugState::Invalidate(const char* category, const char* reason) {
  if (!enabled)
    return;
  ++invalidations[category];
  last_reason = reason;
}

bool SdfgiDebugState::MatchesCamera(const uint64_t id, const bool editor_scene, const bool ordinary_raster) const {
  return enabled && ordinary_raster && (camera_id ? camera_id == id : editor_scene);
}

void evo_engine::AddSdfgiHistoryReset(RenderGraph& graph, const std::shared_ptr<SdfgiRuntime>& runtime) {
  const auto resources = runtime->resources;
  RenderPassDescriptor pass{"SdfgiHistoryReset", RenderPassQueue::Graphics, RenderPassScope::Frame};
  pass.dependencies = {RenderPassNames::sdfgi_maintenance};
  for (const auto& [name, texture] : resources->textures)
    if (name == "Atlas" || name.find("History") != std::string::npos || name.find("Average") != std::string::npos)
      pass.resources.push_back(
          {"Frame.SDFGI." + name, RenderResourceUsage::Write, RenderResourceState::TransferDestinationGeneral});
  pass.resources.push_back({"Frame.SDFGI.Status", RenderResourceUsage::ReadWrite, RenderResourceState::General});
  resources->transport_pass = 0;
  resources->transport_recorded = false;
  runtime->published = false;
  runtime->debug->Invalidate("reset", "Explicit probe-history reset");
  graph.AddPass(pass, [runtime, resources](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
      ApplyGraphResourceBarriers(command, context);
      for (const auto& [name, texture] : resources->textures)
        if (name == "Atlas" || name.find("History") != std::string::npos || name.find("Average") != std::string::npos) {
          const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, texture.requirement.layers};
          Platform::ClearColorImage(command, *texture.image, VkClearColorValue{}, 1, &range);
        }
      resources->buffers.at("Status").buffer->Fill(command, 0, 4, 0);
      resources->buffers.at("Status").buffer->Fill(command, 8, 4, 0);
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                             VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
    });
    runtime->debug->reset_history = false;
  });
}

void evo_engine::UpdateSdfgiDebugMemory(const SdfgiRuntime& runtime,
                                        const std::vector<std::vector<std::shared_ptr<SdfgiResources>>>& owners) {
  auto& debug = *runtime.debug;
  if (!debug.enabled)
    return;
  debug.active_bytes = {};
  debug.retiring_bytes = {};
  std::set<const SdfgiResources*> fields;
  for (const auto& slot : owners)
    for (const auto& owner : slot)
      if (owner)
        fields.insert(owner.get());
  if (runtime.resources)
    fields.insert(runtime.resources.get());
  for (const auto* field : fields)
    for (uint32_t i = 0; i < 4; ++i)
      (field == runtime.resources.get() ? debug.active_bytes : debug.retiring_bytes)[i] +=
          field->GetAllocationBytes(static_cast<SdfgiMemoryClass>(i));
  debug.peak_transient_bytes = std::max(
      debug.peak_transient_bytes, debug.active_bytes[1] + debug.active_bytes[2] + debug.active_bytes[3] +
                                      debug.retiring_bytes[1] + debug.retiring_bytes[2] + debug.retiring_bytes[3]);
}

std::string evo_engine::BuildSdfgiDebugSnapshot(const SdfgiRuntime& runtime) {
  const auto& debug = *runtime.debug;
  const auto& field = runtime.resources;
  YAML::Node node;
  node["schema"] = 1;
  node["reference_repository"] = "C:/Users/lllll/Documents/GitHub/godot";
  node["reference_commit"] = kSdfgiReferenceCommit;
  node["device"] = runtime.capabilities.device_name;
  node["driver_version"] = runtime.capabilities.driver_version;
  node["rt_pipeline"] = Platform::RayTracingEnabled();
  node["ray_query"] = Platform::RayQueryEnabled();
  node["blas"] = node["tlas"] = Platform::RayAccelerationStructureEnabled();
  node["requested_provider"] = "Automatic SDFGI";
  node["effective_provider"] = runtime.published ? "Automatic SDFGI" : "Environment";
  node["capability"] = runtime.capabilities.ToString();
  YAML::Emitter settings;
  SerializeSdfgiSettings(settings, runtime.settings);
  node["settings"] = YAML::Load(settings.c_str());
  node["voxel_grid"] = runtime.settings.GridSize();
  node["probe_grid"] = runtime.settings.ProbeSize();
  node["published"] = runtime.published;
  node["scene_frame"] = runtime.last_scene_frame;
  node["maintenance_count"] = runtime.maintenance_count;
  node["generation"] = field ? field->transport_pass : 0;
  node["history_index"] =
      field && field->transport_pass ? (field->transport_pass - 1) % std::max(1u, runtime.settings.history_size) : 0;
  node["light_phase"] = runtime.last_scene_frame % std::max(1u, runtime.settings.light_update_frames);
  node["frozen"] = debug.frozen;
  node["seed"] = field ? field->debug_seed : 0;
  node["requested_seed"] = debug.seed;
  node["depth_test"] = debug.depth_test;
  node["fallback_coverage_view"] = "red=Environment, green=SDFGI, black=no deferred surface";
  node["view"] = GetSdfgiDebugViewName(debug.view);
  node["selected_cascade"] = debug.cascade;
  node["selected_probe"] = debug.probe;
  node["selected_slice"] = debug.slice;
  node["anchor"]["camera"] = runtime.anchor.camera_id;
  node["anchor"]["position"] = runtime.anchor.world_position;
  node["anchor"]["source"] = static_cast<uint32_t>(runtime.anchor.source);
  node["anchor"]["missing"] = runtime.missing_anchor;
  node["anchor"]["override_fell_back"] = runtime.anchor.override_fell_back;
  node["last_reason"] = debug.last_reason;
  node["fallback_reason"] = runtime.fallback_reason;
  node["debug_failure"] = debug.failure;
  node["diagnostic_tracking_enabled"] = debug.enabled;
  node["invalidation_counts"] = debug.invalidations;
  node["accepted_contributors"] = runtime.contributors.entries.size();
  for (const auto& [reason, count] : runtime.scene_snapshot.excluded)
    node["excluded_contributors"][GetSdfgiExclusionName(reason)] = count;
  for (uint32_t c = 0; c < runtime.cascades.size(); ++c) {
    const auto& cascade = runtime.cascades[c];
    const auto bounds = cascade.WorldBounds(SdfgiYMultiplier(runtime.settings.vertical_scale));
    YAML::Node entry;
    entry["index"] = c;
    entry["cell_size"] = cascade.cell_size;
    entry["center_cells"] = cascade.position;
    entry["probe_spacing"] =
        glm::vec3(8 * cascade.cell_size) / glm::vec3(1, SdfgiYMultiplier(runtime.settings.vertical_scale), 1);
    entry["min"] = bounds.min;
    entry["max"] = bounds.max;
    entry["dirty"] = cascade.dirty_regions;
    entry["full_redraw"] = cascade.full_redraw;
    entry["pending_changes"] = c < runtime.pending_changes.size() ? runtime.pending_changes[c] : 0;
    if (field && c < field->solid_cell_dispatch.size())
      entry["compact_cells_fence_delayed"] = field->solid_cell_dispatch[c].total_count;
    node["cascades"].push_back(entry);
  }
  for (const auto& region : runtime.pending_regions) {
    YAML::Node entry;
    entry["cascade"] = region.cascade;
    entry["offset"] = region.offset;
    entry["size"] = region.size;
    node["pending_regions"].push_back(entry);
  }
  const char* memory_names[]{"field", "scratch", "upload", "diagnostic"};
  for (uint32_t i = 0; i < 4; ++i) {
    node["memory_bytes"]["active"][memory_names[i]] = debug.active_bytes[i];
    node["memory_bytes"]["retiring"][memory_names[i]] = debug.retiring_bytes[i];
  }
  node["memory_bytes"]["peak_transient"] = debug.peak_transient_bytes;
  if (field) {
    node["geometry_updates"] = field->geometry_update_count;
    node["payload_updates"] = field->payload_update_count;
    node["initialized"] = field->initialization_recorded;
    node["last_preprocess_readback"]["available"] = field->preprocess_status_available;
    node["last_preprocess_readback"]["failure_flags"] = field->preprocess_status.failure_flags;
    node["last_preprocess_readback"]["generation"] = field->preprocess_status.generation;
    node["voxel_failure"] = field->voxel_failure;
    node["light_failure"] = field->light_failure;
    node["transport_failure"] = field->transport_failure;
    std::shared_ptr<SdfgiLightFrame> latest_lights;
    for (const auto& frame : field->light_frames)
      if (frame && (!latest_lights || frame->scene_frame > latest_lights->scene_frame))
        latest_lights = frame;
    if (latest_lights)
      for (const auto& cascade : latest_lights->lights) {
        YAML::Node entry;
        entry["static"] = cascade.data[0].size();
        entry["dynamic"] = cascade.data[1].size();
        entry["static_overflow"] = cascade.overflow[0];
        entry["dynamic_overflow"] = cascade.overflow[1];
        node["cascade_lights"].push_back(entry);
      }
  }
  node["timings_fence_delayed"] = true;
  for (const auto& stat : Platform::GetCpuTimingStats())
    if (stat.name == "SDFGI Planning")
      node["cpu_ms"][stat.name] = stat.last_milliseconds;
  for (const auto& stat : Platform::GetGpuTimestampStats())
    if (stat.name.find("Sdfgi") != std::string::npos || stat.name.find("SDFGI") != std::string::npos ||
        stat.name == "Deferred Compute Lighting") {
      auto entry = node["gpu_ms"][stat.name];
      entry["last"] = stat.last_milliseconds;
      entry["median"] = stat.MedianMilliseconds();
      entry["samples"] = stat.sample_count;
    }
  YAML::Emitter out;
  out << node;
  return out.c_str();
}

void evo_engine::CaptureSdfgiDebugImage(const SdfgiRuntime& runtime, const std::filesystem::path& path) {
  const auto& debug = *runtime.debug;
  const auto camera = debug.rendered_camera.lock();
  if (!debug.enabled || !camera || !camera->GetRenderTexture() || debug.rendered_field.lock() != runtime.resources ||
      debug.rendered_generation != (runtime.resources ? runtime.resources->transport_pass : 0) ||
      debug.rendered_view != debug.view ||
      debug.rendered_selection != glm::uvec3(debug.cascade, debug.probe, debug.slice) ||
      debug.rendered_boundary != debug.last_boundary || debug.rendered_resolution != camera->GetSize() ||
      debug.rendered_camera_selection != debug.camera_id || debug.rendered_depth_test != debug.depth_test)
    throw std::runtime_error("Render the selected SDFGI view before capturing its matching snapshot");
  auto metadata_path = path;
  metadata_path.replace_extension("yaml");
  if (std::filesystem::exists(path) || std::filesystem::exists(metadata_path))
    throw std::runtime_error("Choose a new capture filename; existing evidence is not overwritten");
  if (!path.parent_path().empty())
    std::filesystem::create_directories(path.parent_path());
  auto snapshot = YAML::Load(BuildSdfgiDebugSnapshot(runtime));
  snapshot["image"] = path.filename().string();
  snapshot["camera"]["id"] = camera->GetHandle().GetValue();
  snapshot["camera"]["resolution"] = camera->GetSize();
  snapshot["camera"]["view_projection"] = debug.rendered_view_projection;
  std::vector<glm::vec4> pixels;
  camera->GetRenderTexture()->GetRgbaChannelData(pixels);
  if (runtime.resources && runtime.resources->initialization_recorded) {
    SdfgiFieldStatus status;
    runtime.resources->buffers.at("Status").buffer->Download(status);
    snapshot["gpu_status"]["ready"] = status.ready;
    snapshot["gpu_status"]["generation"] = status.generation;
    snapshot["gpu_status"]["failure_flags"] = status.failure_flags;
    snapshot["effective_provider"] =
        runtime.published && status.ready && !status.failure_flags && status.generation == debug.rendered_generation
            ? "Automatic SDFGI"
            : "Environment";
  }
  uint64_t nonfinite = 0, fallback = 0, covered = 0;
  for (auto& pixel : pixels) {
    if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y) || !std::isfinite(pixel.z) || !std::isfinite(pixel.w))
      ++nonfinite;
    if (debug.view == SdfgiDebugView::Fallback && pixel.x + pixel.y > 0)
      (pixel.x > pixel.y ? fallback : covered)++;
    pixel.w = 1;
  }
  if (nonfinite)
    throw std::runtime_error("SDFGI capture contains " + std::to_string(nonfinite) + " nonfinite pixels");
  snapshot["nonfinite_pixels"] = nonfinite;
  if (debug.view == SdfgiDebugView::Fallback) {
    snapshot["fallback_dominant_pixels"] = fallback;
    snapshot["sdfgi_dominant_pixels"] = covered;
  }
  std::vector<float> rgba(pixels.size() * 4);
  std::memcpy(rgba.data(), pixels.data(), rgba.size() * sizeof(float));
  Texture2D::StoreToPng(path, rgba, debug.rendered_resolution.x, debug.rendered_resolution.y, 4, 4);
  if (!std::filesystem::exists(path))
    throw std::runtime_error("SDFGI image export failed");
  std::ofstream output(metadata_path);
  output << snapshot;
  if (!output)
    throw std::runtime_error("SDFGI snapshot export failed");
}
