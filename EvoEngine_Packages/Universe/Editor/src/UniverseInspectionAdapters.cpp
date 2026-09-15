#include "UniverseInspectionAdapters.hpp"
#include "EditorLayer.hpp"
#include "UniverseLayer.hpp"
using namespace universe_package;

bool universe_package::InspectUniverseLayer(InspectorContext&, UniverseLayer& layer) {
  const auto window_title = layer.GetLayerName();
  bool open = layer.enable_inspection;
  if (!ImGui::Begin(window_title.c_str(), &open)) {
    ImGui::End();
    layer.enable_inspection = open;
    return false;
  }
  ImGui::Text("Global simulation time: %.3f", layer.global_simulation_time_);
  ImGui::Text("Frame: %llu", static_cast<unsigned long long>(layer.frame_number_));
  ImGui::Text("FP64 compute: %s", layer.fp64_supported_ ? "supported" : "unavailable");
  ImGui::Text("Compute pipeline: %s", layer.compute_ready_ ? "ready" : "unavailable");
  ImGui::Text("Forward pipeline: %s", layer.render_ready_ ? "ready" : "unavailable");
  ImGui::Text("Registered clusters: %u, stars: %u", layer.registered_render_cluster_count_,
              layer.registered_render_star_count_);
  ImGui::Checkbox("Write star depth", &layer.depth_write);
  ImGui::Checkbox("Show orbit strands", &layer.show_orbit_strands);
  if (layer.show_orbit_strands) {
    int mode = static_cast<int>(layer.orbit_display);
    if (ImGui::Combo("Orbit display", &mode, "All\0Occupied\0Selected\0"))
      layer.orbit_display = static_cast<StarOrbitDisplay>(mode);
    ImGui::DragFloat("Orbit strand radius", &layer.orbit_strand_radius, 0.01f, 0.001f, 10000, "%.3f",
                     ImGuiSliderFlags_AlwaysClamp);
    ImGui::Text("Orbit strands: %zu loops, %zu segments, %u cluster submissions", layer.displayed_orbits_,
                layer.displayed_orbits_ * 256, layer.orbit_draws_);
    ImGui::Text("Last rebuild/upload: %.3f / %.3f ms", layer.orbit_rebuild_ms_, layer.orbit_upload_ms_);
    ImGui::TextWrapped("%s", layer.orbit_status_.c_str());
  }
  ImGui::SliderFloat("Star-view fade strength", &layer.star_fade_strength, 0.0f, 4.0f, "%.2f",
                     ImGuiSliderFlags_AlwaysClamp);
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip(
        "Locked-view target for stars enlarged to one pixel. Galaxy view uses 0. The value transitions with the view. "
        "1: area compensation; higher: stronger fade. Bloom suppression remains enabled.");
  ImGui::TextUnformatted("Star compression: 70%-99% of each camera far distance");
  const double coordinate_scale = layer.star_follow_.following ? 1.0 : kGalaxyDisplayScale;
  ImGui::Text("Universe coordinate scale: %.3f", coordinate_scale);
  if (layer.batch_.parameters.size() == 1)
    ImGui::Text("Effective displayed disk diameter: %.1f",
                (layer.batch_.parameters[0].ellipse0.x + layer.batch_.parameters[0].ellipse0.y) * coordinate_scale);
  ImGui::Text("Capacity: %zu stars, %zu clusters", layer.star_capacity_, layer.cluster_capacity_);
  ImGui::Text("Population / computed / rendered revision: %llu / %llu / %llu",
              static_cast<unsigned long long>(layer.batch_.population_revision),
              static_cast<unsigned long long>(layer.computed_revision_),
              static_cast<unsigned long long>(layer.rendered_revision_));
  ImGui::Text("Render slot: %u; draws across cameras this frame: %u", layer.render_slot_, layer.draws_this_frame_);
  if (ImGui::TreeNode("Packed cluster ranges")) {
    for (const auto& range : layer.batch_.ranges)
      ImGui::Text("%llu: offset %u, count %u", static_cast<unsigned long long>(range.identity), range.offset,
                  range.count);
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Orbit buckets")) {
    for (const auto& [component, clock] : layer.batch_.clocks) {
      const auto cluster = clock.component.lock();
      if (!cluster)
        continue;
      ImGui::PushID(component);
      if (ImGui::TreeNode("Cluster", "Cluster %llu", static_cast<unsigned long long>(clock.identity))) {
        const auto& layout = clock.layout;
        ImGui::Text("Requested %u, active %u; %zu orbits, %llu slots", cluster->GetStarCount(), clock.active_count,
                    layout.orbits.size(), static_cast<unsigned long long>(layout.capacity));
        ImGui::Text("Nominal radial spacing %.6f; layout revision %llu", layout.radial_spacing,
                    static_cast<unsigned long long>(layout.revision));
        ImGui::TextWrapped("%s", layout.status.c_str());
        if (cluster->GetStarCount() > layout.capacity)
          ImGui::TextUnformatted("Capacity limit: excess requested stars are not simulated or rendered.");
        if (ImGui::TreeNode("Orbit occupancy")) {
          ImGuiListClipper clipper;
          clipper.Begin(static_cast<int>(layout.orbits.size()));
          while (clipper.Step())
            for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
              const auto& orbit = layout.orbits[i];
              ImGui::Text("%d: proportion %.6f, occupied %u / %llu", i, orbit.proportion,
                          clock.active_count ? orbit.occupied : 0, static_cast<unsigned long long>(orbit.capacity));
            }
          ImGui::TreePop();
        }
        ImGui::TreePop();
      }
      ImGui::PopID();
    }
    ImGui::TreePop();
  }
  ImGui::TextUnformatted("One population draw per camera, plus an optional hover ring. Picking reads back 48 bytes.");
  if (ImGui::TreeNodeEx("Star picking", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Active: %s%s", layer.pick_camera_name_.c_str(),
                layer.star_picker_.state.current.valid ? "" : " (inactive cursor)");
    ImGui::DragFloat("Minimum picking radius (display pixels)", &layer.pick_minimum_radius_, 0.1f, 0.0f, 20.0f, "%.1f",
                     ImGuiSliderFlags_AlwaysClamp);
    const auto show_hit = [&](const char* label, const StarPickSnapshot& hit) {
      if (!hit.result.valid) {
        ImGui::Text("%s: none", label);
        return;
      }
      const auto cluster = hit.cluster.lock();
      const auto scene = layer.GetScene();
      const std::string name = cluster && scene && scene->IsEntityValid(cluster->GetOwner())
                                   ? scene->GetEntityName(cluster->GetOwner())
                                   : "Deleted cluster";
      ImGui::Text("%s: %s / star %u (cluster %llu)", label, name.c_str(), hit.ordinal,
                  static_cast<unsigned long long>(hit.identity));
      ImGui::Text("Sampled render-space center: %.6f, %.6f, %.6f; ray distance: %.6f", hit.result.position_radius.x,
                  hit.result.position_radius.y, hit.result.position_radius.z, hit.result.distance);
      ImGui::Text("Sample age: %llu frames", static_cast<unsigned long long>(layer.frame_number_ - hit.frame));
    };
    show_hit("Hover", layer.star_picker_.state.hovered);
    show_hit("Selected", layer.star_picker_.state.selected);
    ImGui::Text("Readback: %s; click: %s", layer.star_picker_.Pending() ? "pending" : "idle",
                layer.star_picker_.PendingClick() ? "pending" : "idle");
    ImGui::Text("Selection: %s; following: %s (Space toggles)",
                layer.star_follow_.available ? "available" : "unavailable",
                layer.star_follow_.following ? "yes" : "no");
    ImGui::TextWrapped("%s", layer.star_follow_.status.c_str());
    ImGui::Text("View radius scale: %.3f -> %.0f; fade: %.3f -> %.2f", layer.star_view_.radius_scale,
                layer.star_view_.target_scale, layer.star_view_.fade_strength, layer.star_view_.target_fade_strength);
    if (layer.star_follow_.following)
      ImGui::TextUnformatted("Star-local view: hover and selection locked; Space exits.");
    if (layer.star_follow_.available) {
      const auto& position = layer.star_follow_.selected_world_position;
      ImGui::Text("CPU world position: %.6f, %.6f, %.6f", position.x, position.y, position.z);
      const auto& frame = layer.star_follow_.selected_frame;
      const auto rotation = glm::degrees(glm::eulerAngles(glm::quat_cast(glm::dmat3(frame))));
      ImGui::Text("Reference origin: %.6f, %.6f, %.6f", frame[3].x, frame[3].y, frame[3].z);
      ImGui::Text("Reference rotation (degrees): %.3f, %.3f, %.3f", rotation.x, rotation.y, rotation.z);
    }
    ImGui::TextWrapped("%s", layer.star_picker_.status.c_str());
    if (layer.pick_benchmark_)
      ImGui::TextUnformatted("Benchmark cursor override: center of active camera");
    ImGui::TreePop();
  }
  ImGui::TextWrapped("GPU status: %s", layer.gpu_status_.c_str());
  ImGui::End();
  layer.enable_inspection = open;
  return false;
}

bool universe_package::InspectStarCluster(InspectorContext&, StarCluster& cluster) {
  bool changed = false;
  changed |= ImGui::InputScalar("Star count", ImGuiDataType_U32, &cluster.star_count_);
  changed |= ImGui::InputScalar("Seed", ImGuiDataType_U64, &cluster.seed);
  const double minimum_distance = 0.001, maximum_distance = (std::numeric_limits<double>::max)();
  changed |= ImGui::DragScalar("Star minimum distance", ImGuiDataType_Double, &cluster.star_minimum_distance, 0.01f,
                               &minimum_distance, &maximum_distance, "%.3f", ImGuiSliderFlags_AlwaysClamp);
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip(
        "Nominal center spacing in cluster-local units. Gaussian offsets and motion can reduce separation.");
  changed |= ImGui::Checkbox("Paused", &cluster.paused);
  changed |= ImGui::DragScalar("Time scale", ImGuiDataType_Double, &cluster.time_scale, 0.1f);
  changed |= ImGui::DragScalar("Phase", ImGuiDataType_Double, &cluster.phase, 1.0f);
  if (ImGui::TreeNode("Star size")) {
    const double zero = 0;
    const double maximum = (std::numeric_limits<double>::max)();
    changed |= ImGui::DragScalar("Minimum radius", ImGuiDataType_Double, &cluster.radius_min, 0.01f, &zero, &maximum,
                                 "%.3f", ImGuiSliderFlags_AlwaysClamp);
    changed |= ImGui::DragScalar("Maximum radius", ImGuiDataType_Double, &cluster.radius_max, 0.01f,
                                 &cluster.radius_min, &maximum, "%.3f", ImGuiSliderFlags_AlwaysClamp);
    changed |= ImGui::DragScalar("Normalized deviation", ImGuiDataType_Double, &cluster.radius_deviation, 0.001f, &zero,
                                 &maximum, "%.4f", ImGuiSliderFlags_AlwaysClamp);
    if (cluster.radius_max < cluster.radius_min) {
      cluster.radius_max = cluster.radius_min;
      changed = true;
    }
    ImGui::TextUnformatted("Radius = mix(min, max, clamp(0.5 + Gaussian x deviation, 0, 1)).");
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Density wave")) {
    changed |= ImGui::DragScalar("Disk diameter", ImGuiDataType_Double, &cluster.disk_diameter, 1.0f);
    changed |= ImGui::DragScalar("Disk eccentricity", ImGuiDataType_Double, &cluster.disk_eccentricity, 0.01f);
    changed |= ImGui::DragScalar("Core proportion", ImGuiDataType_Double, &cluster.core_proportion, 0.01f);
    changed |= ImGui::DragScalar("Core eccentricity", ImGuiDataType_Double, &cluster.core_eccentricity, 0.01f);
    changed |= ImGui::DragScalar("Center diameter", ImGuiDataType_Double, &cluster.center_diameter, 1.0f);
    changed |= ImGui::DragScalar("Center eccentricity", ImGuiDataType_Double, &cluster.center_eccentricity, 0.01f);
    changed |= ImGui::DragScalar("Y spread", ImGuiDataType_Double, &cluster.y_spread, 0.001f);
    changed |= ImGui::DragScalar("XZ spread", ImGuiDataType_Double, &cluster.xz_spread, 0.001f);
    changed |= ImGui::DragScalar("Twist", ImGuiDataType_Double, &cluster.twist, 1.0f);
    changed |= ImGui::DragScalarN("Center offset", ImGuiDataType_Double, &cluster.center_offset.x, 3, 1.0f);
    changed |= ImGui::DragScalarN("Center position", ImGuiDataType_Double, &cluster.center_position.x, 3, 1.0f);
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Movement")) {
    changed |= ImGui::DragScalar("Disk speed", ImGuiDataType_Double, &cluster.disk_speed, 0.1f);
    changed |= ImGui::DragScalar("Core speed", ImGuiDataType_Double, &cluster.core_speed, 0.1f);
    changed |= ImGui::DragScalar("Center speed", ImGuiDataType_Double, &cluster.center_speed, 0.1f);
    changed |= ImGui::DragScalar("Disk X tilt", ImGuiDataType_Double, &cluster.disk_tilt_x, 1.0f);
    changed |= ImGui::DragScalar("Disk Z tilt", ImGuiDataType_Double, &cluster.disk_tilt_z, 1.0f);
    changed |= ImGui::DragScalar("Core X tilt", ImGuiDataType_Double, &cluster.core_tilt_x, 1.0f);
    changed |= ImGui::DragScalar("Core Z tilt", ImGuiDataType_Double, &cluster.core_tilt_z, 1.0f);
    changed |= ImGui::DragScalar("Center X tilt", ImGuiDataType_Double, &cluster.center_tilt_x, 1.0f);
    changed |= ImGui::DragScalar("Center Z tilt", ImGuiDataType_Double, &cluster.center_tilt_z, 1.0f);
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Colors and emission")) {
    changed |= ImGui::ColorEdit3("Disk color", &cluster.disk_color.x);
    changed |= ImGui::ColorEdit3("Core color", &cluster.core_color.x);
    changed |= ImGui::ColorEdit3("Center color", &cluster.center_color.x);
    changed |= ImGui::DragFloat("Disk emission", &cluster.disk_emission_intensity, 0.01f);
    changed |= ImGui::DragFloat("Core emission", &cluster.core_emission_intensity, 0.01f);
    changed |= ImGui::DragFloat("Center emission", &cluster.center_emission_intensity, 0.01f);
    ImGui::TreePop();
  }
  ImGui::TextUnformatted("Simulation and rendering are managed by Universe Layer.");
  return changed;
}

bool universe_package::InspectPlanetTerrain(InspectorContext& context, PlanetTerrain& planet_terrain) {
  return context.editor_layer->DragAndDropButton<Material>(planet_terrain.surface_material, "Material");
}
