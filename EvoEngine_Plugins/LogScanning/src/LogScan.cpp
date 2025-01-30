#include "LogScan.hpp"
#include "JoeScanScanner.hpp"
#include "Json.hpp"
#include "Prefab.hpp"
#include "Scene.hpp"
using namespace log_scanning_plugin;
void LogScan::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "profiles" << YAML::Value << YAML::BeginSeq;
  for (const auto& profile : profiles) {
    out << YAML::BeginMap;
    {
      out << YAML::Key << "encoder_value" << YAML::Value << profile.encoder_value;
      out << YAML::Key << "points" << YAML::Value
          << YAML::Binary(reinterpret_cast<const unsigned char*>(profile.points.data()),
                          profile.points.size() * sizeof(glm::vec2));
      out << YAML::Key << "brightness" << YAML::Value
          << YAML::Binary(reinterpret_cast<const unsigned char*>(profile.brightness.data()),
                          profile.brightness.size() * sizeof(float));
    }
    out << YAML::EndMap;
  }
}

void LogScan::Deserialize(const YAML::Node& in) {
  if (in["profiles"]) {
    profiles.clear();
    for (const auto& in_profile : in["profiles"]) {
      profiles.emplace_back();
      auto& profile = profiles.back();
      if (in_profile["encoder_value"])
        profile.encoder_value = in_profile["encoder_value"].as<float>();
      if (in_profile["points"]) {
        const auto in_points = in_profile["points"].as<YAML::Binary>();
        profile.points.resize(in_points.size() / sizeof(glm::vec2));
        std::memcpy(profile.points.data(), in_points.data(), in_points.size());
      }
      if (in_profile["brightness"]) {
        const auto in_brightness = in_profile["brightness"].as<YAML::Binary>();
        profile.brightness.resize(in_brightness.size() / sizeof(float));
        std::memcpy(profile.brightness.data(), in_brightness.data(), in_brightness.size());
      }
    }
  }
}

void LogScan::CollectAssetRef(std::vector<AssetRef>& list) {
}

bool LogScan::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  static AssetRef config;
  static AssetRef scanner_prefab;
  editor_layer->DragAndDropButton<Json>(config, "JsConfig");
  editor_layer->DragAndDropButton<Prefab>(scanner_prefab, "Scanner Model");
  const auto config_asset = config.Get<Json>();
  const auto scanner_prefab_asset = scanner_prefab.Get<Prefab>();
  if (config_asset && scanner_prefab_asset) {
    if (ImGui::Button("Generate prefabs")) {
      JoeScanConfig joe_scan_config;
      joe_scan_config.Import(config_asset);
      joe_scan_config.PlacePrefabs(scanner_prefab_asset);
    }
  }

  static std::shared_ptr<ParticleInfoList> joe_scan_list;
  static std::shared_ptr<ParticleInfoList> profile_list;
  static std::shared_ptr<ParticleInfoList> profile_points_list;
  if (!joe_scan_list)
    joe_scan_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!profile_list)
    profile_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!profile_points_list)
    profile_points_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  if (ImGui::Button("Regularize")) {
    Regularize();
  }
  if (ImGui::Button("Recenter")) {
    Recenter();
  }

  static bool enable_joe_scan_rendering = true;
  ImGui::Checkbox("Render LogScan", &enable_joe_scan_rendering);
  static bool enable_profile_rendering = true;
  ImGui::Checkbox("Render Profile", &enable_profile_rendering);
  static auto scan_color = glm::vec4(1, 1, 1, 0.1f);
  static auto profile_color = glm::vec4(1, 0, 0, 1.f);
  static auto profile_points_color = glm::vec4(0, 0, 1, 1.f);
  static float brightness_factor = 1.f;
  static bool brightness = true;
  ImGui::Checkbox("Brightness", &brightness);
  if (brightness)
    ImGui::DragFloat("Brightness factor", &brightness_factor, 0.001f, 0.0f, 3.0f);

  ImGui::ColorEdit4("Color", &scan_color.x);
  if (enable_joe_scan_rendering) {
    if (ImGui::Button("Refresh LogScan")) {
      std::vector<ParticleInfo> data;
      for (const auto& profile : profiles) {
        const auto start_index = data.size();
        data.resize(profile.points.size() + start_index);
        Jobs::RunParallelFor(profile.points.size(), [&](unsigned i) {
          data[i + start_index].instance_matrix.SetPosition(
              glm::vec3(profile.points[i].x, profile.points[i].y, profile.encoder_value));
          data[i + start_index].instance_matrix.SetScale(glm::vec3(0.0005f, 0.0005f, 2.4384f / profiles.size()));
          data[i + start_index].instance_color = scan_color;
          if (brightness)
            data[i + start_index].instance_color.w =
                static_cast<float>(profile.brightness[i]) / 2048.f * brightness_factor;
        });
      }
      joe_scan_list->SetParticleInfos(data);
    }
  }
  if (config_asset && enable_profile_rendering && !profiles.empty()) {
    static int profile_index = 0;
    static int scan_head_index = 0;
    bool changed = ImGui::DragInt("Scan head index", &scan_head_index, 1, 0, 3);

    if (ImGui::DragInt("Profile Index", &profile_index, 1, 0, profiles.size()) || changed) {
      profile_index = glm::clamp(profile_index, 0, static_cast<int>(profiles.size()));
      std::vector<ParticleInfo> profile_data;
      auto& profile = profiles[profile_index];
      JoeScanConfig joe_scan_config;
      joe_scan_config.Import(config_asset);
      const auto boundary_points = profile.BuildBoundary(joe_scan_config);
      profile_data.resize(boundary_points.size());
      Jobs::RunParallelFor(boundary_points.size(), [&](unsigned i) {
        profile_data[i].instance_matrix.SetPosition(
            glm::vec3(boundary_points[i].x, boundary_points[i].y, profile.encoder_value));
        profile_data[i].instance_matrix.SetScale(glm::vec3(0.001f, 0.001f, 0.001f));
        profile_data[i].instance_color = profile_color;
      });
      profile_list->SetParticleInfos(profile_data);

      /*
      points_data.resize(profile.points.size());
      Jobs::RunParallelFor(points_data.size(), [&](unsigned i) {
        points_data[i].instance_matrix.SetPosition(
            glm::vec3(profile.points[i].x, profile.points[i].y, profile.encoder_value));
        points_data[i].instance_matrix.SetScale(glm::vec3(0.001f, 0.001f, 0.001f));
        points_data[i].instance_color = profile_points_color;
      });
      */
      /*
      if (scan_head_index >= 0 && scan_head_index <= profile.grid_points.size()) {
        std::vector<ParticleInfo> points_data;
        points_data.resize(profile.grid_points[scan_head_index].size());
        Jobs::RunParallelFor(points_data.size(), [&](unsigned i) {
          points_data[i].instance_matrix.SetPosition(glm::vec3(profile.grid_points[scan_head_index][i].x,
                                                               profile.grid_points[scan_head_index][i].y,
                                                               profile.encoder_value));
          points_data[i].instance_matrix.SetScale(glm::vec3(0.001f, 0.001f, 0.001f));
          points_data[i].instance_color = profile_points_color;
        });
        profile_points_list->SetParticleInfos(points_data);
      }*/
    }
  }
  GizmoSettings settings{};
  settings.draw_settings.blending = true;

  if (enable_profile_rendering) {
    GizmoSettings settings{};
    settings.draw_settings.blending = true;
    editor_layer->DrawGizmoCubes(profile_list, glm::mat4(1), 1.f, settings);
    editor_layer->DrawGizmoCubes(profile_points_list, glm::mat4(1), 1.f, settings);
  }
  if (enable_joe_scan_rendering) {
    editor_layer->DrawGizmoCubes(joe_scan_list, glm::mat4(1), 1.f, settings);
  }

  return changed;
}

void LogScan::Regularize() {
  std::map<float, uint32_t> valid_profiles;
  for (uint32_t i = 0; i < profiles.size(); i++) {
    auto& profile = profiles[i];
    if (profile.points.size() < 1000) {
      continue;
    }
    valid_profiles.insert({profile.encoder_value, i});
  }
  const auto copy = profiles;
  profiles.clear();
  profiles.reserve(valid_profiles.size());
  for (const auto& i : valid_profiles) {
    profiles.emplace_back(copy[i.second]);
  }

  float min_encoder = FLT_MAX;
  float max_encoder = -FLT_MAX;
  for (const auto& i : profiles) {
    min_encoder = glm::min(min_encoder, i.encoder_value);
    max_encoder = glm::max(max_encoder, i.encoder_value);
  }
  const auto distance = max_encoder - min_encoder;
  const auto center = (max_encoder + min_encoder) * .5f;
  for (auto& profile : profiles) {
    profile.encoder_value -= center;
    profile.encoder_value *= 2.4384f / distance;
  }
  for (auto& profile : profiles) {
    for (auto& point : profile.points) {
      point *= 0.0001f;
      point *= 0.254f;
    }
  }
}

void LogScan::Recenter() {
  auto center = glm::vec2(0.f);
  for (const auto& profile : profiles) {
    auto points_min = glm::vec2(FLT_MAX, FLT_MAX);
    auto points_max = glm::vec2(-FLT_MAX, -FLT_MAX);

    const auto centered_points = profile.points;
    for (const auto& i : centered_points) {
      points_min = glm::min(points_min, i);
      points_max = glm::max(points_max, i);
    }
    center += (points_min + points_max) * .5f;
  }
  center /= profiles.size();
  for (auto& profile : profiles) {
    for (auto& point : profile.points)
      point -= center;
  }
}

void JoeScanConfig::Import(const std::shared_ptr<Json>& json) {
  auto& config = json->m_json;
  const auto& heads = config["ScanHeads"];
  scan_heads.clear();
  for (auto& it : heads) {
    scan_heads.emplace_back();
    auto head = it.get<json::object_t>();
    const int id = head["Id"];
    auto& alignments = head["Alignments"];
    int alignment_count = 0;
    glm::vec2 shift = {0.0f, 0.0f};
    float roll_degree = 0;
    for (auto& itc : alignments) {
      auto alignment = itc.get<json::object_t>();
      alignment_count++;
      std::string cam_str = alignment["Camera"];
      std::string las_str = alignment["Laser"];
      std::string pair_str = cam_str + " " + las_str;
      roll_degree += glm::mod(static_cast<float>(alignment["RollDeg"]), 360.f);
      shift.x -= static_cast<float>(alignment["ShiftX"]);
      shift.y -= static_cast<float>(alignment["ShiftY"]);
    }
    auto& scan_head = scan_heads.back();
    scan_head.id = id;
    scan_head.roll = glm::radians(roll_degree / static_cast<float>(alignment_count));
    scan_head.shift = shift / static_cast<float>(alignment_count);
  }
}

void JoeScanConfig::PlacePrefabs(const std::shared_ptr<Prefab>& prefab) const {
  const auto scene = Application::GetActiveScene();
  const auto parent = scene->CreateEntity("ScanHeads");
  for (const auto& scan_head : scan_heads) {
    auto entity = prefab->ToEntity(scene);
    scene->SetEntityName(entity, std::to_string(scan_head.id));
    GlobalTransform gt;
    gt.SetPosition(glm::vec3(scan_head.shift.x * 0.0254f, scan_head.shift.y * 0.0254f, 0.0f));
    gt.SetEulerRotation(glm::vec3(glm::radians(90.f), glm::radians(-90.f), scan_head.roll));
    scene->SetDataComponent(entity, gt);
    scene->SetParent(entity, parent);
  }
}

std::vector<glm::vec2> LogScanProfile::BuildBoundary(const JoeScanConfig& joe_scan_config) {
  std::vector<glm::vec2> ret_val;
  /*
  const auto get_radians = [](const glm::vec2& a, const glm::vec2& b) {
    const auto radians = glm::acos(glm::clamp(glm::dot(a, b) / (glm::length(a) * glm::length(b)), -1.0f, 1.0f));
    if (const auto det = a.x * b.y - a.y * b.x; det < 0)
      return -glm::degrees(radians);
    return glm::degrees(radians);
  };

  const auto ray_intersects_aabb = [](const glm::vec2& ray_origin, const glm::vec2& ray_dir, const glm::vec2& min_bound,
                                      const glm::vec2& max_bound, float& t) {
    float t_min = (min_bound.x - ray_origin.x) / ray_dir.x;
    float t_max = (max_bound.x - ray_origin.x) / ray_dir.x;

    if (t_min > t_max)
      std::swap(t_min, t_max);

    float ty_min = (min_bound.y - ray_origin.y) / ray_dir.y;
    float ty_max = (max_bound.y - ray_origin.y) / ray_dir.y;

    if (ty_min > ty_max)
      std::swap(ty_min, ty_max);

    // Check for overlap between the intervals on the x and y axes
    if (t_min > ty_max || ty_min > t_max)
      return false;

    // Update tMin and tMax to ensure the intersection occurs within both intervals
    if (ty_min > t_min)
      t_min = ty_min;
    if (ty_max < t_max)
      t_max = ty_max;

    // Set the first intersection point distance and calculate the intersection point
    t = t_min;
    return true;
  };

  auto points_min = glm::vec2(FLT_MAX, FLT_MAX);
  auto points_max = glm::vec2(-FLT_MAX, -FLT_MAX);

  auto centered_points = points;
  // constexpr auto x_bound = 0.0943f;
  // constexpr auto y_bound = 0.0689f;
  for (const auto& i : points) {
    points_min = glm::min(points_min, i);
    points_max = glm::max(points_max, i);
  }

  // const auto center = (points_min + points_max) * .5f;
  constexpr auto center = glm::vec2(0.0f);
  for (auto& i : centered_points)
    i -= center;
  points_max -= center;
  points_min -= center;
  grids.resize(joe_scan_config.scan_heads.size());
  constexpr auto x_limit = 0.0635f;
  constexpr auto y_limit = 0.0381f;
  for (uint32_t grid_i = 0; grid_i < grids.size(); grid_i++) {
    constexpr float rotation_factor = 5.f;
    auto& grid = grids[grid_i];
    grid.Reset(0.001f, points_min - glm::vec2(0.005f), points_max + glm::vec2(0.005f));
    grid.Clear();

    const auto& scan_head = joe_scan_config.scan_heads[grid_i];
    const auto scan_head_position = scan_head.shift * 0.0254f;
    const auto scan_head_direction = glm::normalize(-scan_head_position);
    std::map<int, float> rotation_depth{};

    bool top_valid = glm::dot(scan_head_direction, glm::vec2(0, 1)) < -0.5f;
    bool left_valid = glm::dot(scan_head_direction, glm::vec2(1, 0)) < -0.5f;
    bool right_valid = glm::dot(scan_head_direction, glm::vec2(-1, 0)) < -0.5f;
    bool bottom_valid = glm::dot(scan_head_direction, glm::vec2(0, -1)) < -0.5f;

    for (const auto& point : centered_points) {
      if (glm::abs(point.x) > 0.125f)
        continue;
      if (glm::abs(point.y) > 0.1f)
        continue;
      bool point_top = point.y > 0.f && glm::abs(point.x / point.y) < 9.f / 7.f;
      bool point_bottom = point.y < 0.f && glm::abs(point.x / point.y) < 9.f / 7.f;
      bool point_left = point.x > 0.f && glm::abs(point.x / point.y) >= 9.f / 7.f;
      bool point_right = point.x < 0.f && glm::abs(point.x / point.y) >= 9.f / 7.f;

      if (top_valid) {
        if (left_valid) {
          if (point_bottom || point_right)
            continue;
        } else if (right_valid) {
          if (point_bottom || point_left)
            continue;
        } else {
          if (point.y < 0.f)
            continue;
        }
      } else if (bottom_valid) {
        if (left_valid) {
          if (point_top || point_right)
            continue;
        } else if (right_valid) {
          if (point_top || point_left)
            continue;
        } else {
          if (point.y > 0.f)
            continue;
        }
      } else {
        if (left_valid) {
          if (point.x < 0.f)
            continue;
        } else if (right_valid) {
          if (point.x > 0.f)
            continue;
        }
      }
      const auto v = point - scan_head_position;
      const auto v_dir = glm::normalize(v);
      const auto radians = get_radians(v, scan_head_direction);
      const int rotation = static_cast<int>(radians * rotation_factor);
      auto depth = glm::length(v);

      if (float t; ray_intersects_aabb(scan_head_position, v_dir, glm::vec2(-x_limit, -y_limit),
                                       glm::vec2(x_limit, y_limit), t)) {
        depth = glm::min(t, depth);
      }
      if (const auto search = rotation_depth.find(rotation); search == rotation_depth.end()) {
        rotation_depth[rotation] = depth;
      } else if (depth < search->second) {
        rotation_depth.at(rotation) = depth;
      }
    }

    std::vector<std::pair<int, float>> rotation_depth_list;
    rotation_depth_list.reserve(rotation_depth.size());
    for (const auto& it : rotation_depth) {
      rotation_depth_list.emplace_back(it.first, it.second);
    }
    if (rotation_depth_list.size() < 2)
      continue;
    Jobs::RunParallelFor(grid.RefCells().size(), [&](const size_t cell_i) {
      const auto pixel_position = grid.GetPosition(static_cast<unsigned>(cell_i));
      const auto v = pixel_position - scan_head_position;
      const auto radians = get_radians(v, scan_head_direction);
      const int rotation = static_cast<int>(radians * rotation_factor);
      const auto depth = glm::length(v);
      if (rotation < rotation_depth_list.front().first)
        return;
      if (rotation > rotation_depth_list.back().first)
        return;
      for (uint32_t list_i = 0; list_i < rotation_depth_list.size() - 1; list_i++) {
        const auto& left = rotation_depth_list[list_i];
        const auto& right = rotation_depth_list[list_i + 1];
        if (rotation < left.first || rotation > right.first)
          continue;
        const auto a = static_cast<float>(rotation - left.first) / static_cast<float>(right.first - left.first);
        if (const auto interpolated_depth = glm::mix(left.second, right.second, a); depth >= interpolated_depth) {
          grid.RefCell(static_cast<unsigned>(cell_i)).occluded = true;
          return;
        }
      }
    });
  }
  grid_points.resize(grids.size());

  for (int grid_index = 0; grid_index < grids.size(); grid_index++) {
    grid_points[grid_index].clear();
    const auto& grid = grids[grid_index];
    for (uint32_t cell_i = 0; cell_i < grid.PeekCells().size(); cell_i++) {
      const auto cell_position = grid.GetPosition(cell_i);
      if (glm::abs(cell_position.x) < x_limit && glm::abs(cell_position.y) < y_limit)
        continue;
      if (grid.PeekCells()[cell_i].occluded) {
        grid_points[grid_index].emplace_back(grid.GetPosition(cell_i) + center);
      }
    }
  }

  ProfileGrid final_grid;
  final_grid.Reset(0.001f, points_min - glm::vec2(0.005f), points_max + glm::vec2(0.005f));
  final_grid.Clear();

  for (uint32_t cell_i = 0; cell_i < final_grid.RefCells().size(); cell_i++) {
    const auto cell_position = final_grid.GetPosition(cell_i);
    if (glm::abs(cell_position.x) < x_limit && glm::abs(cell_position.y) < y_limit)
      continue;

    bool occluded = true;
    for (const auto& grid : grids) {
      if (!grid.PeekCells()[cell_i].occluded) {
        occluded = false;
        break;
      }
    }
    final_grid.RefCells()[cell_i].occluded = occluded;
    if (occluded) {
      ret_val.emplace_back(final_grid.GetPosition(cell_i) + center);
    }
  }
  */
  return ret_val;
}
