#include "LogScan.hpp"

#include "CellGrid.hpp"
#include "JoeScanScanner.hpp"
#include "Json.hpp"
#include "Prefab.hpp"
#include "Scene.hpp"
using namespace log_scanning_plugin;
using namespace eco_sys_lab_plugin;
void LogScan::Serialize(YAML::Emitter& out) const {
  config.Save("config", out);
  scanner_prefab.Save("scanner_prefab", out);
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
  config.Load("config", in);
  scanner_prefab.Load("scanner_prefab", in);
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
  list.emplace_back(config);
  list.emplace_back(scanner_prefab);
}

bool LogScan::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

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
    joe_scan_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!profile_list)
    profile_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  if (!profile_points_list)
    profile_points_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  if (ImGui::Button("Regularize")) {
    Regularize();
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
      const auto& profile = profiles[profile_index];
      JoeScanConfig joe_scan_config;
      joe_scan_config.Import(config_asset);
      const auto boundary_points = profile.BuildBoundary(joe_scan_config, scan_head_index);
      profile_data.resize(boundary_points.size());
      Jobs::RunParallelFor(boundary_points.size(), [&](unsigned i) {
        profile_data[i].instance_matrix.SetPosition(
            glm::vec3(boundary_points[i].x, boundary_points[i].y, profile.encoder_value));
        profile_data[i].instance_matrix.SetScale(glm::vec3(0.001f, 0.001f, 0.001f));
        profile_data[i].instance_color = profile_color;
      });
      profile_list->SetParticleInfos(profile_data);

      std::vector<ParticleInfo> points_data;
      points_data.resize(profile.points.size());
      Jobs::RunParallelFor(points_data.size(), [&](unsigned i) {
        points_data[i].instance_matrix.SetPosition(
            glm::vec3(profile.points[i].x, profile.points[i].y, profile.encoder_value));
        points_data[i].instance_matrix.SetScale(glm::vec3(0.001f, 0.001f, 0.001f));
        points_data[i].instance_color = profile_points_color;
      });

      profile_points_list->SetParticleInfos(points_data);
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

  glm::vec2 offset = glm::vec2(0.0f);
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

  if (const auto center_profile_index = profiles.size() * .5f; center_profile_index < profiles.size()) {
    const auto& profile = profiles[center_profile_index];
    for (const auto& point : profile.points) {
      offset += point;
    }
    offset /= profiles[center_profile_index].points.size();
  }

  for (auto& profile : profiles) {
    for (auto& point : profile.points) {
      point -= offset;
      point *= 0.0001f;
      point *= 0.254f;
    }
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

std::vector<glm::vec2> LogScanProfile::BuildBoundary(const JoeScanConfig& joe_scan_config, int target_scan_head) const {
  const auto get_radians = [&](const glm::vec2& a, const glm::vec2& b) {
    const auto radians = glm::acos(glm::clamp(glm::dot(a, b) / (glm::length(a) * glm::length(b)), -1.0f, 1.0f));
    if (const auto det = a.x * b.y - a.y * b.x; det < 0)
      return -radians;
    return radians;
  };

  auto points_min = glm::vec2(FLT_MAX, FLT_MAX);
  auto points_max = glm::vec2(-FLT_MAX, -FLT_MAX);
  for (const auto& i : points) {
    points_min = glm::min(points_min, i);
    points_max = glm::max(points_max, i);
  }
  std::vector<ProfileGrid> grids(1);
  // grids.resize(joe_scan_config.scan_heads.size());

  // for (uint32_t i = 0; i < grids.size(); i++) {
  auto& grid = grids[0];
  grid.Reset(0.001f, points_min - glm::vec2(0.1f), points_max + glm::vec2(0.1f));
  grid.Clear();
  const float rotation_factor = 180.f;
  const auto& scan_head = joe_scan_config.scan_heads[target_scan_head];
  const auto scan_head_position = scan_head.shift * 0.0254f;
  // float head_center_distance = glm::length(scan_head_position);
  const auto scan_head_direction = glm::normalize(-scan_head_position);
  std::map<int, float> rotation_depth{};
  
  for (const auto& point : points) {
    const auto v = point - scan_head_position;
    const auto radians = get_radians(v, scan_head_direction);
    const int rotation = static_cast<int>(radians * rotation_factor);
    const auto depth = glm::length(v);
    if (const auto search = rotation_depth.find(rotation); search != rotation_depth.end() && search->second < depth) {
      search->second = depth;
    } else {
      rotation_depth[rotation] = depth;
    }
  }

  std::vector<std::pair<int, float>> rotation_depth_list;
  rotation_depth_list.reserve(rotation_depth.size());
  for (const auto& it : rotation_depth) {
    rotation_depth_list.emplace_back(it.first, it.second);
  }
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
      const auto a = (rotation - left.first) / static_cast<float>(right.first - left.first);
      if (const auto interpolated_depth = glm::mix(left.second, right.second, a); depth > interpolated_depth) {
        grid.RefCell(static_cast<unsigned>(cell_i)).occluded = true;
      }
    }
  });
  //}

  ProfileGrid final_grid;
  final_grid.Reset(0.001f, points_min - glm::vec2(0.1f), points_max + glm::vec2(0.1f));
  final_grid.Clear();
  std::vector<glm::vec2> ret_val;
  for (uint32_t i = 0; i < final_grid.RefCells().size(); i++) {
    bool occluded = true;
    // for (uint32_t grid_i = 1; grid_i < grids.size(); grid_i++) {
    const auto& grid = grids[0];
    if (!grid.PeekCells()[i].occluded) {
      occluded = false;
      // break;
    }
    //}
    final_grid.RefCells()[i].occluded = occluded;
    if (occluded) {
      ret_val.emplace_back(final_grid.GetPosition(i));
    }
  }
  return ret_val;
}
