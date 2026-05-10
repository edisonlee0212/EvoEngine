#include "LogScan.hpp"
#include "JoeScanScanner.hpp"
#include "Json.hpp"
#include "LogScanReconstruction.hpp"
#include "Prefab.hpp"
#include "Scene.hpp"
using namespace evo_engine;
using namespace log_scanning_plugin;
using namespace nlohmann;

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
        Jobs::RunParallelFor(profile.points.size(), [&](size_t i) {
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
  static int profile_index = 0;
  const auto& profile = profiles[profile_index];
  if (enable_profile_rendering && !profiles.empty()) {
    static LogScanReconstruction reconstruction{};
    static LogScanReconstruction::ReconstructionParameter reconstruction_parameter{};
    if (ImGui::TreeNode("Reconstruction settings")) {
      changed = reconstruction_parameter.OnInspect(editor_layer) || changed;
      ImGui::TreePop();
    }
    if (ImGui::DragInt("Profile Index", &profile_index, 1, 0, profiles.size()) || changed) {
      profile_index = glm::clamp(profile_index, 0, static_cast<int>(profiles.size()));
      std::vector<ParticleInfo> profile_data;
      reconstruction.Initialize(reconstruction_parameter, profile);
      profile_data.resize(reconstruction.processed_points.size());
      Jobs::RunParallelFor(reconstruction.processed_points.size(), [&](size_t i) {
        const auto& processed_point = reconstruction.processed_points[i];
        profile_data[i].instance_matrix.SetPosition(
            glm::vec3(processed_point.position.x, processed_point.position.y, profile.encoder_value));
        profile_data[i].instance_matrix.SetScale(glm::vec3(0.0005f, 0.0005f, 0.0015f));
        profile_data[i].instance_color = processed_point.color;
      });

      profile_list->SetParticleInfos(profile_data);

      std::vector<ParticleInfo> points_data;
      points_data.resize(reconstruction.profile_grid.RefCells().size());
      Jobs::RunParallelFor(points_data.size(), [&](size_t i) {
        const auto& cell = reconstruction.profile_grid.PeekCells()[i];
        const auto position = reconstruction.profile_grid.GetPosition(i);
        points_data[i].instance_matrix.SetPosition(glm::vec3(position.x, position.y, profile.encoder_value));
        points_data[i].instance_matrix.SetScale(glm::vec3(0.003f, 0.003f, 0.001f));
        switch (cell.type) {
          case LogScanReconstruction::CellData::Type::Invalid:
            points_data[i].instance_color = glm::vec4(0, 0, 0, 0.2f);
            break;
          case LogScanReconstruction::CellData::Type::ValidTop:
            points_data[i].instance_color = glm::vec4(1, 0, 1, 0.5);
            break;
          case LogScanReconstruction::CellData::Type::ValidBottom:
            points_data[i].instance_color = glm::vec4(0, 1, 0, 0.5);
            break;
          case LogScanReconstruction::CellData::Type::ValidLeft:
            points_data[i].instance_color = glm::vec4(0, 0, 1, 0.5);
            break;
          case LogScanReconstruction::CellData::Type::ValidRight:
            points_data[i].instance_color = glm::vec4(1, 0, 0, 0.5);
            break;
          case LogScanReconstruction::CellData::Type::Skipped:
            points_data[i].instance_color = glm::vec4(1, 1, 1, .2f);
            break;
        }
      });
      profile_points_list->SetParticleInfos(points_data);
    }
  }
  GizmoSettings settings{};
  settings.draw_settings.blending = true;
  if (enable_joe_scan_rendering) {
    editor_layer->DrawGizmoCubes(joe_scan_list, glm::translate(glm::vec3(0, 0, -profile.encoder_value)), 1.f, settings);
  }
  if (enable_profile_rendering) {
    GizmoSettings settings{};
    settings.draw_settings.blending = true;
    editor_layer->DrawGizmoCubes(profile_points_list, glm::translate(glm::vec3(0, 0, -profile.encoder_value)), 1.f,
                                 settings);
    editor_layer->DrawGizmoCubes(profile_list, glm::translate(glm::vec3(0, 0, -profile.encoder_value)), 1.f, settings);
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
      std::string pair_str = cam_str.append(" ").append(las_str);
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
  const auto scene = ApplicationContext::Get().GetActiveScene();
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
