#include "LogScan.hpp"
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

bool LogScan::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  static std::shared_ptr<ParticleInfoList> joe_scan_list;
  if (!joe_scan_list)
    joe_scan_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();

  static bool enable_joe_scan_rendering = true;
  ImGui::Checkbox("Render LogScan", &enable_joe_scan_rendering);
  static float divider = 3100.f;
  ImGui::DragFloat("Divider", &divider);

  static auto color = glm::vec4(1, 1, 1, 0.01f);
  static float brightness_factor = 1.f;
  static bool brightness = true;
  ImGui::Checkbox("Brightness", &brightness);
  if (brightness)
    ImGui::DragFloat("Brightness factor", &brightness_factor, 0.001f, 0.0f, 1.0f);

  ImGui::ColorEdit4("Color", &color.x);
  if (enable_joe_scan_rendering) {
    if (ImGui::Button("Refresh LogScan")) {
      std::vector<ParticleInfo> data;
      for (const auto& profile : profiles) {
        const auto start_index = data.size();
        data.resize(profile.points.size() + start_index);
        Jobs::RunParallelFor(profile.points.size(), [&](unsigned i) {
          data[i + start_index].instance_matrix.SetPosition(
              glm::vec3(profile.points[i].x / 10000.f, profile.points[i].y / 10000.f, profile.encoder_value / divider));
          data[i + start_index].instance_matrix.SetScale(glm::vec3(0.005f));
          data[i + start_index].instance_color = color;
          if (brightness)
            data[i + start_index].instance_color.w =
                static_cast<float>(profile.brightness[i]) / 2048.f * brightness_factor;
        });
      }
      joe_scan_list->SetParticleInfos(data);
      /*
      const auto scene = Application::GetActiveScene();
      const auto newEntity = scene->CreateEntity("Scan");
      const auto particles = scene->GetOrSetPrivateComponent<Particles>(newEntity).lock();
      const auto material = ProjectManager::CreateTemporaryAsset<Material>();
      particles->material = material;
      particles->mesh = Resources::GetResource<Mesh>("PRIMITIVE_CUBE");
      particles->particle_info_list = joeScanList;
      */
    }
  }

  if (enable_joe_scan_rendering) {
    GizmoSettings settings{};
    settings.draw_settings.blending = true;
    editor_layer->DrawGizmoCubes(joe_scan_list, glm::mat4(1), 1.f, settings);
  }

  return changed;
}