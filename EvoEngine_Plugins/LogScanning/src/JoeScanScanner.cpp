#include "JoeScanScanner.hpp"
#include "Json.hpp"
#include "Scene.hpp"
using namespace eco_sys_lab;

void JoeScan::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "m_profiles" << YAML::Value << YAML::BeginSeq;
  for (const auto& profile : m_profiles) {
    out << YAML::BeginMap;
    {
      out << YAML::Key << "m_encoderValue" << YAML::Value << profile.m_encoderValue;
      out << YAML::Key << "m_points" << YAML::Value
          << YAML::Binary(reinterpret_cast<const unsigned char*>(profile.m_points.data()),
                          profile.m_points.size() * sizeof(glm::vec2));
      out << YAML::Key << "m_brightness" << YAML::Value
          << YAML::Binary(reinterpret_cast<const unsigned char*>(profile.m_brightness.data()),
                          profile.m_brightness.size() * sizeof(float));
    }
    out << YAML::EndMap;
  }
}

void JoeScan::Deserialize(const YAML::Node& in) {
  if (in["m_profiles"]) {
    m_profiles.clear();
    for (const auto& inProfile : in["m_profiles"]) {
      m_profiles.emplace_back();
      auto& profile = m_profiles.back();
      if (inProfile["m_encoderValue"])
        profile.m_encoderValue = inProfile["m_encoderValue"].as<float>();
      if (inProfile["m_points"]) {
        const auto inPoints = inProfile["m_points"].as<YAML::Binary>();
        profile.m_points.resize(inPoints.size() / sizeof(glm::vec2));
        std::memcpy(profile.m_points.data(), inPoints.data(), inPoints.size());
      }
      if (inProfile["m_brightness"]) {
        const auto inBrightness = inProfile["m_brightness"].as<YAML::Binary>();
        profile.m_brightness.resize(inBrightness.size() / sizeof(float));
        std::memcpy(profile.m_brightness.data(), inBrightness.data(), inBrightness.size());
      }
    }
  }
}

bool JoeScan::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;
  static std::shared_ptr<ParticleInfoList> joeScanList;
  if (!joeScanList)
    joeScanList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();

  static bool enableJoeScanRendering = true;
  ImGui::Checkbox("Render JoeScan", &enableJoeScanRendering);
  static float divider = 2000.f;
  ImGui::DragFloat("Divider", &divider);

  static auto color = glm::vec3(1);
  static float brightnessFactor = 1.f;
  static bool brightness = true;
  ImGui::Checkbox("Brightness", &brightness);
  if (brightness)
    ImGui::DragFloat("Brightness factor", &brightnessFactor, 0.001f, 0.0f, 1.0f);

  ImGui::ColorEdit3("Color", &color.x);
  if (enableJoeScanRendering) {
    if (ImGui::Button("Refresh JoeScan")) {
      std::vector<ParticleInfo> data;
      for (const auto& profile : m_profiles) {
        const auto startIndex = data.size();
        data.resize(profile.m_points.size() + startIndex);
        Jobs::RunParallelFor(profile.m_points.size(), [&](unsigned i) {
          data[i + startIndex].instance_matrix.SetPosition(glm::vec3(
              profile.m_points[i].x / 10000.f, profile.m_points[i].y / 10000.f, profile.m_encoderValue / divider));
          data[i + startIndex].instance_matrix.SetScale(glm::vec3(0.005f));
          data[i + startIndex].instance_color = glm::vec4(color, 1.f / 128);
          if (brightness)
            data[i + startIndex].instance_color.w =
                static_cast<float>(profile.m_brightness[i]) / 2048.f * brightnessFactor;
        });
      }
      joeScanList->SetParticleInfos(data);
      /*
      const auto scene = Application::GetActiveScene();
      const auto newEntity = scene->CreateEntity("Scan");
      const auto particles = scene->GetOrSetPrivateComponent<Particles>(newEntity).lock();
      const auto material = ProjectManager::CreateTemporaryAsset<Material>();
      particles->m_material = material;
      particles->m_mesh = Resources::GetResource<Mesh>("PRIMITIVE_CUBE");
      particles->m_particleInfoList = joeScanList;
      */
    }
  }

  if (enableJoeScanRendering) {
    GizmoSettings settings{};
    settings.draw_settings.blending = true;
    editorLayer->DrawGizmoCubes(joeScanList, glm::mat4(1), 1.f, settings);
  }

  return changed;
}

void logger(const jsError err, const std::string msg) {
  EVOENGINE_LOG(msg);
  if (0 != err) {
    // If `err` is non-zero, `jsSetupConfigParse` failed parsing or initializing
    // something in the JSON file.
    const char* err_str = nullptr;
    jsGetError(err, &err_str);
    EVOENGINE_ERROR("JoeScan Error (" + std::to_string(err) + "): " + err_str)
  }
}

void JoeScanScanner::StopScanningProcess() {
  if (m_scanEnabled) {
    m_scanEnabled = false;
  } else {
    return;
  }
  if (m_scannerJob.Valid()) {
    Jobs::Wait(m_scannerJob);
    m_scannerJob = {};
    m_points.clear();
    if (const auto joeScan = m_joeScan.Get<JoeScan>()) {
      joeScan->m_profiles.clear();
      for (const auto& profile : m_preservedProfiles) {
        joeScan->m_profiles.emplace_back(profile.second);
      }

      EVOENGINE_LOG("Recorded " + std::to_string(joeScan->m_profiles.size()));
    }
  }
}

void JoeScanScanner::StartScanProcess(const JoeScanScannerSettings& settings) {
  StopScanningProcess();

  m_points.clear();
  m_preservedProfiles.clear();
  const int32_t minPeriod = jsScanSystemGetMinScanPeriod(m_scanSystem);
  if (0 >= minPeriod) {
    EVOENGINE_ERROR("Failed to read min scan period.");
  }

  const int startScanningResult =
      jsScanSystemStartFrameScanning(m_scanSystem, minPeriod, JS_DATA_FORMAT_XY_BRIGHTNESS_FULL);
  if (0 > startScanningResult) {
    EVOENGINE_ERROR("Failed to start scanning.");
    return;
  }

  m_scanEnabled = true;
  m_scannerJob = Jobs::Run([&, settings]() {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
    const auto profileSize = jsScanSystemGetProfilesPerFrame(m_scanSystem);
    int i = 0;
    while (m_scanEnabled) {
      const int scanResult = jsScanSystemWaitUntilFrameAvailable(m_scanSystem, 1000);
      if (0 == scanResult) {
        continue;
      }
      if (0 > scanResult) {
        EVOENGINE_ERROR("Failed to wait for frame.");
        break;
      }

      std::vector<jsProfile> profiles;
      profiles.resize(profileSize);
      const int getFrameResult = jsScanSystemGetFrame(m_scanSystem, profiles.data());
      if (0 >= getFrameResult) {
        EVOENGINE_ERROR("Failed to read frame.");
        break;
      }
      size_t validCount = 0;
      std::vector<glm::vec2> points;
      JoeScanProfile joeScanProfile;
      bool hasEncoderValue = false;
      for (int profileIndex = 0; profileIndex < profileSize; profileIndex++) {
        if (jsRawProfileIsValid(profiles[profileIndex])) {
          bool containRealData = false;
          for (const auto& point : profiles[profileIndex].data) {
            if (point.x != 0 || point.y != 0) {
              containRealData = true;
              points.emplace_back(glm::vec2(point.x, point.y) / 10000.f);
              joeScanProfile.m_points.emplace_back(glm::vec2(point.x, point.y));
              joeScanProfile.m_brightness.emplace_back(point.brightness);
            }
          }
          if (containRealData)
            validCount++;
        }
        if (validCount != 0 && !hasEncoderValue) {
          hasEncoderValue = true;
          joeScanProfile.m_encoderValue = profiles[profileIndex].encoder_values[0];
        }
      }
      if (hasEncoderValue) {
        std::lock_guard lock(*m_scannerMutex);
        m_points = points;
        m_preservedProfiles[joeScanProfile.m_encoderValue / settings.m_step] = joeScanProfile;
      }
      i++;
    }
  });
  Jobs::Execute(m_scannerJob);
}

JoeScanScanner::JoeScanScanner() {
  m_scannerMutex = std::make_shared<std::mutex>();
}

bool JoeScanScanner::InitializeScanSystem(const std::shared_ptr<Json>& json, jsScanSystem& scanSystem,
                                          std::vector<jsScanHead>& scanHeads) {
  try {
    FreeScanSystem(scanSystem, scanHeads);
    if (!json) {
      EVOENGINE_ERROR("JoeScan Error: Json config missing!");
      return false;
    }
    int retVal = joescan::jsSetupConfigParse(json->m_json, scanSystem, scanHeads, &logger);
    if (0 > retVal) {
      // The Scan System and Scan Heads should be assumed to be in an
      // indeterminate state; only action to take is to free the Scan System.
      EVOENGINE_ERROR("JoeScan Error: Configuration failed");
      return false;
    }
    // Scan System and Scan Heads are fully configured.
    EVOENGINE_LOG("JoeScan: Configured successfully");

    retVal = jsScanSystemConnect(scanSystem, 5);
    if (retVal < 0) {
      EVOENGINE_ERROR("JoeScan Error: Connection failed");
      return false;
    }
    EVOENGINE_LOG("JoeScan: Connected to " + std::to_string(retVal) + " heads");

  } catch (const std::exception& e) {
    EVOENGINE_ERROR(e.what());
    return false;
  }
  return true;
}

void JoeScanScanner::FreeScanSystem(jsScanSystem& scanSystem, std::vector<jsScanHead>& scanHeads) {
  try {
    jsScanSystemDisconnect(scanSystem);
    EVOENGINE_LOG("JoeScan: Disconnected " + std::to_string(scanHeads.size()) + " heads");
    scanHeads.clear();
    if (scanSystem != 0) {
      jsScanSystemFree(scanSystem);
    }
    scanSystem = 0;
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(e.what());
    return;
  }
  EVOENGINE_LOG("JoeScan: ScanSysten Freed!");
}

void JoeScanScanner::Serialize(YAML::Emitter& out) const {
  m_config.Save("m_config", out);
  m_joeScan.Save("m_joeScan", out);
}

void JoeScanScanner::Deserialize(const YAML::Node& in) {
  m_config.Load("m_config", in);
  m_joeScan.Load("m_joeScan", in);
}

bool JoeScanScanner::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;
  if (editorLayer->DragAndDropButton<Json>(m_config, "Json Config"))
    changed = true;
  if (editorLayer->DragAndDropButton<JoeScan>(m_joeScan, "JoeScan"))
    changed = true;
  const auto config = m_config.Get<Json>();
  if (config && ImGui::Button("Initialize ScanSystem")) {
    InitializeScanSystem(config, m_scanSystem, m_scanHeads);
  }

  if (m_scanSystem != 0 && ImGui::Button("Free ScanSystem")) {
    FreeScanSystem(m_scanSystem, m_scanHeads);
  }

  ImGui::Separator();
  if (m_scanSystem != 0 && !m_scanEnabled && ImGui::Button("Start Scanning")) {
    std::vector<glm::vec2> results;
    StartScanProcess({});
  }

  if (m_scanEnabled && ImGui::Button("Stop Scanning")) {
    StopScanningProcess();
  }
  static std::shared_ptr<ParticleInfoList> latestPointList;
  if (!latestPointList)
    latestPointList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  if (m_scanEnabled) {
    static bool enableLatestPointRendering = true;
    ImGui::Checkbox("Render Latest Points", &enableLatestPointRendering);
    if (enableLatestPointRendering && !m_points.empty()) {
      std::vector<ParticleInfo> data;
      std::lock_guard lock(*m_scannerMutex);
      data.resize(m_points.size());
      {
        Jobs::RunParallelFor(m_points.size(), [&](unsigned i) {
          data[i].instance_matrix.SetPosition(glm::vec3(m_points[i].x, m_points[i].y, -1.f));
        });
      }
      latestPointList->SetParticleInfos(data);
      editorLayer->DrawGizmoCubes(latestPointList, glm::mat4(1), 0.001f);
    }
  }

  return changed;
}

void JoeScanScanner::FixedUpdate() {
}

void JoeScanScanner::OnCreate() {
}

void JoeScanScanner::OnDestroy() {
}

void JoeScanScanner::CollectAssetRef(std::vector<AssetRef>& list) {
  if (m_config.Get<Json>())
    list.emplace_back(m_config);
}
