#include "JoeScanScanner.hpp"
#include "Json.hpp"
#include "Scene.hpp"
using namespace log_scanning_plugin;

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
  if (scan_enabled_) {
    scan_enabled_ = false;
  } else {
    return;
  }
  if (scanner_job_.Valid()) {
    Jobs::Wait(scanner_job_);
    scanner_job_ = {};
    points_.clear();
    if (const auto joeScan = log_scan.Get<LogScan>()) {
      joeScan->profiles.clear();
      for (const auto& profile : preserved_profiles_) {
        joeScan->profiles.emplace_back(profile.second);
      }

      EVOENGINE_LOG("Recorded " + std::to_string(joeScan->profiles.size()));
    }
  }
}

void JoeScanScanner::StartScanProcess(const JoeScanScannerSettings& settings) {
  StopScanningProcess();

  points_.clear();
  preserved_profiles_.clear();
  const int32_t min_period = jsScanSystemGetMinScanPeriod(scan_system);
  if (0 >= min_period) {
    EVOENGINE_ERROR("Failed to read min scan period.");
  }

  if (const int start_scanning_result =
      jsScanSystemStartFrameScanning(scan_system, min_period, JS_DATA_FORMAT_XY_BRIGHTNESS_FULL); 0 > start_scanning_result) {
    EVOENGINE_ERROR("Failed to start scanning.");
    return;
  }

  scan_enabled_ = true;
  scanner_job_ = Jobs::Run([&, settings]() {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
    const auto profile_size = jsScanSystemGetProfilesPerFrame(scan_system);
    int i = 0;
    while (scan_enabled_) {
      const int scan_result = jsScanSystemWaitUntilFrameAvailable(scan_system, 1000);
      if (0 == scan_result) {
        continue;
      }
      if (0 > scan_result) {
        EVOENGINE_ERROR("Failed to wait for frame.");
        break;
      }

      std::vector<jsProfile> profiles;
      profiles.resize(profile_size);
      if (const int get_frame_result = jsScanSystemGetFrame(scan_system, profiles.data()); 0 >= get_frame_result) {
        EVOENGINE_ERROR("Failed to read frame.");
        break;
      }
      size_t valid_count = 0;
      std::vector<glm::vec2> points;
      LogScanProfile joe_scan_profile;
      bool has_encoder_value = false;
      for (int profile_index = 0; profile_index < profile_size; profile_index++) {
        if (jsRawProfileIsValid(profiles[profile_index])) {
          bool contain_real_data = false;
          for (const auto& point : profiles[profile_index].data) {
            if (point.x != 0 || point.y != 0) {
              contain_real_data = true;
              points.emplace_back(glm::vec2(point.x, point.y) / 10000.f);
              joe_scan_profile.points.emplace_back(point.x, point.y);
              joe_scan_profile.brightness.emplace_back(point.brightness);
            }
          }
          if (contain_real_data)
            valid_count++;
        }
        if (valid_count != 0 && !has_encoder_value) {
          has_encoder_value = true;
          joe_scan_profile.encoder_value = profiles[profile_index].encoder_values[0];
        }
      }
      if (has_encoder_value) {
        std::lock_guard lock(*scanner_mutex_);
        points_ = points;
        preserved_profiles_[joe_scan_profile.encoder_value / settings.step] = joe_scan_profile;
      }
      i++;
    }
  });
  Jobs::Execute(scanner_job_);
}

JoeScanScanner::JoeScanScanner() {
  scanner_mutex_ = std::make_shared<std::mutex>();
}

bool JoeScanScanner::InitializeScanSystem(const std::shared_ptr<Json>& json, jsScanSystem& scan_system,
                                          std::vector<jsScanHead>& scan_heads) {
  try {
    FreeScanSystem(scan_system, scan_heads);
    if (!json) {
      EVOENGINE_ERROR("JoeScan Error: Json config missing!");
      return false;
    }
    int ret_val = joescan::jsSetupConfigParse(json->m_json, scan_system, scan_heads, &logger);
    if (0 > ret_val) {
      // The Scan System and Scan Heads should be assumed to be in an
      // indeterminate state; only action to take is to free the Scan System.
      EVOENGINE_ERROR("JoeScan Error: Configuration failed");
      FreeScanSystem(scan_system, scan_heads);
      return false;
    }
    // Scan System and Scan Heads are fully configured.
    EVOENGINE_LOG("JoeScan: Configured successfully");

    ret_val = jsScanSystemConnect(scan_system, 5);
    if (ret_val < 0) {
      EVOENGINE_ERROR("JoeScan Error: Connection failed");
      FreeScanSystem(scan_system, scan_heads);
      return false;
    }
    EVOENGINE_LOG("JoeScan: Connected to " + std::to_string(ret_val) + " heads");

  } catch (const std::exception& e) {
    EVOENGINE_ERROR(e.what());
    return false;
  }
  return true;
}

void JoeScanScanner::FreeScanSystem(jsScanSystem& scan_system, std::vector<jsScanHead>& scan_heads) {
  try {
    jsScanSystemDisconnect(scan_system);
    EVOENGINE_LOG("JoeScan: Disconnected " + std::to_string(scan_heads.size()) + " heads");
    scan_heads.clear();
    if (scan_system != 0) {
      jsScanSystemFree(scan_system);
    }
    scan_system = 0;
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(e.what());
    return;
  }
  EVOENGINE_LOG("JoeScan: ScanSysten Freed!");
}

void JoeScanScanner::Serialize(YAML::Emitter& out) const {
  config.Save("config", out);
  log_scan.Save("log_scan", out);
}

void JoeScanScanner::Deserialize(const YAML::Node& in) {
  config.Load("config", in);
  log_scan.Load("log_scan", in);
}

bool JoeScanScanner::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (editor_layer->DragAndDropButton<Json>(config, "Json Config"))
    changed = true;
  if (editor_layer->DragAndDropButton<LogScan>(log_scan, "LogScan"))
    changed = true;

  if (const auto json_config = this->config.Get<Json>(); json_config && ImGui::Button("Initialize ScanSystem")) {
    InitializeScanSystem(json_config, scan_system, scan_heads);
  }

  ImGui::Separator();
  if (scan_system != 0 && !scan_enabled_ && ImGui::Button("Start Scanning")) {
    std::vector<glm::vec2> results;
    StartScanProcess({});
  }

  if (scan_enabled_ && ImGui::Button("Stop Scanning")) {
    StopScanningProcess();
    FreeScanSystem(scan_system, scan_heads);
  }
  static std::shared_ptr<ParticleInfoList> latest_point_list;
  if (!latest_point_list)
    latest_point_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  if (scan_enabled_) {
    static bool enable_latest_point_rendering = true;
    ImGui::Checkbox("Render Latest Points", &enable_latest_point_rendering);
    if (enable_latest_point_rendering && !points_.empty()) {
      std::vector<ParticleInfo> data;
      std::lock_guard lock(*scanner_mutex_);
      data.resize(points_.size());
      {
        Jobs::RunParallelFor(points_.size(), [&](unsigned i) {
          data[i].instance_matrix.SetPosition(glm::vec3(points_[i].x, points_[i].y, -1.f));
        });
      }
      latest_point_list->SetParticleInfos(data);
      editor_layer->DrawGizmoCubes(latest_point_list, glm::mat4(1), 0.001f);
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
  if (config.Get<Json>())
    list.emplace_back(config);
  if (log_scan.Get<LogScan>())
    list.emplace_back(log_scan);
}
