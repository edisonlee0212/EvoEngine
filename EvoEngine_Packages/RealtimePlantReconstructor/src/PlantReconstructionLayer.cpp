#include "PlantReconstructionLayer.hpp"

#include "Application.hpp"
#include "AssetManager.hpp"
#include "Json.hpp"
#include "ProjectManager.hpp"
#include "Scene.hpp"
#include "Transform.hpp"

#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <random>
#include <sstream>
#include <stdexcept>

using namespace realtime_plant_reconstructor;

namespace {
std::filesystem::path FindDefaultScaleRoot() {
  const std::filesystem::path fixed = "C:/AlexC/02_skellies/03_SCALEtonization";
  if (std::filesystem::exists(fixed)) {
    return fixed;
  }

  std::filesystem::path cursor = std::filesystem::current_path();
  while (!cursor.empty()) {
    const auto candidate = cursor / "03_SCALEtonization";
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
    const auto parent = cursor.parent_path();
    if (parent == cursor) {
      break;
    }
    cursor = parent;
  }
  return fixed;
}

std::string JsonString(const nlohmann::json& json, const char* key) {
  const auto it = json.find(key);
  return it != json.end() && it->is_string() ? it->get<std::string>() : std::string();
}

float JsonFloat(const nlohmann::json& json, const char* key) {
  const auto it = json.find(key);
  return it != json.end() && it->is_number() ? it->get<float>() : 0.0f;
}

bool LoadJsonFile(const std::filesystem::path& path, nlohmann::json& json) {
  std::ifstream stream(path);
  if (!stream) {
    return false;
  }
  std::string text((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xEF && static_cast<unsigned char>(text[1]) == 0xBB &&
      static_cast<unsigned char>(text[2]) == 0xBF) {
    text.erase(0, 3);
  }
  json = nlohmann::json::parse(text, nullptr, false);
  return !json.is_discarded();
}

std::filesystem::file_time_type LastWriteTime(const std::filesystem::path& path) {
  std::error_code error;
  const auto time = std::filesystem::last_write_time(path, error);
  return error ? std::filesystem::file_time_type{} : time;
}

std::string StemString(const std::filesystem::path& path) {
  return path.stem().string();
}

std::string SafeAssetStem(std::string stem) {
  for (auto& character : stem) {
    const auto value = static_cast<unsigned char>(character);
    if (!std::isalnum(value) && character != '-' && character != '_') {
      character = '_';
    }
  }
  if (stem.empty()) {
    return "PointCloud";
  }
  return stem;
}

std::string ReadString(const YAML::Node& in, const char* key, const std::string& fallback = {}) {
  const auto node = in[key];
  return node && node.IsScalar() ? node.as<std::string>() : fallback;
}

float ReadFloat(const YAML::Node& in, const char* key, const float fallback) {
  const auto node = in[key];
  return node && node.IsScalar() ? node.as<float>() : fallback;
}

int ReadInt(const YAML::Node& in, const char* key, const int fallback) {
  const auto node = in[key];
  return node && node.IsScalar() ? node.as<int>() : fallback;
}

bool ReadBool(const YAML::Node& in, const char* key, const bool fallback) {
  const auto node = in[key];
  return node && node.IsScalar() ? node.as<bool>() : fallback;
}

bool IsPlyFile(const std::filesystem::path& path) {
  const auto extension = path.extension().string();
  return extension == ".ply" || extension == ".PLY";
}

bool HasPathSegment(const std::filesystem::path& path, const std::string& segment) {
  for (const auto& part : path) {
    if (part.string() == segment) {
      return true;
    }
  }
  return false;
}

size_t ReadSizeT(const YAML::Node& in, const char* key, const size_t fallback) {
  const auto node = in[key];
  return node && node.IsScalar() ? node.as<size_t>() : fallback;
}

std::filesystem::path ReadAbsolutePath(const YAML::Node& in, const char* key) {
  const auto node = in[key];
  if (!node || !node.IsScalar()) {
    return {};
  }
  return std::filesystem::absolute(node.as<std::string>());
}

std::string ResultSafe(std::string value) {
  std::replace(value.begin(), value.end(), '"', '\'');
  return value;
}

struct RawKnnVideoConfig {
  std::filesystem::path output_dir;
  int width = 1920;
  int height = 1080;
  int fps = 60;
  size_t raw_frame_count = 300;
  size_t knn_frame_count = 300;
  size_t warmup_frames = 8;
  size_t max_load_frames = 30000;
  unsigned png_compression_level = 1;
};

RawKnnVideoConfig LoadRawKnnVideoConfig(const YAML::Node& root) {
  RawKnnVideoConfig config;
  config.output_dir = ReadAbsolutePath(root, "output_dir");
  if (config.output_dir.empty()) {
    throw std::invalid_argument("raw_knn_video requires output_dir.");
  }
  config.width = std::max(1, ReadInt(root, "width", config.width));
  config.height = std::max(1, ReadInt(root, "height", config.height));
  config.fps = std::max(1, ReadInt(root, "fps", config.fps));
  config.raw_frame_count = std::max<size_t>(1, ReadSizeT(root, "raw_frame_count", config.raw_frame_count));
  config.knn_frame_count = std::max<size_t>(1, ReadSizeT(root, "knn_frame_count", config.knn_frame_count));
  config.warmup_frames = ReadSizeT(root, "warmup_frames", config.warmup_frames);
  config.max_load_frames = std::max<size_t>(1, ReadSizeT(root, "max_load_frames", config.max_load_frames));
  config.png_compression_level = static_cast<unsigned>(
      std::clamp(ReadInt(root, "png_compression_level", static_cast<int>(config.png_compression_level)), 0, 9));
  return config;
}

void SetCenteredYawTransform(const std::shared_ptr<Scene>& scene, const Entity& entity, const glm::vec3& center,
                             const float angle_radians) {
  if (!scene || !scene->IsEntityValid(entity)) {
    return;
  }
  Transform transform;
  transform.value = glm::translate(center) * glm::mat4_cast(glm::angleAxis(angle_radians, glm::vec3(0, 1, 0))) *
                    glm::translate(-center);
  scene->SetDataComponent(entity, transform);
  GlobalTransform global_transform;
  global_transform.value = transform.value;
  scene->SetDataComponent(entity, global_transform);
}

}  // namespace

void PlantReconstructionLayer::OnCreate() {
  SetScaleRoot(FindDefaultScaleRoot());
  RefreshInputFolders();
}

void PlantReconstructionLayer::Update() {
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  DrawSubjectPreviews(editor_layer);
}

bool PlantReconstructionLayer::SupportsProjectStateSerialization() const {
  return true;
}

void PlantReconstructionLayer::SerializeProjectState(YAML::Emitter& out) const {
  out << YAML::Key << "scale_root" << YAML::Value << ScaleRoot().string();

  out << YAML::Key << "display" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "show_preview" << YAML::Value << show_preview_;
  out << YAML::Key << "point_size_mm" << YAML::Value << point_size_mm_;
  out << YAML::Key << "point_size_multiplier" << YAML::Value << point_size_multiplier_;
  out << YAML::Key << "point_opacity" << YAML::Value << point_opacity_;
  out << YAML::Key << "point_color" << YAML::Value << glm::vec3(point_color_[0], point_color_[1], point_color_[2]);
  out << YAML::Key << "max_preview_points" << YAML::Value << max_preview_points_;
  out << YAML::Key << "use_ply_colors" << YAML::Value << use_ply_colors_;
  out << YAML::EndMap;

  out << YAML::Key << "neighborhood_defaults" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "k" << YAML::Value << neighborhood_k_;
  out << YAML::Key << "max_points" << YAML::Value << max_neighborhood_points_;
  out << YAML::EndMap;

  std::set<std::string> checked_paths = saved_checked_input_folders_;
  if (!input_folders_.empty()) {
    checked_paths.clear();
    for (const auto& folder : input_folders_) {
      if (folder.checked) {
        checked_paths.insert(NormalizeInputFolderPath(folder.path));
      }
    }
  }
  out << YAML::Key << "checked_input_folders" << YAML::Value << YAML::BeginSeq;
  for (const auto& path : checked_paths) {
    out << path;
  }
  out << YAML::EndSeq;
}

void PlantReconstructionLayer::DeserializeProjectState(const YAML::Node& in) {
  if (const auto scale_root = in["scale_root"]; scale_root && scale_root.IsScalar()) {
    SetScaleRoot(scale_root.as<std::string>());
  }
  if (const auto display = in["display"]) {
    show_preview_ = ReadBool(display, "show_preview", show_preview_);
    point_size_mm_ = ReadFloat(display, "point_size_mm", point_size_mm_);
    point_size_multiplier_ = ReadFloat(display, "point_size_multiplier", point_size_multiplier_);
    point_opacity_ = ReadFloat(display, "point_opacity", point_opacity_);
    if (display["point_color"]) {
      const auto color = display["point_color"].as<glm::vec3>();
      point_color_[0] = color.x;
      point_color_[1] = color.y;
      point_color_[2] = color.z;
    }
    max_preview_points_ = ReadInt(display, "max_preview_points", max_preview_points_);
    use_ply_colors_ = ReadBool(display, "use_ply_colors", use_ply_colors_);
  }
  if (const auto neighborhood = in["neighborhood_defaults"]) {
    neighborhood_k_ = ReadInt(neighborhood, "k", neighborhood_k_);
    max_neighborhood_points_ = ReadInt(neighborhood, "max_points", max_neighborhood_points_);
  }

  saved_checked_input_folders_.clear();
  const auto checked = in["checked_input_folders"];
  input_folder_selection_loaded_ = checked && checked.IsSequence();
  if (input_folder_selection_loaded_) {
    for (const auto& path : checked) {
      if (path.IsScalar()) {
        saved_checked_input_folders_.insert(NormalizeInputFolderPath(path.as<std::string>()));
      }
    }
  }
  RefreshInputFolders();
}

bool PlantReconstructionLayer::SupportsLayerAutomationMode(const std::string& mode) const {
  return mode == "raw_knn_video";
}

int PlantReconstructionLayer::RunLayerAutomation(const YAML::Node& config) {
  const auto mode = ReadString(config, "mode");
  if (!SupportsLayerAutomationMode(mode)) {
    std::cerr << "RPR_VIDEO_RESULT failed reason=\"unsupported mode: " << ResultSafe(mode) << "\"" << std::endl;
    return 1;
  }

  const auto video_config = LoadRawKnnVideoConfig(config);
  auto& application = ApplicationContext::Get();
  const auto fail = [&](const std::string& reason) {
    application.End();
    std::cerr << "RPR_VIDEO_RESULT failed reason=\"" << ResultSafe(reason) << "\"" << std::endl;
    return 1;
  };

  application.Start(false);
  for (size_t frame = 0; !ProjectManager::IsProjectIdle(); ++frame) {
    if (frame >= video_config.max_load_frames) {
      return fail("project load timed out");
    }
    if (!application.Loop()) {
      return fail("application ended before project load completed");
    }
  }

  const auto editor_layer = application.GetLayer<EditorLayer>();
  if (!editor_layer) {
    return fail("editor layer is missing");
  }
  const auto scene = application.GetActiveScene();
  if (!scene) {
    return fail("active scene is missing");
  }
  const auto scene_camera = editor_layer->GetSceneCamera();
  if (!scene_camera || !scene_camera->GetRenderTexture()) {
    return fail("scene camera render texture is missing");
  }

  RefreshInputFolders();
  if (GetCheckedInputFolderCount() == 0) {
    return fail("no input folders are checked");
  }
  if (GetCheckedInputCandidateCount() == 0) {
    return fail("checked input folders have no raw point-cloud PLY candidates");
  }

  const auto seed = static_cast<uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
  const auto import_result = ImportRandomCheckedPointCloud(seed, false);
  if (!import_result.success || !scene->IsEntityValid(import_result.entity)) {
    return fail(import_result.message.empty() ? "failed to import random checked point cloud" : import_result.message);
  }
  const auto subject = scene->GetOrSetPrivateComponent<PlantReconstructionSubject>(import_result.entity).lock();
  if (!subject) {
    return fail("imported entity has no PlantReconstructionSubject");
  }

  Bound local_bound;
  if (!subject->GetLocalBound(local_bound)) {
    return fail("imported point cloud has no valid bounds");
  }
  const auto rotation_center = local_bound.Center();
  editor_layer->SetSelectedEntity(import_result.entity, false);
  scene_camera->Resize({static_cast<uint32_t>(video_config.width), static_cast<uint32_t>(video_config.height)});
  if (!editor_layer->FrameEntityInSceneCamera(import_result.entity, 0.0f)) {
    return fail("failed to frame imported point cloud");
  }

  const auto frame_dir = video_config.output_dir / "frames";
  std::filesystem::create_directories(frame_dir);

  nlohmann::json frames = nlohmann::json::array();
  size_t frame_number = 0;
  const auto loop_once = [&]() {
    scene_camera->Resize({static_cast<uint32_t>(video_config.width), static_cast<uint32_t>(video_config.height)});
    return application.Loop();
  };
  const auto warmup = [&]() {
    for (size_t frame = 0; frame < video_config.warmup_frames; ++frame) {
      if (!loop_once()) {
        return false;
      }
    }
    return true;
  };
  const auto capture_segment = [&](const char* segment, const size_t frame_count) {
    if (!warmup()) {
      return false;
    }
    constexpr float two_pi = 6.2831853071795864769f;
    for (size_t frame = 0; frame < frame_count; ++frame) {
      const auto angle = two_pi * static_cast<float>(frame) / static_cast<float>(frame_count);
      SetCenteredYawTransform(scene, import_result.entity, rotation_center, angle);
      if (!loop_once()) {
        return false;
      }
      std::stringstream name;
      name << "frame_" << std::setw(6) << std::setfill('0') << frame_number << ".png";
      const auto frame_path = frame_dir / name.str();
      scene_camera->GetRenderTexture()->StoreToPng(frame_path, video_config.width, video_config.height,
                                                   video_config.png_compression_level);
      frames.push_back(
          {{"index", frame_number}, {"segment", segment}, {"segment_index", frame}, {"path", frame_path.string()}});
      ++frame_number;
    }
    return true;
  };

  subject->show_neighborhood_density = false;
  subject->MarkPreviewDirty();
  subject->RebuildPreview();
  if (!capture_segment("raw", video_config.raw_frame_count)) {
    return fail("application ended while capturing raw segment");
  }

  if (!subject->BuildNeighborhoodField()) {
    return fail(subject->GetNeighborhoodStatus());
  }
  subject->show_neighborhood_density = true;
  subject->MarkPreviewDirty();
  subject->RebuildPreview();
  if (!capture_segment("knn", video_config.knn_frame_count)) {
    return fail("application ended while capturing KNN segment");
  }

  nlohmann::json manifest;
  manifest["mode"] = "raw_knn_video";
  manifest["seed"] = seed;
  manifest["source_path"] = import_result.source_path.string();
  manifest["sample_name"] = import_result.sample_name;
  manifest["total_point_count"] = import_result.total_point_count;
  manifest["displayed_point_count"] = import_result.displayed_point_count;
  manifest["width"] = video_config.width;
  manifest["height"] = video_config.height;
  manifest["fps"] = video_config.fps;
  manifest["raw_frame_count"] = video_config.raw_frame_count;
  manifest["knn_frame_count"] = video_config.knn_frame_count;
  manifest["frames"] = frames;

  const auto manifest_path = video_config.output_dir / "manifest.json";
  std::ofstream manifest_out(manifest_path);
  manifest_out << manifest.dump(2);
  manifest_out << std::endl;

  if (scene->IsEntityValid(import_result.entity)) {
    scene->DeleteEntity(import_result.entity);
  }
  application.End();
  std::cout << "RPR_VIDEO_RESULT passed manifest=\"" << manifest_path.string() << "\" frames=" << frame_number
            << std::endl;
  return 0;
}

bool PlantReconstructionLayer::TryGetEntityEditorBound(const std::shared_ptr<Scene>& scene, const Entity& entity,
                                                       Bound& bound) const {
  if (!scene || !scene->IsEntityValid(entity) || !scene->HasPrivateComponent<PlantReconstructionSubject>(entity)) {
    return false;
  }
  const auto subject = scene->GetOrSetPrivateComponent<PlantReconstructionSubject>(entity).lock();
  return subject && subject->GetLocalBound(bound);
}

bool PlantReconstructionLayer::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool open = enable_inspection;
  bool changed = false;
  if (!layout_seeded_) {
    ImGui::SetNextWindowPos(ImVec2(880.0f, 80.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(380.0f, 520.0f), ImGuiCond_FirstUseEver);
    layout_seeded_ = true;
  }
  if (ImGui::Begin(GetLayerName().c_str(), &open)) {
    changed |= ImGui::InputText("SCALEtonization Root", scale_root_buffer_.data(), scale_root_buffer_.size());
    if (ImGui::Button("Import Grid")) {
      ImportGrid();
    }
    ImGui::SameLine();
    if (ImGui::Button("Refresh Input Folders")) {
      RefreshInputFolders();
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
      candidates_.clear();
      input_folders_.clear();
      selected_candidate_index_ = -1;
      status_ = "Cleared candidate list. Imported scene entities were left unchanged.";
    }

    if (ImGui::TreeNodeEx("Input Folders", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (input_folders_.empty()) {
        ImGui::TextWrapped("No valid SCALEtonization input folders were found.");
      }
      bool selection_changed = false;
      for (auto& folder : input_folders_) {
        const auto label = folder.name + " (" + std::to_string(folder.candidate_count) + ")";
        selection_changed |= ImGui::Checkbox(label.c_str(), &folder.checked);
      }
      if (selection_changed) {
        StoreCheckedInputFolderSelection();
      }
      ImGui::TreePop();
    }

    if (!candidates_.empty()) {
      selected_candidate_index_ = std::clamp(selected_candidate_index_, 0, static_cast<int>(candidates_.size() - 1));
      const auto current_label = CandidateLabel(candidates_[selected_candidate_index_]);
      if (ImGui::BeginCombo("Candidate", current_label.c_str())) {
        for (int i = 0; i < static_cast<int>(candidates_.size()); ++i) {
          const bool selected = i == selected_candidate_index_;
          if (ImGui::Selectable(CandidateLabel(candidates_[i]).c_str(), selected)) {
            selected_candidate_index_ = i;
          }
          if (selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
    }

    bool style_changed = false;
    style_changed |= ImGui::Checkbox("Show Preview", &show_preview_);
    style_changed |= ImGui::DragFloat("Point Size (mm)", &point_size_mm_, 1.0f, 0.1f, 10000.0f);
    style_changed |= ImGui::DragFloat("Point Size Multiplier", &point_size_multiplier_, 0.01f, 0.0f, 10.0f);
    style_changed |= ImGui::DragFloat("Point Opacity", &point_opacity_, 0.01f, 0.0f, 1.0f);
    style_changed |= ImGui::ColorEdit3("Point Color", point_color_);
    style_changed |= ImGui::Checkbox("Use PLY Colors", &use_ply_colors_);
    style_changed |= ImGui::DragInt("Max Preview Points", &max_preview_points_, 1000, 1000, 5000000);
    style_changed |= ImGui::DragInt("Default K", &neighborhood_k_, 1, 1, 64);
    style_changed |= ImGui::DragInt("Default Max KNN Points", &max_neighborhood_points_, 1000, 1000, 500000);
    if (style_changed && editor_layer) {
      const auto scene = ApplicationContext::Get().GetActiveScene();
      const auto selected_entity = editor_layer->GetSelectedEntity();
      if (scene && scene->IsEntityValid(selected_entity) &&
          scene->HasPrivateComponent<PlantReconstructionSubject>(selected_entity)) {
        if (const auto subject = scene->GetOrSetPrivateComponent<PlantReconstructionSubject>(selected_entity).lock()) {
          ApplyDefaultsToSubject(*subject);
        }
      }
    }
    changed |= style_changed;

    ImGui::TextWrapped("Status: %s", status_.c_str());
    ImGui::Text("Input folders: %d checked / %d total", static_cast<int>(GetCheckedInputFolderCount()),
                static_cast<int>(input_folders_.size()));
    ImGui::Text("Checked candidates: %d", static_cast<int>(GetCheckedInputCandidateCount()));
    if (editor_layer) {
      const auto scene = ApplicationContext::Get().GetActiveScene();
      const auto selected_entity = editor_layer->GetSelectedEntity();
      if (scene && scene->IsEntityValid(selected_entity) &&
          scene->HasPrivateComponent<PlantReconstructionSubject>(selected_entity)) {
        if (const auto subject = scene->GetOrSetPrivateComponent<PlantReconstructionSubject>(selected_entity).lock()) {
          ImGui::Separator();
          ImGui::Text("Selected point-cloud entity");
          ImGui::Text("Points: %zu / %zu", subject->GetDisplayedPointCount(), subject->GetTotalPointCount());
          ImGui::TextWrapped("Source: %s", subject->GetImportMetadata().source_path.string().c_str());
        }
      }
    }
  }
  ImGui::End();
  enable_inspection = open;
  return changed;
}

std::filesystem::path PlantReconstructionLayer::ScaleRoot() const {
  return std::filesystem::path(std::string(scale_root_buffer_.data()));
}

void PlantReconstructionLayer::SetScaleRoot(const std::filesystem::path& path) {
  const auto text = path.string();
  std::memset(scale_root_buffer_.data(), 0, scale_root_buffer_.size());
  std::strncpy(scale_root_buffer_.data(), text.c_str(), scale_root_buffer_.size() - 1);
}

void PlantReconstructionLayer::RefreshInputFolders() {
  const auto input_root = ScaleRoot() / "input";
  input_folders_.clear();
  if (!std::filesystem::exists(input_root)) {
    status_ = "SCALEtonization input folder not found: " + input_root.string();
    return;
  }

  std::error_code error;
  for (const auto& entry : std::filesystem::directory_iterator(
           input_root, std::filesystem::directory_options::skip_permission_denied, error)) {
    if (error) {
      break;
    }
    std::error_code entry_error;
    if (!entry.is_directory(entry_error) || !HasInputFolderTransform(entry.path())) {
      continue;
    }
    std::vector<PointCloudCandidate> folder_candidates;
    AddInputFolderCandidates(entry.path(), folder_candidates);
    if (folder_candidates.empty()) {
      continue;
    }

    InputFolderInfo folder;
    folder.path = entry.path();
    folder.name = entry.path().filename().string();
    folder.candidate_count = folder_candidates.size();
    const auto normalized_path = NormalizeInputFolderPath(folder.path);
    folder.checked = !input_folder_selection_loaded_ ||
                     saved_checked_input_folders_.find(normalized_path) != saved_checked_input_folders_.end();
    input_folders_.push_back(folder);
  }

  std::sort(input_folders_.begin(), input_folders_.end(), [](const InputFolderInfo& a, const InputFolderInfo& b) {
    return a.name < b.name;
  });

  if (!input_folder_selection_loaded_) {
    StoreCheckedInputFolderSelection();
  }
  status_ = "Found " + std::to_string(input_folders_.size()) + " importable input folder(s).";
}

size_t PlantReconstructionLayer::GetCheckedInputFolderCount() const {
  return static_cast<size_t>(
      std::count_if(input_folders_.begin(), input_folders_.end(), [](const InputFolderInfo& folder) {
        return folder.checked;
      }));
}

size_t PlantReconstructionLayer::GetCheckedInputCandidateCount() const {
  size_t count = 0;
  for (const auto& folder : input_folders_) {
    if (folder.checked) {
      count += folder.candidate_count;
    }
  }
  return count;
}

const std::string& PlantReconstructionLayer::GetStatus() const {
  return status_;
}

PlantReconstructionLayer::ImportResult PlantReconstructionLayer::ImportRandomCheckedPointCloud(const uint64_t seed,
                                                                                               const bool save_asset) {
  if (input_folders_.empty()) {
    RefreshInputFolders();
  }
  const auto checked_candidates = BuildCheckedInputCandidates();
  if (checked_candidates.empty()) {
    status_ = "No raw point-cloud candidates in checked input folders.";
    ImportResult result;
    result.message = status_;
    return result;
  }

  std::mt19937_64 rng(seed);
  std::uniform_int_distribution<size_t> pick(0, checked_candidates.size() - 1);
  auto result = ImportCandidate(checked_candidates[pick(rng)], save_asset);
  if (result.success) {
    result.message += " seed=" + std::to_string(seed);
  }
  return result;
}

void PlantReconstructionLayer::ImportGrid() {
  const auto seed = static_cast<uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
  const auto result = ImportRandomCheckedPointCloud(seed, true);
  if (result.success) {
    status_ += " Seed: " + std::to_string(seed) + ".";
  }
}

void PlantReconstructionLayer::DiscoverCandidates() {
  candidates_.clear();
  const auto root = ScaleRoot();
  const std::array roots{root / "o", root / "output" / "experiments"};
  for (const auto& experiments_root : roots) {
    if (!std::filesystem::exists(experiments_root)) {
      continue;
    }
    std::error_code error;
    for (std::filesystem::recursive_directory_iterator
             it(experiments_root, std::filesystem::directory_options::skip_permission_denied, error),
         end;
         !error && it != end; it.increment(error)) {
      if (it->is_regular_file(error) && it->path().filename() == "run_manifest.json") {
        AddManifestCandidates(it->path());
      }
    }
  }
  AddInputCandidates(root / "input");

  std::sort(candidates_.begin(), candidates_.end(), [](const PointCloudCandidate& a, const PointCloudCandidate& b) {
    if (a.priority != b.priority) {
      return a.priority < b.priority;
    }
    return a.modified_time > b.modified_time;
  });

  status_ = "Discovered " + std::to_string(candidates_.size()) + " raw point-cloud candidate(s).";
}

void PlantReconstructionLayer::AddManifestCandidates(const std::filesystem::path& run_manifest_path) {
  nlohmann::json run_manifest;
  if (!LoadJsonFile(run_manifest_path, run_manifest)) {
    return;
  }
  const auto method_name = JsonString(run_manifest, "method_name");
  const auto samples = run_manifest.find("samples");
  if (samples == run_manifest.end() || !samples->is_array()) {
    return;
  }

  for (const auto& sample : *samples) {
    if (JsonString(sample, "status") != "completed") {
      continue;
    }
    const auto asset_manifest_path = std::filesystem::path(JsonString(sample, "asset_manifest_path"));
    if (asset_manifest_path.empty()) {
      continue;
    }
    nlohmann::json asset_manifest;
    if (!LoadJsonFile(asset_manifest_path, asset_manifest)) {
      continue;
    }
    const auto assets = asset_manifest.find("assets");
    if (assets == asset_manifest.end() || !assets->is_array()) {
      continue;
    }
    for (const auto& asset : *assets) {
      if (JsonString(asset, "kind") != "ply" || JsonString(asset, "role") != "point_cloud") {
        continue;
      }
      PointCloudCandidate candidate;
      candidate.path = JsonString(asset, "path");
      if (!std::filesystem::exists(candidate.path)) {
        continue;
      }
      candidate.manifest_path = asset_manifest_path;
      candidate.sample_name = JsonString(sample, "sample_name");
      candidate.source_name = method_name.empty() ? run_manifest_path.parent_path().filename().string() : method_name;
      candidate.source_kind = "manifest raw";
      candidate.role = "point_cloud";
      candidate.stage = JsonString(asset, "stage");
      candidate.priority = 0;
      candidate.modified_time = LastWriteTime(run_manifest_path);
      const auto hints = asset.find("view_hints");
      if (hints != asset.end() && hints->is_object()) {
        candidate.view_point_size_mm = JsonFloat(*hints, "pc_size_mm");
        if (candidate.view_point_size_mm <= 0.0f) {
          candidate.view_point_size_mm = JsonFloat(*hints, "pc_base_size_mm");
        }
      }
      candidates_.push_back(candidate);
      break;
    }
  }
}

void PlantReconstructionLayer::AddInputCandidates(const std::filesystem::path& input_root) {
  if (!std::filesystem::exists(input_root)) {
    return;
  }
  std::error_code error;
  for (const auto& dataset : std::filesystem::directory_iterator(
           input_root, std::filesystem::directory_options::skip_permission_denied, error)) {
    std::error_code entry_error;
    if (error || !dataset.is_directory(entry_error)) {
      continue;
    }
    AddInputFolderCandidates(dataset.path(), candidates_);
  }
}

void PlantReconstructionLayer::AddInputFolderCandidates(const std::filesystem::path& input_folder_path,
                                                        std::vector<PointCloudCandidate>& candidates) const {
  if (!HasInputFolderTransform(input_folder_path)) {
    return;
  }

  std::error_code error;
  for (std::filesystem::recursive_directory_iterator
           it(input_folder_path, std::filesystem::directory_options::skip_permission_denied, error),
       end;
       !error && it != end; it.increment(error)) {
    std::error_code entry_error;
    if (!it->is_regular_file(entry_error) || !IsPlyFile(it->path())) {
      continue;
    }
    if (HasPathSegment(it->path().lexically_relative(input_folder_path), "transforms")) {
      continue;
    }
    PointCloudCandidate candidate;
    candidate.path = it->path();
    candidate.input_folder_path = input_folder_path;
    candidate.sample_name = StemString(it->path());
    candidate.source_name = input_folder_path.filename().string();
    candidate.source_kind = "input raw";
    candidate.role = "point_cloud";
    candidate.stage = "input";
    candidate.priority = 1;
    candidate.modified_time = LastWriteTime(candidate.path);
    candidates.push_back(candidate);
  }
}

bool PlantReconstructionLayer::HasInputFolderTransform(const std::filesystem::path& input_folder_path) const {
  const auto transform_root = input_folder_path / "transforms" / "unknown_to_point_cloud_xyz";
  if (!std::filesystem::exists(transform_root)) {
    return false;
  }
  std::error_code error;
  for (std::filesystem::recursive_directory_iterator
           it(transform_root, std::filesystem::directory_options::skip_permission_denied, error),
       end;
       !error && it != end; it.increment(error)) {
    std::error_code entry_error;
    if (it->is_regular_file(entry_error) && it->path().filename() == "transform.py") {
      return true;
    }
  }
  return false;
}

std::vector<PlantReconstructionLayer::PointCloudCandidate> PlantReconstructionLayer::BuildCheckedInputCandidates()
    const {
  std::vector<PointCloudCandidate> checked_candidates;
  for (const auto& folder : input_folders_) {
    if (folder.checked) {
      AddInputFolderCandidates(folder.path, checked_candidates);
    }
  }
  std::sort(checked_candidates.begin(), checked_candidates.end(),
            [](const PointCloudCandidate& a, const PointCloudCandidate& b) {
              if (a.source_name != b.source_name) {
                return a.source_name < b.source_name;
              }
              return a.modified_time > b.modified_time;
            });
  return checked_candidates;
}

PlantReconstructionLayer::ImportResult PlantReconstructionLayer::ImportCandidate(const PointCloudCandidate& candidate,
                                                                                 const bool save_asset) {
  ImportResult result;
  result.source_path = candidate.path;
  result.sample_name = candidate.sample_name;

  const auto cloud = AssetManager::CreateTemporaryAsset<PointCloud>();
  PointCloud::PointCloudLoadSettings settings{};
  if (!cloud->LoadPly(settings, candidate.path)) {
    status_ = "Failed to load PLY: " + candidate.path.string();
    result.message = status_;
    return result;
  }

  const auto entity_name = CandidateEntityName(candidate);
  if (save_asset) {
    const auto folder = ProjectManager::GetOrCreateFolder("RealtimePlantReconstructor/PointClouds").lock();
    const auto asset_stem = SafeAssetStem(entity_name);
    if (!ProjectManager::SaveAsset(cloud, folder, asset_stem, ".evepointcloud", true)) {
      status_ = "Failed to save point-cloud asset for " + candidate.path.string();
      result.message = status_;
      return result;
    }
  }

  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    status_ = "No active scene.";
    result.message = status_;
    return result;
  }
  const auto entity = scene->CreateEntity(entity_name);
  const auto subject = scene->GetOrSetPrivateComponent<PlantReconstructionSubject>(entity).lock();
  if (!subject) {
    status_ = "Failed to create PlantReconstructionSubject.";
    result.message = status_;
    return result;
  }
  PlantReconstructionSubject::ImportMetadata metadata;
  metadata.source_path = candidate.path;
  metadata.manifest_path = candidate.manifest_path;
  metadata.sample_name = candidate.sample_name;
  metadata.source_name = candidate.source_name;
  metadata.source_kind = candidate.source_kind;
  metadata.role = candidate.role;
  metadata.stage = candidate.stage;
  metadata.view_point_size_mm = candidate.view_point_size_mm;
  ApplyDefaultsToSubject(*subject);
  subject->SetPointCloudAsset(cloud, metadata);
  subject->RebuildPreview();

  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    editor_layer->SetSelectedEntity(entity);
  }

  status_ = "Imported " + std::to_string(subject->GetDisplayedPointCount()) + " / " +
            std::to_string(subject->GetTotalPointCount()) + " point(s) as entity '" + entity_name + "'.";
  result.success = true;
  result.entity = entity;
  result.total_point_count = subject->GetTotalPointCount();
  result.displayed_point_count = subject->GetDisplayedPointCount();
  result.message = status_;
  return result;
}

void PlantReconstructionLayer::DrawSubjectPreviews(const std::shared_ptr<EditorLayer>& editor_layer) const {
  if (!editor_layer) {
    return;
  }
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return;
  }
  const auto owners = scene->UnsafeGetPrivateComponentOwnersList<PlantReconstructionSubject>();
  if (!owners) {
    return;
  }
  for (const auto entity : *owners) {
    if (!scene->IsEntityValid(entity)) {
      continue;
    }
    if (const auto subject = scene->GetOrSetPrivateComponent<PlantReconstructionSubject>(entity).lock()) {
      subject->DrawEditorPreview(editor_layer);
    }
  }
}

void PlantReconstructionLayer::ApplyDefaultsToSubject(PlantReconstructionSubject& subject) const {
  subject.show_preview = show_preview_;
  subject.point_size_mm = point_size_mm_;
  subject.point_size_multiplier = point_size_multiplier_;
  subject.point_opacity = point_opacity_;
  subject.point_color = glm::vec3(point_color_[0], point_color_[1], point_color_[2]);
  subject.max_preview_points = max_preview_points_;
  subject.use_ply_colors = use_ply_colors_;
  if (subject.neighborhood_k != neighborhood_k_ || subject.max_neighborhood_points != max_neighborhood_points_) {
    subject.neighborhood_k = neighborhood_k_;
    subject.max_neighborhood_points = max_neighborhood_points_;
    subject.MarkNeighborhoodDirty();
  }
  subject.MarkPreviewDirty();
}

void PlantReconstructionLayer::StoreCheckedInputFolderSelection() {
  saved_checked_input_folders_.clear();
  for (const auto& folder : input_folders_) {
    if (folder.checked) {
      saved_checked_input_folders_.insert(NormalizeInputFolderPath(folder.path));
    }
  }
  input_folder_selection_loaded_ = true;
}

std::string PlantReconstructionLayer::NormalizeInputFolderPath(const std::filesystem::path& path) const {
  std::error_code error;
  const auto absolute = std::filesystem::absolute(path, error);
  const auto normalized = (error ? path : absolute).lexically_normal();
  return normalized.string();
}

std::string PlantReconstructionLayer::CandidateEntityName(const PointCloudCandidate& candidate) const {
  if (!candidate.sample_name.empty()) {
    return candidate.sample_name;
  }
  return StemString(candidate.path);
}

std::string PlantReconstructionLayer::CandidateLabel(const PointCloudCandidate& candidate) const {
  std::stringstream stream;
  stream << candidate.sample_name << " [" << candidate.source_kind << "]";
  if (!candidate.stage.empty()) {
    stream << " " << candidate.stage;
  }
  stream << " - " << candidate.source_name;
  return stream.str();
}
