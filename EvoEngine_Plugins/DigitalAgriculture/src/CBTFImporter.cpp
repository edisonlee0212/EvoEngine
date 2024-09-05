//
// Created by lllll on 10/17/2022.
//

#include "CBTFImporter.hpp"
#ifdef OPTIX_RAY_TRACER_PLUGIN
#  include "CompressedBTF.hpp"
#endif

using namespace digital_agriculture;

bool CBTFImporter::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Text("Current output folder: %s", m_currentExportFolder.string().c_str());
  FileUtils::OpenFolder(
      "Choose output folder...",
      [&](const std::filesystem::path& path) {
        m_currentExportFolder = std::filesystem::absolute(path);
      },
      false);

  FileUtils::OpenFolder(
      "Collect CBTF Folders",
      [&](const std::filesystem::path& path) {
        m_importFolders.clear();
        auto& projectManager = ProjectManager::GetInstance();
        if (std::filesystem::exists(path) && std::filesystem::is_directory(path)) {
          for (const auto& folderEntry : std::filesystem::recursive_directory_iterator(path)) {
            if (std::filesystem::is_directory(folderEntry.path())) {
              for (const auto& fileEntry : std::filesystem::directory_iterator(folderEntry)) {
                if (!std::filesystem::is_directory(fileEntry.path()) &&
                    fileEntry.path().filename() == "all_materialInfo.txt") {
                  m_importFolders.emplace_back(folderEntry);
                  break;
                }
              }
            }
          }
        }
      },
      false);

  ImGui::Text(("Remaining Folders: " + std::to_string(m_importFolders.size())).c_str());

  if (m_processing) {
    if (ImGui::Button("Pause")) {
      m_processing = false;
    }
  } else {
    if (Application::IsPlaying() && !m_importFolders.empty()) {
      if (ImGui::Button("Process")) {
        m_processing = true;
      }
    }
    if (!m_importFolders.empty() && ImGui::Button("Clear"))
      m_importFolders.clear();
  }
  return false;
}
void digital_agriculture::CBTFImporter::Update() {
  if (!m_processing)
    return;
  if (m_importFolders.empty()) {
    m_processing = false;
    return;
  }
  auto path = m_importFolders.back();
  m_importFolders.pop_back();
#ifdef OPTIX_RAY_TRACER_PLUGIN
  auto asset = ProjectManager::CreateTemporaryAsset<CompressedBTF>();
  asset->ImportFromFolder(path);
  asset->Export(m_currentExportFolder.string() + "\\" + path.filename().string() + ".cbtf");
#endif
}
