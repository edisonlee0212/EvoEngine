//
// Created by lllll on 10/17/2022.
//

#include "CBTFImporter.hpp"
#include "DigitalAgricultureInspectionAdapters.hpp"
#ifdef CUDA_MODULE_SERVICE
#  include "BtfMaterial.hpp"
#endif

using namespace digital_agriculture_package;

bool digital_agriculture_package::InspectCBTFImporter(InspectorContext& context, CBTFImporter& importer) {
  (void)context;
  ImGui::Text("Current output folder: %s", importer.m_currentExportFolder.string().c_str());
  FileUtils::OpenFolder(
      "Choose output folder...",
      [&importer](const std::filesystem::path& path) {
        importer.m_currentExportFolder = std::filesystem::absolute(path);
      },
      false);

  FileUtils::OpenFolder(
      "Collect CBTF Folders",
      [&importer](const std::filesystem::path& path) {
        importer.m_importFolders.clear();
        auto& projectManager = ProjectManager::GetInstance();
        if (std::filesystem::exists(path) && std::filesystem::is_directory(path)) {
          for (const auto& folderEntry : std::filesystem::recursive_directory_iterator(path)) {
            if (std::filesystem::is_directory(folderEntry.path())) {
              for (const auto& fileEntry : std::filesystem::directory_iterator(folderEntry)) {
                if (!std::filesystem::is_directory(fileEntry.path()) &&
                    fileEntry.path().filename() == "all_materialInfo.txt") {
                  importer.m_importFolders.emplace_back(folderEntry);
                  break;
                }
              }
            }
          }
        }
      },
      false);

  ImGui::Text(("Remaining Folders: " + std::to_string(importer.m_importFolders.size())).c_str());

  if (importer.m_processing) {
    if (ImGui::Button("Pause")) {
      importer.m_processing = false;
    }
  } else {
    if (ApplicationContext::Get().IsPlaying() && !importer.m_importFolders.empty()) {
      if (ImGui::Button("Process")) {
        importer.m_processing = true;
      }
    }
    if (!importer.m_importFolders.empty() && ImGui::Button("Clear"))
      importer.m_importFolders.clear();
  }
  return false;
}
void digital_agriculture_package::CBTFImporter::Update() {
  if (!m_processing)
    return;
  if (m_importFolders.empty()) {
    m_processing = false;
    return;
  }
  auto path = m_importFolders.back();
  m_importFolders.pop_back();
#ifdef CUDA_MODULE_SERVICE
  auto asset = AssetManager::CreateTemporaryAsset<BtfMaterial>();
  asset->ImportFromFolder(path);
  asset->Export(m_currentExportFolder.string() + "\\" + path.filename().string() + ".cbtf");
#endif
}
