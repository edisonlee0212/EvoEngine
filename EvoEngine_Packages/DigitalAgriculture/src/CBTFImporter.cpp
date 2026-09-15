//
// Created by lllll on 10/17/2022.
//

#include "CBTFImporter.hpp"
#include "BtfMaterial.hpp"

using namespace digital_agriculture_package;

void digital_agriculture_package::CBTFImporter::Update() {
  if (!m_processing)
    return;
  if (m_importFolders.empty()) {
    m_processing = false;
    return;
  }
  auto path = m_importFolders.back();
  m_importFolders.pop_back();
  auto asset = AssetManager::CreateTemporaryAsset<BtfMaterial>();
  asset->ImportFromFolder(path);
  asset->Export(m_currentExportFolder.string() + "\\" + path.filename().string() + ".btf");
}
