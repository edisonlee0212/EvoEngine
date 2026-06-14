
#pragma once
#include <SorghumLayer.hpp>

namespace digital_agriculture_package {
using namespace evo_engine;

/**
 * @class CBTFImporter
 * @brief A class responsible for importing CBTF files into the DigitalAgriculture package.
 *
 * This class facilitates the handling of CBTF files, managing their processing state,
 * and providing an interface for inspecting and updating their import process.
 */
class CBTFImporter : public IPrivateComponent {
 public:
  /** @brief Indicates whether the importer is currently processing files. */
  bool m_processing = false;

  /** @brief The folder where the current export process is storing files. */
  std::filesystem::path m_currentExportFolder;

  /** @brief A list of folders from which CBTF files will be imported. */
  std::vector<std::filesystem::path> m_importFolders;

  /**
   * @brief Updates the importer state for handling the import process.
   *
   * This function should be called periodically to process files and manage the import lifecycle.
   */
  void Update() override;
};

}  // namespace digital_agriculture_package
