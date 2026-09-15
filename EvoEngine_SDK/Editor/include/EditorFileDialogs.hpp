#pragma once

#include <filesystem>
#include <functional>
#include <string>
#include <vector>
#include "EvoEngineAPI.hpp"
#include "EvoEngineEditorAPI.hpp"

namespace evo_engine {
class EVOENGINE_EDITOR_API EditorFileDialogs {
 public:
  /**
   * @brief Opens a folder selection dialog.
   * @param dialog_title The title of the dialog.
   * @param func A callback function that is invoked with the selected folder path.
   * @param project_dir_check Boolean flag to enforce project directory check. Defaults to true.
   */
  static void OpenFolder(const std::string& dialog_title,
                         const std::function<void(const std::filesystem::path& path)>& func,
                         bool project_dir_check = true);

  /**
   * @brief Opens a file selection dialog.
   * @param dialog_title The title of the dialog.
   * @param file_type The file type description.
   * @param extensions A list of acceptable file extensions.
   * @param func A callback function that is invoked with the selected file path.
   * @param project_dir_check Boolean flag to enforce project directory check. Defaults to true.
   */
  static void OpenFile(const std::string& dialog_title, const std::string& file_type,
                       const std::vector<std::string>& extensions,
                       const std::function<void(const std::filesystem::path& path)>& func,
                       bool project_dir_check = true);

  /**
   * @brief Opens a save file dialog.
   * @param dialog_title The title of the dialog.
   * @param file_type The file type description.
   * @param extensions A list of acceptable file extensions.
   * @param func A callback function that is invoked with the save file path.
   * @param project_dir_check Boolean flag to enforce project directory check. Defaults to true.
   */
  static void SaveFile(const std::string& dialog_title, const std::string& file_type,
                       const std::vector<std::string>& extensions,
                       const std::function<void(const std::filesystem::path& path)>& func,
                       bool project_dir_check = true);
};
}  // namespace evo_engine
