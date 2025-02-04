
#pragma once
#include "RandomSampler.hpp"

namespace evo_engine {
/**
 * @class FileUtils
 * @brief A utility class for file handling operations such as loading, opening, and saving files.
 */
class FileUtils {
 public:
  /**
   * @brief Loads the contents of a file as a string.
   * @param path The file path. Defaults to an empty path if not provided.
   * @return The contents of the file as a string.
   */
  static std::string LoadFileAsString(const std::filesystem::path& path = "");

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

/**
 * @class SphereMeshGenerator
 * @brief A utility class for generating mesh data for 3D spheres.
 */
class SphereMeshGenerator {
 public:
  /**
   * @brief Generates an icosahedron mesh.
   * @param vertices The output vector of vertices.
   * @param triangles The output vector of triangle indices.
   */
  static void Icosahedron(std::vector<glm::vec3>& vertices, std::vector<glm::uvec3>& triangles);
};

}  // namespace evo_engine

namespace ImGui {
/**
 * @brief Creates a splitter widget for resizing UI panels.
 * @param split_vertically Indicates whether the splitter is vertical or horizontal.
 * @param thickness The thickness of the splitter.
 * @param size1 Reference to the size of the first panel.
 * @param size2 Reference to the size of the second panel.
 * @param min_size1 Minimum size of the first panel.
 * @param min_size2 Minimum size of the second panel.
 * @param splitter_long_axis_size Length of the splitter along the long axis. Defaults to -1.0f for auto-calculation.
 * @return True if the sizes of the panels were modified, false otherwise.
 */
IMGUI_API bool Splitter(bool split_vertically, float thickness, float& size1, float& size2, float min_size1,
                        float min_size2, float splitter_long_axis_size = -1.0f);

/**
 * @brief Creates a combo box widget with a list of selectable items.
 * @param label The label for the combo box widget.
 * @param items A list of selectable items.
 * @param current_selection Reference to an unsigned value storing the current selection.
 * @param flags Optional flags for customizing the combo box behavior. Defaults to 0.
 * @return True if the selection was changed, false otherwise.
 */
IMGUI_API bool Combo(const std::string& label, const std::vector<std::string>& items, unsigned& current_selection,
                     ImGuiComboFlags flags = 0);

/**
 * @brief Creates a combo box widget with a list of selectable items.
 * @param label The label for the combo box widget.
 * @param items A list of selectable items.
 * @param current_selection Reference to an int value storing the current selection.
 * @param flags Optional flags for customizing the combo box behavior. Defaults to 0.
 * @return True if the selection was changed, false otherwise.
 */
IMGUI_API bool Combo(const std::string& label, const std::vector<std::string>& items, int& current_selection,
                     ImGuiComboFlags flags = 0);
}  // namespace ImGui
