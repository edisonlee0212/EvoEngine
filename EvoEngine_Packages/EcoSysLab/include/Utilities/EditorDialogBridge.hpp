#pragma once
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace eco_sys_lab_package {
using EditorFileDialogFn = void (*)(const std::string& dialog_title, const std::string& file_type,
                                    const std::vector<std::string>& extensions,
                                    const std::function<void(const std::filesystem::path&)>& func,
                                    bool project_dir_check);

void SetEditorOpenFileFn(EditorFileDialogFn fn);
void SetEditorSaveFileFn(EditorFileDialogFn fn);
void OpenEditorFile(const std::string& dialog_title, const std::string& file_type,
                    const std::vector<std::string>& extensions,
                    const std::function<void(const std::filesystem::path&)>& func, bool project_dir_check = true);
void SaveEditorFile(const std::string& dialog_title, const std::string& file_type,
                    const std::vector<std::string>& extensions,
                    const std::function<void(const std::filesystem::path&)>& func, bool project_dir_check = true);
}  // namespace eco_sys_lab_package
