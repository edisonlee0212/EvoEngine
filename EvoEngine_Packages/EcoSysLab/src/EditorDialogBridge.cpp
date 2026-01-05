#include "EditorDialogBridge.hpp"

namespace eco_sys_lab_package {
namespace {
EditorFileDialogFn open_file_fn = nullptr;
EditorFileDialogFn save_file_fn = nullptr;
}  // namespace

void SetEditorOpenFileFn(const EditorFileDialogFn fn) {
  open_file_fn = fn;
}

void SetEditorSaveFileFn(const EditorFileDialogFn fn) {
  save_file_fn = fn;
}

void OpenEditorFile(const std::string& dialog_title, const std::string& file_type,
                    const std::vector<std::string>& extensions,
                    const std::function<void(const std::filesystem::path&)>& func, const bool project_dir_check) {
  if (open_file_fn) {
    open_file_fn(dialog_title, file_type, extensions, func, project_dir_check);
  }
}

void SaveEditorFile(const std::string& dialog_title, const std::string& file_type,
                    const std::vector<std::string>& extensions,
                    const std::function<void(const std::filesystem::path&)>& func, const bool project_dir_check) {
  if (save_file_fn) {
    save_file_fn(dialog_title, file_type, extensions, func, project_dir_check);
  }
}
}  // namespace eco_sys_lab_package
