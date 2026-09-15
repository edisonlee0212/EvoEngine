#include "EditorFileDialogs.hpp"
#ifdef EVOENGINE_WINDOWS
#  include "shlobj.h"
#endif
#include "Application.hpp"
#include "Console.hpp"
#include "EditorLayer.hpp"
#include "PathUtils.hpp"
#include "ProjectManager.hpp"
#include "WindowLayer.hpp"

#ifdef EVOENGINE_WINDOWS
#  include "ShObjIdl.h"
#  define GLFW_EXPOSE_NATIVE_WIN32  ///< Exposes native Win32 context for GLFW
#  include "GLFW/glfw3native.h"
#  define STBI_MSC_SECURE_CRT  ///< Configures CRT secure functions for stb_image on Windows
#else
// Linux specific includes
#  define GLFW_EXPOSE_NATIVE_WAYLAND
#  include "GLFW/glfw3native.h"
#endif
#include "Utilities/X11MacroCleanup.hpp"

using namespace evo_engine;

namespace {
void EmitDialogPath(const std::filesystem::path& selected_path, const bool project_dir_check,
                    const std::function<void(const std::filesystem::path& path)>& func) {
  const auto normalized_path = path_utils::NormalizeAbsolutePath(selected_path);
  if (!project_dir_check || ProjectManager::IsInAssetsFolder(normalized_path)) {
    func(normalized_path);
  }
}
}  // namespace

#ifdef EVOENGINE_WINDOWS
namespace {
std::wstring ToWideString(const std::string& value) {
  if (value.empty()) {
    return {};
  }
  const int size = MultiByteToWideChar(CP_UTF8, 0, value.c_str(), static_cast<int>(value.size()), nullptr, 0);
  if (size <= 0) {
    return std::wstring(value.begin(), value.end());
  }
  std::wstring result(size, L'\0');
  MultiByteToWideChar(CP_UTF8, 0, value.c_str(), static_cast<int>(value.size()), result.data(), size);
  return result;
}

std::wstring BuildFileDialogPattern(const std::vector<std::string>& extensions) {
  std::wstring pattern;
  for (size_t i = 0; i < extensions.size(); ++i) {
    if (i != 0) {
      pattern += L";";
    }
    pattern += L"*";
    pattern += ToWideString(extensions[i]);
  }
  return pattern;
}

std::wstring BuildFileDialogFilterName(const std::string& file_type, const std::vector<std::string>& extensions) {
  std::wstring name = ToWideString(file_type);
  const std::wstring pattern = BuildFileDialogPattern(extensions);
  if (!pattern.empty()) {
    name += L" (" + pattern + L")";
  }
  return name;
}

void RefreshOwnerChrome(const std::shared_ptr<WindowLayer>& window_layer) {
  if (window_layer) {
    window_layer->RefreshCustomTitleBar();
  }
}

bool OpenWindowsFileDialog(const std::shared_ptr<WindowLayer>& window_layer, const std::string& dialog_title,
                           const std::string& file_type, const std::vector<std::string>& extensions,
                           std::filesystem::path& selected_path, const bool pick_folder = false) {
  if (!window_layer || !window_layer->GetGlfwWindow()) {
    return false;
  }

  RefreshOwnerChrome(window_layer);
  const HRESULT initialize_result = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
  const bool uninitialize = SUCCEEDED(initialize_result);
  if (FAILED(initialize_result) && initialize_result != RPC_E_CHANGED_MODE) {
    RefreshOwnerChrome(window_layer);
    return false;
  }

  IFileOpenDialog* dialog = nullptr;
  HRESULT result = CoCreateInstance(CLSID_FileOpenDialog, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&dialog));
  if (FAILED(result) || !dialog) {
    if (uninitialize) {
      CoUninitialize();
    }
    RefreshOwnerChrome(window_layer);
    return false;
  }

  const std::wstring title = ToWideString(dialog_title);
  dialog->SetTitle(title.c_str());

  DWORD options = 0;
  if (SUCCEEDED(dialog->GetOptions(&options))) {
    dialog->SetOptions(options | FOS_FORCEFILESYSTEM | FOS_PATHMUSTEXIST | FOS_NOCHANGEDIR |
                       (pick_folder ? FOS_PICKFOLDERS : FOS_FILEMUSTEXIST));
  }

  const std::wstring filter_name = BuildFileDialogFilterName(file_type, extensions);
  const std::wstring filter_pattern = BuildFileDialogPattern(extensions);
  if (!filter_name.empty() && !filter_pattern.empty()) {
    const COMDLG_FILTERSPEC filters[] = {{filter_name.c_str(), filter_pattern.c_str()}};
    dialog->SetFileTypes(1, filters);
    dialog->SetFileTypeIndex(1);
  }

  const HWND owner = glfwGetWin32Window(window_layer->GetGlfwWindow());
  bool selected = false;
  result = dialog->Show(owner);
  if (SUCCEEDED(result)) {
    IShellItem* item = nullptr;
    if (SUCCEEDED(dialog->GetResult(&item)) && item) {
      PWSTR file_path = nullptr;
      if (SUCCEEDED(item->GetDisplayName(SIGDN_FILESYSPATH, &file_path)) && file_path) {
        selected_path = std::filesystem::path(file_path);
        selected = true;
        CoTaskMemFree(file_path);
      }
      item->Release();
    }
  }

  dialog->Release();
  if (uninitialize) {
    CoUninitialize();
  }
  RefreshOwnerChrome(window_layer);
  return selected;
}
}  // namespace
#endif

void EditorFileDialogs::OpenFolder(const std::string& dialog_title,
                                   const std::function<void(const std::filesystem::path& path)>& func,
                                   bool project_dir_check) {
  const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>();
#ifdef EVOENGINE_WINDOWS
  if (window_layer && ImGui::Button(dialog_title.c_str())) {
    std::filesystem::path path;
    if (OpenWindowsFileDialog(window_layer, dialog_title, {}, {}, path, true)) {
      EmitDialogPath(path, project_dir_check, func);
    }
  }
#else
  if (window_layer && ImGui::Button(dialog_title.c_str()))
    ImGuiFileDialog::Instance()->OpenDialog(dialog_title, "Choose Folder", nullptr, ".", 1, nullptr,
                                            ImGuiFileDialogFlags_Default);
  // display
  ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_Appearing);
  if (ImGuiFileDialog::Instance()->Display(dialog_title, ImGuiWindowFlags_AlwaysAutoResize)) {
    // action if OK
    if (ImGuiFileDialog::Instance()->IsOk()) {
      // action
      EmitDialogPath(ImGuiFileDialog::Instance()->GetCurrentPath(), project_dir_check, func);
    }
    // close
    ImGuiFileDialog::Instance()->Close();
  }
#endif
}

void EditorFileDialogs::OpenFile(const std::string& dialog_title, const std::string& file_type,
                                 const std::vector<std::string>& extensions,
                                 const std::function<void(const std::filesystem::path& path)>& func,
                                 bool project_dir_check) {
  auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>();
#ifdef EVOENGINE_WINDOWS
  if (window_layer && ImGui::Button(dialog_title.c_str())) {
    std::filesystem::path path;
    if (OpenWindowsFileDialog(window_layer, dialog_title, file_type, extensions, path)) {
      EmitDialogPath(path, project_dir_check, func);
    }
  }
#else
  std::stringstream fileExtensions;
  for (int i = 0; i < extensions.size(); i++) {
    fileExtensions << extensions[i];
    if (i != extensions.size() - 1)
      fileExtensions << ",";
  }
  if (window_layer && ImGui::Button(dialog_title.c_str()))
    ImGuiFileDialog::Instance()->OpenDialog(dialog_title, "Choose File", fileExtensions.str().c_str(), ".", 1, nullptr,
                                            ImGuiFileDialogFlags_Default);
  // display
  ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_Appearing);
  if (ImGuiFileDialog::Instance()->Display(dialog_title)) {
    // action if OK
    if (ImGuiFileDialog::Instance()->IsOk()) {
      // action
      EmitDialogPath(ImGuiFileDialog::Instance()->GetFilePathName(), project_dir_check, func);
    }

    // close
    ImGuiFileDialog::Instance()->Close();
  }
#endif
}

void EditorFileDialogs::SaveFile(const std::string& dialog_title, const std::string& file_type,
                                 const std::vector<std::string>& extensions,
                                 const std::function<void(const std::filesystem::path& path)>& func,
                                 bool project_dir_check) {
  const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>();
#ifdef EVOENGINE_WINDOWS
  if (ImGui::Button(dialog_title.c_str())) {
    RefreshOwnerChrome(window_layer);
    OPENFILENAMEA ofn;
    CHAR sz_file[260] = {0};
    ZeroMemory(&ofn, sizeof(OPENFILENAME));
    ofn.lStructSize = sizeof(OPENFILENAME);
    ofn.hwndOwner = glfwGetWin32Window(window_layer->GetGlfwWindow());
    ofn.lpstrFile = sz_file;
    ofn.nMaxFile = sizeof(sz_file);
    std::string filters = file_type + " (";
    for (int i = 0; i < extensions.size(); i++) {
      filters += "*" + extensions[i];
      if (i < extensions.size() - 1)
        filters += ", ";
    }
    filters += ") ";
    std::string filters2;
    for (int i = 0; i < extensions.size(); i++) {
      filters2 += "*" + extensions[i];
      if (i < extensions.size() - 1)
        filters2 += ";";
    }
    char actual_filter[256];
    char title[256];
    strcpy(title, dialog_title.c_str());
    int index = 0;
    for (auto& i : filters) {
      actual_filter[index] = i;
      index++;
    }
    actual_filter[index] = 0;
    index++;
    for (auto& i : filters2) {
      actual_filter[index] = i;
      index++;
    }
    actual_filter[index] = 0;
    index++;
    actual_filter[index] = 0;
    index++;
    ofn.lpstrFilter = actual_filter;
    ofn.nFilterIndex = 1;
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
    ofn.lpstrTitle = title;
    // Sets the default extension by extracting it from the filter
    ofn.lpstrDefExt = strchr(actual_filter, '\0') + 1;

    if (GetSaveFileNameA(&ofn) == TRUE) {
      EmitDialogPath(ofn.lpstrFile, project_dir_check, func);
    }
    RefreshOwnerChrome(window_layer);
  }
#else
  std::stringstream fileExtensions;
  for (int i = 0; i < extensions.size(); i++) {
    fileExtensions << extensions[i];
    if (i != extensions.size() - 1)
      fileExtensions << ",";
  }
  if (window_layer && ImGui::Button(dialog_title.c_str()))
    ImGuiFileDialog::Instance()->OpenDialog(dialog_title, "Choose File", fileExtensions.str().c_str(), ".", 1, nullptr,
                                            ImGuiFileDialogFlags_Default);
  ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_Appearing);
  // display
  if (ImGuiFileDialog::Instance()->Display(dialog_title)) {
    // action if OK
    if (ImGuiFileDialog::Instance()->IsOk()) {
      // action
      EmitDialogPath(ImGuiFileDialog::Instance()->GetFilePathName(), project_dir_check, func);
    }

    // close
    ImGuiFileDialog::Instance()->Close();
  }
#endif
}
