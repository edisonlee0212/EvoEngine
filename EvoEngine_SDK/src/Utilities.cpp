#include "Utilities.hpp"
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#  include "shlobj.h"
#endif
#include "Application.hpp"
#include "Console.hpp"
#include "EditorLayer.hpp"
#include "ProjectManager.hpp"
#include "WindowLayer.hpp"

using namespace evo_engine;
std::string FileUtils::LoadFileAsString(const std::filesystem::path& path) {
  std::ifstream file;
  file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  try {
    // open files
    file.open(path);
    std::stringstream stream;
    // read file's buffer contents into streams
    stream << file.rdbuf();
    // close file handlers
    file.close();
    // convert stream into string
    return stream.str();
  } catch (std::ifstream::failure e) {
    EVOENGINE_ERROR("Load file failed!")
    throw;
  }
}

void FileUtils::OpenFolder(const std::string& dialog_title,
                           const std::function<void(const std::filesystem::path& path)>& func, bool project_dir_check) {
  const auto window_layer = Application::GetLayer<WindowLayer>();
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  if (window_layer && ImGui::Button(dialog_title.c_str())) {
    TCHAR path[MAX_PATH];
    BROWSEINFO bi = {0};
    bi.lpszTitle = dialog_title.c_str();
    bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;
    // bi.lpfn       = BrowseCallbackProc;
    // bi.lParam     = (LPARAM) path_param;
    LPITEMIDLIST pidl = SHBrowseForFolder(&bi);
    if (pidl != nullptr) {
      // get the name of the folder and put it in path
      SHGetPathFromIDList(pidl, path);
      // free memory used
      IMalloc* imalloc = nullptr;
      if (SUCCEEDED(SHGetMalloc(&imalloc))) {
        imalloc->Free(pidl);
        imalloc->Release();
      }
      std::string ret_val = path;
      const std::string search = "\\";
      size_t pos = ret_val.find(search);
      // Repeat till end is reached
      while (pos != std::string::npos) {
        // Replace this occurrence of Sub String
        ret_val.replace(pos, 1, "/");
        // Get the next occurrence from the current position
        pos = ret_val.find(search, pos + 1);
      }
      std::filesystem::path path = ret_val;
      if (!project_dir_check || ProjectManager::IsInProjectFolder(path))
        func(path);
    }
  }
#else
  if (windowLayer && ImGui::Button(dialogTitle.c_str()))
    ImGuiFileDialog::Instance()->OpenDialog(dialogTitle, "Choose Folder", nullptr, ".", 1, nullptr,
                                            ImGuiFileDialogFlags_Default);
  // display
  ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_Appearing);
  if (ImGuiFileDialog::Instance()->Display(dialogTitle, ImGuiWindowFlags_AlwaysAutoResize)) {
    // action if OK
    if (ImGuiFileDialog::Instance()->IsOk()) {
      // action
      std::filesystem::path path = ImGuiFileDialog::Instance()->GetCurrentPath();
      if (!projectDirCheck || ProjectManager::IsInProjectFolder(path))
        func(path);
    }
    // close
    ImGuiFileDialog::Instance()->Close();
  }
#endif
}

void FileUtils::OpenFile(const std::string& dialog_title, const std::string& file_type,
                         const std::vector<std::string>& extensions,
                         const std::function<void(const std::filesystem::path& path)>& func, bool project_dir_check) {
  auto window_layer = Application::GetLayer<WindowLayer>();
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  if (window_layer && ImGui::Button(dialog_title.c_str())) {
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
    if (GetOpenFileNameA(&ofn) == TRUE) {
      std::string ret_val = ofn.lpstrFile;
      const std::string search = "\\";
      size_t pos = ret_val.find(search);
      // Repeat till end is reached
      while (pos != std::string::npos) {
        // Replace this occurrence of Sub String
        ret_val.replace(pos, 1, "/");
        // Get the next occurrence from the current position
        pos = ret_val.find(search, pos + 1);
      }
      std::filesystem::path path = ret_val;
      if (!project_dir_check || ProjectManager::IsInProjectFolder(path))
        func(path);
    }
  }
#else
  std::stringstream fileExtensions;
  for (int i = 0; i < extensions.size(); i++) {
    fileExtensions << extensions[i];
    if (i != extensions.size() - 1)
      fileExtensions << ",";
  }
  if (windowLayer && ImGui::Button(dialogTitle.c_str()))
    ImGuiFileDialog::Instance()->OpenDialog(dialogTitle, "Choose File", fileExtensions.str().c_str(), ".", 1, nullptr,
                                            ImGuiFileDialogFlags_Default);
  // display
  ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_Appearing);
  if (ImGuiFileDialog::Instance()->Display(dialogTitle)) {
    // action if OK
    if (ImGuiFileDialog::Instance()->IsOk()) {
      // action
      std::filesystem::path path = ImGuiFileDialog::Instance()->GetFilePathName();
      if (!projectDirCheck || ProjectManager::IsInProjectFolder(path))
        func(path);
    }

    // close
    ImGuiFileDialog::Instance()->Close();
  }
#endif
}

void FileUtils::SaveFile(const std::string& dialog_title, const std::string& file_type,
                         const std::vector<std::string>& extensions,
                         const std::function<void(const std::filesystem::path& path)>& func, bool project_dir_check) {
  const auto window_layer = Application::GetLayer<WindowLayer>();
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  if (ImGui::Button(dialog_title.c_str())) {
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
      std::string ret_val = ofn.lpstrFile;
      const std::string search = "\\";
      size_t pos = ret_val.find(search);
      // Repeat till end is reached
      while (pos != std::string::npos) {
        // Replace this occurrence of Sub String
        ret_val.replace(pos, 1, "/");
        // Get the next occurrence from the current position
        pos = ret_val.find(search, pos + 1);
      }
      std::filesystem::path path = ret_val;
      if (!project_dir_check || ProjectManager::IsInProjectFolder(path))
        func(path);
    }
  }
#else
  std::stringstream fileExtensions;
  for (int i = 0; i < extensions.size(); i++) {
    fileExtensions << extensions[i];
    if (i != extensions.size() - 1)
      fileExtensions << ",";
  }
  if (windowLayer && ImGui::Button(dialogTitle.c_str()))
    ImGuiFileDialog::Instance()->OpenDialog(dialogTitle, "Choose File", fileExtensions.str().c_str(), ".", 1, nullptr,
                                            ImGuiFileDialogFlags_Default);
  ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_Appearing);
  // display
  if (ImGuiFileDialog::Instance()->Display(dialogTitle)) {
    // action if OK
    if (ImGuiFileDialog::Instance()->IsOk()) {
      // action
      std::filesystem::path path = ImGuiFileDialog::Instance()->GetFilePathName();
      if (!projectDirCheck || ProjectManager::IsInProjectFolder(path))
        func(path);
    }

    // close
    ImGuiFileDialog::Instance()->Close();
  }
#endif
}



void SphereMeshGenerator::Icosahedron(std::vector<glm::vec3>& vertices, std::vector<glm::uvec3>& triangles) {
  vertices.clear();
  triangles.clear();

  const float phi = (1.0f + glm::sqrt(5.0f)) * 0.5f;  // golden ratio
  const float a = 1.0f;
  const float b = 1.0f / phi;

  // add vertices
  vertices.push_back(glm::normalize(glm::vec3(0, b, -a)));
  vertices.push_back(glm::normalize(glm::vec3(b, a, 0)));
  vertices.push_back(glm::normalize(glm::vec3(-b, a, 0)));
  vertices.push_back(glm::normalize(glm::vec3(0, b, a)));
  vertices.push_back(glm::normalize(glm::vec3(0, -b, a)));
  vertices.push_back(glm::normalize(glm::vec3(-a, 0, b)));
  vertices.push_back(glm::normalize(glm::vec3(0, -b, -a)));
  vertices.push_back(glm::normalize(glm::vec3(a, 0, -b)));
  vertices.push_back(glm::normalize(glm::vec3(a, 0, b)));
  vertices.push_back(glm::normalize(glm::vec3(-a, 0, -b)));
  vertices.push_back(glm::normalize(glm::vec3(b, -a, 0)));
  vertices.push_back(glm::normalize(glm::vec3(-b, -a, 0)));

  // add triangles
  triangles.emplace_back(3, 2, 1);
  triangles.emplace_back(2, 3, 4);
  triangles.emplace_back(6, 5, 4);
  triangles.emplace_back(5, 9, 4);
  triangles.emplace_back(8, 7, 1);
  triangles.emplace_back(7, 10, 1);
  triangles.emplace_back(12, 11, 5);
  triangles.emplace_back(11, 12, 7);
  triangles.emplace_back(10, 6, 3);
  triangles.emplace_back(6, 10, 12);
  triangles.emplace_back(9, 8, 2);
  triangles.emplace_back(8, 9, 11);
  triangles.emplace_back(3, 6, 4);
  triangles.emplace_back(9, 2, 4);
  triangles.emplace_back(10, 3, 1);
  triangles.emplace_back(2, 8, 1);
  triangles.emplace_back(12, 10, 7);
  triangles.emplace_back(8, 11, 7);
  triangles.emplace_back(6, 12, 5);
  triangles.emplace_back(11, 9, 5);

  for (auto& i : triangles) {
    i.x -= 1;
    i.y -= 1;
    i.z -= 1;
  }
}

bool ImGui::Splitter(bool split_vertically, float thickness, float& size1, float& size2, float min_size1,
                     float min_size2, float splitter_long_axis_size) {
  ImGuiContext& g = *GImGui;
  ImGuiWindow* window = g.CurrentWindow;
  ImGuiID id = window->GetID("##Splitter");
  ImRect bb;
  bb.Min = window->DC.CursorPos + (split_vertically ? ImVec2(size1, 0.0f) : ImVec2(0.0f, size1));
  bb.Max = bb.Min + CalcItemSize(split_vertically ? ImVec2(thickness, splitter_long_axis_size)
                                                  : ImVec2(splitter_long_axis_size, thickness),
                                 0.0f, 0.0f);
  return SplitterBehavior(bb, id, split_vertically ? ImGuiAxis_X : ImGuiAxis_Y, &size1, &size2, min_size1, min_size2,
                          0.0f);
}

bool ImGui::Combo(const std::string& label, const std::vector<std::string>& items, unsigned& current_selection,
                  ImGuiComboFlags flags) {
  bool modified = false;
  current_selection = glm::clamp(current_selection, 0u, static_cast<unsigned>(items.size()));
  if (ImGui::BeginCombo(label.c_str(), items[current_selection].c_str(),
                        flags))  // The second parameter is the label previewed before opening the combo.
  {
    for (unsigned i = 0; i < items.size(); i++) {
      const bool selected = current_selection == i;
      if (ImGui::Selectable(items[i].c_str(), selected)) {
        current_selection = i;
        modified = true;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();  // You may set the initial focus when opening the combo (scrolling
                                       // + for keyboard navigation support)
      }
    }
    ImGui::EndCombo();
  }
  return modified;
}

bool ImGui::Combo(const std::string& label, const std::vector<std::string>& items, int& current_selection,
                  ImGuiComboFlags flags) {
  bool modified = false;
  current_selection = glm::clamp(current_selection, 0, static_cast<int>(items.size()));
  if (ImGui::BeginCombo(label.c_str(), items[current_selection].c_str(),
                        flags))  // The second parameter is the label previewed before opening the combo.
  {
    for (int i = 0; i < items.size(); i++) {
      const bool selected = current_selection == i;
      if (ImGui::Selectable(items[i].c_str(), selected)) {
        current_selection = i;
        modified = true;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();  // You may set the initial focus when opening the combo (scrolling
                                       // + for keyboard navigation support)
      }
    }
    ImGui::EndCombo();
  }
  return modified;
}