#include "Json.hpp"

using namespace eco_sys_lab;

bool Json::SaveInternal(const std::filesystem::path& path) const {
  std::ofstream o(path);
  o << std::setw(4) << m_json << std::endl;
  return true;
}

bool Json::LoadInternal(const std::filesystem::path& path) {
  std::ifstream ifs(path);
  m_json = treeio::json::parse(ifs);
  return true;
}

bool Json::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;

  return changed;
}
