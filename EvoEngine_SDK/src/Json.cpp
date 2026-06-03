#include "Json.hpp"
#include "Console.hpp"

using namespace evo_engine;

namespace {
class JsonStagedLoadPayload final : public StagedAssetLoadPayload {
 public:
  nlohmann::json json;
};
}  // namespace

bool Json::SaveInternal(const std::filesystem::path& path) const {
  std::ofstream o(path);
  o << std::setw(4) << m_json << '\n';
  return true;
}

bool Json::LoadInternal(const std::filesystem::path& path) {
  std::ifstream ifs(path);
  m_json = nlohmann::json::parse(ifs);
  return true;
}

bool Json::SupportsStagedLoading() const {
  return true;
}

std::shared_ptr<StagedAssetLoadPayload> Json::LoadStagedPayloadInternal(const std::filesystem::path& path) const {
  try {
    std::ifstream ifs(path);
    auto payload = std::make_shared<JsonStagedLoadPayload>();
    payload->json = nlohmann::json::parse(ifs);
    return payload;
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to load staged JSON payload: " + std::string(e.what()))
    return {};
  }
}

bool Json::ApplyStagedPayloadInternal(const std::filesystem::path&,
                                      const std::shared_ptr<StagedAssetLoadPayload>& payload) {
  const auto json_payload = std::dynamic_pointer_cast<JsonStagedLoadPayload>(payload);
  if (!json_payload) {
    return false;
  }
  m_json = std::move(json_payload->json);
  return true;
}

bool Json::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  return changed;
}
