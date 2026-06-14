#include "Json.hpp"
#include "Console.hpp"
#include "InspectorRegistry.hpp"
#include "Serialization.hpp"

using namespace evo_engine;

namespace {
class JsonStagedLoadPayload final : public StagedAssetLoadPayload {
 public:
  nlohmann::json json;
};

bool SaveJson(const Json& json, const std::filesystem::path& path) {
  std::ofstream o(path);
  o << std::setw(4) << json.m_json << '\n';
  return true;
}

bool LoadJson(Json& json, const std::filesystem::path& path) {
  std::ifstream ifs(path);
  json.m_json = nlohmann::json::parse(ifs);
  return true;
}

bool SupportsJsonStagedLoading(const Json&, const std::filesystem::path&) {
  return true;
}

std::shared_ptr<StagedAssetLoadPayload> LoadJsonStagedPayload(const Json&, const std::filesystem::path& path) {
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

bool ApplyJsonStagedPayload(Json& json, const std::filesystem::path&,
                            const std::shared_ptr<StagedAssetLoadPayload>& payload) {
  const auto json_payload = std::dynamic_pointer_cast<JsonStagedLoadPayload>(payload);
  if (!json_payload) {
    return false;
  }
  json.m_json = std::move(json_payload->json);
  return true;
}

bool InspectJson(InspectorContext&, Json&) {
  return false;
}
}  // namespace

bool Json::SaveInternal(const std::filesystem::path& path) const {
  return SaveJson(*this, path);
}

bool Json::LoadInternal(const std::filesystem::path& path) {
  return LoadJson(*this, path);
}

bool Json::SupportsStagedLoading() const {
  return true;
}

std::shared_ptr<StagedAssetLoadPayload> Json::LoadStagedPayloadInternal(const std::filesystem::path& path) const {
  return LoadJsonStagedPayload(*this, path);
}

bool Json::ApplyStagedPayloadInternal(const std::filesystem::path&,
                                      const std::shared_ptr<StagedAssetLoadPayload>& payload) {
  return ApplyJsonStagedPayload(*this, {}, payload);
}

void evo_engine::RegisterJsonHandlers() {
  Serialization::RegisterAssetIoHandler<Json>(SaveJson, LoadJson, SupportsJsonStagedLoading, LoadJsonStagedPayload,
                                              ApplyJsonStagedPayload, {}, "Json");
  InspectorRegistry::GetInstance().RegisterInspector<Json>(InspectJson, {}, "Json");
}
