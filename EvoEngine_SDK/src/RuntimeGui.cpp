#include "RuntimeGui.hpp"
#include "IRuntimeGui.hpp"
#include "Scene.hpp"

using namespace evo_engine;

const std::vector<AssetRef>& RuntimeGui::GetGuiAssets() const {
  return gui_assets_;
}

bool RuntimeGui::AddGuiAsset(AssetRef asset) {
  if (asset.GetAssetHandle() == Handle(0) ||
      std::find(gui_assets_.begin(), gui_assets_.end(), asset) != gui_assets_.end())
    return false;
  try {
    if (const auto resolved = asset.Get<IAsset>(); resolved && !std::dynamic_pointer_cast<IRuntimeGui>(resolved))
      return false;
  } catch (const std::exception&) {
    // Optional GUI references can outlive their asset provider.
  }
  gui_assets_.push_back(std::move(asset));
  return true;
}

bool RuntimeGui::RemoveGuiAsset(const size_t index) {
  if (index >= gui_assets_.size())
    return false;
  gui_assets_.erase(gui_assets_.begin() + index);
  return true;
}

bool RuntimeGui::MoveGuiAsset(const size_t from, const size_t to) {
  if (from >= gui_assets_.size() || to >= gui_assets_.size() || from == to)
    return false;
  auto asset = std::move(gui_assets_[from]);
  gui_assets_.erase(gui_assets_.begin() + from);
  gui_assets_.insert(gui_assets_.begin() + to, std::move(asset));
  return true;
}

const std::string& RuntimeGui::GetLayout() const {
  return layout_;
}

void RuntimeGui::SetLayout(std::string layout) {
  layout_ = std::move(layout);
}

void RuntimeGui::Serialize(YAML::Emitter& out) const {
  camera.Save("camera", out);
  out << YAML::Key << "draw_order" << YAML::Value << draw_order;
  out << YAML::Key << "layout" << YAML::Value << layout_;
  out << YAML::Key << "gui_assets" << YAML::Value << YAML::BeginSeq;
  for (const auto& asset : gui_assets_) {
    out << YAML::BeginMap;
    asset.Serialize(out);
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
}

void RuntimeGui::Deserialize(const YAML::Node& in) {
  camera.Load("camera", in, GetScene());
  if (in["draw_order"])
    draw_order = in["draw_order"].as<int>();
  if (in["layout"])
    layout_ = in["layout"].as<std::string>();
  gui_assets_.clear();
  if (const auto assets = in["gui_assets"]; assets && assets.IsSequence())
    for (const auto& entry : assets) {
      AssetRef asset;
      try {
        asset.Deserialize(entry);
      } catch (const std::exception&) {
        // Deserialize has retained the handle/type before attempting resolution.
      }
      if (std::find(gui_assets_.begin(), gui_assets_.end(), asset) == gui_assets_.end())
        gui_assets_.push_back(std::move(asset));
    }
}

void RuntimeGui::Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) {
  camera.Relink(map, scene);
}

void RuntimeGui::CollectAssetRef(std::vector<AssetRef>& list) {
  list.insert(list.end(), gui_assets_.begin(), gui_assets_.end());
}
