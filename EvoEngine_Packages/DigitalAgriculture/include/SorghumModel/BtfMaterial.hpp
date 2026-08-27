#pragma once
#include "EvoEngine_SDK_PCH.hpp"

#include "IAsset.hpp"

#include <filesystem>

#include "BtfData.hpp"
namespace evo_engine {
class BtfMaterial : public IAsset {
  friend void SerializeBtfMaterial(YAML::Emitter& out, const BtfMaterial& target);
  friend void DeserializeBtfMaterial(const YAML::Node& in, BtfMaterial& target);

 public:
  std::vector<float> shared_coordinates_beta_angles;

  std::vector<int> pdf6d;
  std::vector<float> pdf6d_scales;

  std::vector<int> pdf4d;
  std::vector<float> pdf4d_scales;

  std::vector<int> pdf3d;
  std::vector<float> pdf3d_scales;

  std::vector<int> pdf2d;
  std::vector<float> pdf2d_scales;

  std::vector<int> luminance_color_indices;
  std::vector<int> pdf2d_colors;

  std::vector<int> index_ab;

  std::vector<float> pdf1d;

  std::vector<float> vector_color;

  BtfBase btf_base;
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
  bool ImportFromFolder(const std::filesystem::path& path);
};
}  // namespace evo_engine
