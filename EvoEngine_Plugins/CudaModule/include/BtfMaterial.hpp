#pragma once
#include "EvoEngine_SDK_PCH.hpp"

#include "IAsset.hpp"

#include "filesystem"

#include "BtfBase.cuh"
namespace evo_engine {
class BtfMaterial : public IAsset {
  void UploadDeviceData();

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
  bool OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) override;
  bool ImportFromFolder(const std::filesystem::path &path);
  void Serialize(YAML::Emitter &out) const override;

  void Deserialize(const YAML::Node &in) override;
};
}  // namespace evo_engine