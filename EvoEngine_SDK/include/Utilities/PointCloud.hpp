#pragma once
#include "Application.hpp"
#include "Camera.hpp"
#include "Mesh.hpp"
namespace evo_engine {

class PointCloud : public IAsset {
  glm::dvec3 min_ = glm::dvec3(FLT_MAX);
  glm::dvec3 max_ = glm::dvec3(-FLT_MAX);

 protected:
  bool LoadInternal(const std::filesystem::path& path) override;
  bool SaveInternal(const std::filesystem::path& path) const override;

 public:
  struct PointCloudSaveSettings {
    bool binary = true;
    bool double_precision = false;
  };
  struct PointCloudLoadSettings {
    bool binary = true;
  };
  glm::dvec3 offset;
  bool has_positions = false;
  bool has_normals = false;
  bool has_colors = false;
  std::vector<glm::dvec3> positions;
  std::vector<glm::dvec3> normals;
  std::vector<glm::vec4> colors;
  float point_size = 0.01f;
  float compress_factor = 0.01f;
  void OnCreate() override;

  bool Load(const PointCloudLoadSettings& settings, const std::filesystem::path& path);
  bool Save(const PointCloudSaveSettings& settings, const std::filesystem::path& path) const;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Compress(std::vector<glm::dvec3>& points);
  void ApplyCompressed();
  void ApplyOriginal() const;
  void RecalculateBoundingBox();
  static void Crop(std::vector<glm::dvec3>& points, const glm::dvec3& min, const glm::dvec3& max);
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};
}  // namespace evo_engine
