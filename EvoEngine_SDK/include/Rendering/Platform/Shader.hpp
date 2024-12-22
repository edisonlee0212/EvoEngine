#pragma once
#include "GraphicsResources.hpp"
#include "IAsset.hpp"

namespace evo_engine {
enum class ShaderType {
  Vertex,
  TessellationControl,
  TessellationEvaluation,
  Geometry,
  Task,
  Mesh,
  Fragment,
  Compute,
  RayGen,
  ClosestHit,
  Miss,
  AnyHit,
  Intersection,
  Callable,
  Unknown
};
class Shader final : public IAsset {
  std::unique_ptr<ShaderModule> shader_module = {};
  std::string shader_code = {};
  unsigned shader_type = static_cast<unsigned>(ShaderType::Unknown);
  inline static std::set<std::filesystem::path> shader_include_paths{};

 protected:
  [[nodiscard]] bool SaveInternal(const std::filesystem::path& path) const override;
  [[nodiscard]] bool LoadInternal(const std::filesystem::path& path) override;

 public:
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  static void RegisterShaderIncludePath(const std::filesystem::path& path);
  static const std::set<std::filesystem::path>& GetRegisteredShaderIncludePaths();
  [[nodiscard]] bool Compiled() const;
  [[maybe_unused]] bool TryCompile(ShaderType target_shader_type, const std::filesystem::path& path);
  [[maybe_unused]] bool TryCompile(ShaderType target_shader_type, const std::string& header,
                                   const std::filesystem::path& path);
  [[maybe_unused]] bool TryCompile(ShaderType target_shader_type, const std::string& target_shader_code);
  [[maybe_unused]] bool TryCompile();
  [[nodiscard]] const std::unique_ptr<ShaderModule>& GetShaderModule() const;
  [[nodiscard]] ShaderType GetShaderType() const;
  static std::shared_ptr<Shader> CreateTemporary(ShaderType target_shader_type, const std::string& target_shader_code);
  static std::shared_ptr<Shader> CreateTemporary(ShaderType target_shader_type, const std::string& header,
                                                 const std::filesystem::path& path);
  static std::shared_ptr<Shader> CreateTemporary(ShaderType target_shader_type, const std::filesystem::path& path);
};
}  // namespace evo_engine
