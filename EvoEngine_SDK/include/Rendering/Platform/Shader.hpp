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
  Unknown
};
class Shader final : public IAsset {
  std::unique_ptr<ShaderModule> shader_module_ = {};
  std::string code_ = {};
  ShaderType shader_type_ = ShaderType::Unknown;
  inline static std::set<std::filesystem::path> shader_include_paths{};
 public:
  static void RegisterShaderIncludePath(const std::filesystem::path& path);
  static const std::set<std::filesystem::path>& GetRegisteredShaderIncludePaths();
  [[nodiscard]] bool Compiled() const;
  void Set(ShaderType shader_type, const std::filesystem::path& path);
  void Set(ShaderType shader_type, const std::string& header, const std::filesystem::path& path);
  void Set(ShaderType shader_type, const std::string& shader_code);
  [[nodiscard]] const std::unique_ptr<ShaderModule>& GetShaderModule() const;
  [[nodiscard]] ShaderType GetShaderType() const;
};
}  // namespace evo_engine
