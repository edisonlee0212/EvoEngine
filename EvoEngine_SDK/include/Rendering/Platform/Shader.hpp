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
  friend class GraphicsPipeline;
  std::unique_ptr<ShaderModule> shader_module_ = {};
  std::string code_ = {};
  ShaderType shader_type_ = ShaderType::Unknown;

 public:
  [[nodiscard]] bool Compiled() const;
  void Set(ShaderType shader_type, const std::string& shader_code);
  [[nodiscard]] const std::unique_ptr<ShaderModule>& GetShaderModule() const;
};
}  // namespace evo_engine
