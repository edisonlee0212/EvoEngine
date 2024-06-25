#include "Shader.hpp"
#include "Utilities.hpp"

using namespace evo_engine;
bool Shader::Compiled() const {
  return shader_module_ != nullptr;
}

void Shader::Set(const ShaderType shader_type, const std::string& shader_code) {
  shader_type_ = shader_type;
  code_ = shader_code;
  shaderc_shader_kind shader_kind;
  switch (shader_type) {
    case ShaderType::Task:
      shader_kind = shaderc_task_shader;
      break;
    case ShaderType::Mesh:
      shader_kind = shaderc_mesh_shader;
      break;
    case ShaderType::Vertex:
      shader_kind = shaderc_vertex_shader;
      break;
    case ShaderType::TessellationControl:
      shader_kind = shaderc_tess_control_shader;
      break;
    case ShaderType::TessellationEvaluation:
      shader_kind = shaderc_tess_evaluation_shader;
      break;
    case ShaderType::Geometry:
      shader_kind = shaderc_geometry_shader;
      break;
    case ShaderType::Fragment:
      shader_kind = shaderc_fragment_shader;
      break;
    case ShaderType::Compute:
      shader_kind = shaderc_compute_shader;
      break;
    case ShaderType::Unknown:
      shader_kind = shaderc_glsl_infer_from_source;
      break;
  }

  VkShaderModuleCreateInfo create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  const auto binary = ShaderUtils::Get("Shader", shader_kind, code_);
  create_info.pCode = binary.data();
  create_info.codeSize = binary.size() * sizeof(uint32_t);
  shader_module_ = std::make_unique<ShaderModule>(create_info);
}
const std::unique_ptr<ShaderModule>& Shader::GetShaderModule() const {
  return shader_module_;
}