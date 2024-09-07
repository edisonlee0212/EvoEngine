#include "Shader.hpp"
#include "Utilities.hpp"

using namespace evo_engine;
bool Shader::Compiled() const {
  return shader_module_ != nullptr;
}

void Shader::Set(const ShaderType shader_type, const std::string& shader_code) {
  shader_type_ = shader_type;
  code_ = shader_code;
  VkShaderModuleCreateInfo create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  const auto binary = ShaderUtils::CompileGlsl(shader_type, code_);
  create_info.pCode = binary.data();
  create_info.codeSize = binary.size() * sizeof(uint32_t);
  shader_module_ = std::make_unique<ShaderModule>(create_info);
}
const std::unique_ptr<ShaderModule>& Shader::GetShaderModule() const {
  return shader_module_;
}

ShaderType Shader::GetShaderType() const {
  return shader_type_;
}
