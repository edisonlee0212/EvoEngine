#include "Shader.hpp"

#include "Console.hpp"
#include "ResourceLimits.h"
#include "SPIRV/GlslangToSpv.h"
#include "ShaderLang.h"
#include "Utilities.hpp"
using namespace evo_engine;

void Shader::RegisterShaderIncludePath(const std::filesystem::path& path) {
  shader_include_paths.emplace(path);
}

const std::set<std::filesystem::path>& Shader::GetRegisteredShaderIncludePaths() {
  return shader_include_paths;
}

bool Shader::Compiled() const {
  return shader_module_ != nullptr;
}

void Shader::Set(const ShaderType shader_type, const std::filesystem::path& path) {
  Set(shader_type, "", path);
}

void Shader::Set(const ShaderType shader_type, const std::string& header, const std::filesystem::path& path) {
  std::stringstream shader_code_stream;

  shader_code_stream << header;
  shader_code_stream << FileUtils::LoadFileAsString(path);
  Set(shader_type, shader_code_stream.str());
}

class GlslShaderIncluder : public glslang::TShader::Includer {
 public:
  //    explicit GlslShaderIncluder(fileio::Directory* shaderdir)
  //        : mShaderdir(shaderdir) {}

  // Note "local" vs. "system" is not an "either/or": "local" is an
  // extra thing to do over "system". Both might get called, as per
  // the C++ specification.
  //
  // For the "system" or <>-style includes; search the "system" paths.
  IncludeResult* includeSystem(const char* header_name, const char* includer_name, size_t inclusion_depth) override;

  // For the "local"-only aspect of a "" include. Should not search in the
  // "system" paths, because on returning a failure, the parser will
  // call includeSystem() to look in the "system" locations.
  IncludeResult* includeLocal(const char* header_name, const char* includer_name, size_t inclusion_depth) override;

  void releaseInclude(IncludeResult*) override;

 private:
  static inline IncludeResult sm_fail_result_ = IncludeResult("", "Header does not exist!", 0, nullptr);
  std::unordered_map<std::filesystem::path, std::shared_ptr<IncludeResult>> includes_;
  std::unordered_map<std::filesystem::path, std::string> sources_;
};

glslang::TShader::Includer::IncludeResult* GlslShaderIncluder::includeSystem(const char* header_name,
                                                                             const char* includer_name,
                                                                             size_t inclusion_depth) {
  std::filesystem::path resolved_header_path;
  if (const auto it = includes_.find(resolved_header_path); it != includes_.end()) {
    return it->second.get();
  }
  const std::filesystem::path temp(header_name);
  bool found = false;
  // Search in all registered dir.
  const auto& dirs = Shader::GetRegisteredShaderIncludePaths();
  for (const auto& dir : dirs) {
    if (std::filesystem::exists(dir / temp)) {
      found = true;
      resolved_header_path = dir / temp;
      break;
    }
  }
  if (!found && std::filesystem::exists(temp)) {
    resolved_header_path = temp;
    found = true;
  }
  if (found) {
    sources_[resolved_header_path] = FileUtils::LoadFileAsString(resolved_header_path);
  }else {
    return &sm_fail_result_;
  }
  auto [it, b] = includes_.emplace(std::make_pair(
      resolved_header_path,
      std::make_shared<IncludeResult>(resolved_header_path.string(), sources_.at(resolved_header_path).data(),
                                      sources_.at(resolved_header_path).size(), nullptr)));
  if (!b) {
    EVOENGINE_ERROR("Failed to insert IncludeResult into std::map!");
    return &sm_fail_result_;
  }
  return it->second.get();
}

glslang::TShader::Includer::IncludeResult* GlslShaderIncluder::includeLocal(const char* header_name,
                                                                            const char* includer_name,
                                                                            const size_t inclusion_depth) {
  return includeSystem(header_name, includer_name, inclusion_depth);
}

void GlslShaderIncluder::releaseInclude(IncludeResult* result) {
  if (const auto it = sources_.find(result->headerName); it != sources_.end()) {
    sources_.erase(it);
  }
  if (const auto it = includes_.find(result->headerName); it != includes_.end()) {
    includes_.erase(it);
  }
}

GlslShaderIncluder glsl_shader_includer{};

std::vector<uint32_t> CompileGlsl(const ShaderType shader_type, const std::string& source) {
  // 1. Look for compiled resource.
  const auto binary_search_path = std::filesystem::path("./DefaultResources") / "CompiledShaderBinaries" /
                                  (std::to_string(std::hash<std::string>{}(source)) + ".yml");
  std::vector<uint32_t> ret_val;
  if (std::filesystem::exists(binary_search_path)) {
    const std::ifstream stream(binary_search_path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    const YAML::Node in = YAML::Load(string_stream.str());
    assert(in["CompiledBinaries"]);
    const auto binaries = in["CompiledBinaries"].as<YAML::Binary>();
    ret_val.resize(binaries.size() / sizeof(uint32_t));
    std::memcpy(ret_val.data(), binaries.data(), binaries.size());
  } else {
    glslang::InitializeProcess();
    EShLanguage sh_language;
    switch (shader_type) {
      case ShaderType::Task:
        sh_language = EShLangTask;
        break;
      case ShaderType::Mesh:
        sh_language = EShLangMesh;
        break;
      case ShaderType::Vertex:
        sh_language = EShLangVertex;
        break;
      case ShaderType::TessellationControl:
        sh_language = EShLangTessControl;
        break;
      case ShaderType::TessellationEvaluation:
        sh_language = EShLangTessEvaluation;
        break;
      case ShaderType::Geometry:
        sh_language = EShLangGeometry;
        break;
      case ShaderType::Fragment:
        sh_language = EShLangFragment;
        break;
      case ShaderType::Compute:
        sh_language = EShLangCompute;
        break;
      case ShaderType::RayGen:
        sh_language = EShLangRayGen;
        break;
      case ShaderType::Miss:
        sh_language = EShLangMiss;
        break;
      case ShaderType::AnyHit:
        sh_language = EShLangAnyHit;
        break;
      case ShaderType::ClosestHit:
        sh_language = EShLangClosestHit;
        break;
      case ShaderType::Intersection:
        sh_language = EShLangIntersect;
        break;
      case ShaderType::Callable:
        sh_language = EShLangCallable;
        break;
      case ShaderType::Unknown:
        EVOENGINE_ERROR("Unknown type shader!");
        return {};
    }
    glslang::TShader shader(sh_language);
    std::string actual_code = std::string("#version 460\n") + source; 
    const char* sources[1] = {actual_code.data()};
    constexpr int default_version = 460;
    shader.setStrings(sources, 1);
    shader.setEnvClient(glslang::EShClientVulkan, glslang::EShTargetVulkan_1_3);
    shader.setEnvTarget(glslang::EshTargetSpv, glslang::EShTargetSpv_1_4);
    shader.setEnvInput(glslang::EShSourceGlsl, sh_language, glslang::EShClientVulkan, default_version);
    shader.setEntryPoint("main");
    // The resource is an entire discussion in and by itself, here just use default.
    const TBuiltInResource* resources = GetDefaultResources();
    // int defaultVersion = 110, // use 100 for ES environment, overridden by #version in shader

    constexpr bool forward_compatible = false;
    constexpr auto message_flags = static_cast<EShMessages>(EShMsgSpvRules | EShMsgVulkanRules);
    EProfile default_profile = ECoreProfile;  // NOTE: Only for desktop, before profiles showed up!

    std::string preprocessedStr;
    if (!shader.preprocess(resources, default_version, default_profile, false, forward_compatible, message_flags,
                           &preprocessedStr, glsl_shader_includer)) {
      EVOENGINE_ERROR("Failed to preprocess shader: " + std::string(shader.getInfoLog()));
    }
    const char* preprocessedSources[1] = {preprocessedStr.c_str()};
    shader.setStrings(preprocessedSources, 1);

    if (!shader.parse(resources, default_version, default_profile, false, forward_compatible, message_flags,
                      glsl_shader_includer)) {
      EVOENGINE_ERROR("Failed to parse shader: " + std::string(shader.getInfoLog()));
    }

    glslang::TProgram program;
    program.addShader(&shader);
    if (!program.link(message_flags)) {
      EVOENGINE_ERROR("Failed to link shader: " + std::string(program.getInfoLog()));
    }

    // Convert the intermediate generated by glslang to Spir-V
    glslang::TIntermediate& intermediate_ref = *program.getIntermediate(sh_language);
    glslang::SpvOptions options{};
    options.validate = true;

    spv::SpvBuildLogger logger;
    GlslangToSpv(intermediate_ref, ret_val, &logger, &options);

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "CompiledBinaries" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(ret_val.data()), ret_val.size() * sizeof(uint32_t));
    out << YAML::EndMap;
    std::filesystem::create_directories(std::filesystem::path("./DefaultResources") / "CompiledShaderBinaries");
    std::ofstream file_output(binary_search_path);
    file_output << out.c_str();
    file_output.close();
  }
  return ret_val;
}

void Shader::Set(const ShaderType shader_type, const std::string& shader_code) {
  shader_type_ = shader_type;
  code_ = shader_code;
  VkShaderModuleCreateInfo create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  const auto binary = CompileGlsl(shader_type, code_);
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
