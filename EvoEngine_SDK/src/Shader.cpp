#include "Shader.hpp"
#include <cstdlib>
#include "AssetManager.hpp"
#include "Console.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "ResourceLimits.h"
#include "SPIRV/GlslangToSpv.h"
#include "Serialization.hpp"
#include "ShaderLang.h"
#include "Utilities.hpp"

using namespace evo_engine;

namespace {
std::filesystem::path GetShaderBinaryDirectory() {
  if (const char* path = std::getenv("EVOENGINE_SHADER_CACHE_DIR"); path && path[0] != '\0') {
    return path;
  }
  return "./ShaderBinaries";
}

class ShaderStagedLoadPayload final : public StagedAssetLoadPayload {
 public:
  std::string shader_code;
  unsigned shader_type = static_cast<unsigned>(ShaderType::Unknown);
};
}  // namespace

bool Shader::SaveInternal(const std::filesystem::path& path) const {
  try {
    if (path.extension() == ".eveshader") {
      YAML::Emitter out;
      out << YAML::BeginMap;
      out << YAML::Key << "shader_type" << YAML::Value << shader_type;
      out << YAML::Key << "shader_code" << YAML::Value << shader_code;
      out << YAML::EndMap;
      std::ofstream file_output(path.string());
      file_output << out.c_str();
      file_output.close();
    } else {
      std::ofstream file_output(path.string());
      file_output << shader_code.c_str();
      file_output.close();
    }
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to save!")
    return false;
  }
  return true;
}

bool Shader::LoadInternal(const std::filesystem::path& path) {
  if (!std::filesystem::exists(path)) {
    EVOENGINE_ERROR("Not exist!")
    return false;
  }
  try {
    const std::ifstream stream(path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    if (path.extension() == ".eveshader") {
      const YAML::Node in = YAML::Load(string_stream.str());
      if (in["shader_code"])
        shader_code = in["shader_code"].as<std::string>();
      if (in["shader_type"])
        shader_type = in["shader_type"].as<unsigned>();
    } else {
      shader_type = static_cast<unsigned>(ShaderType::Unknown);
      shader_code = string_stream.str();
    }
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to load!")
    return false;
  }
  return true;
}

bool Shader::SupportsStagedLoading() const {
  return true;
}

std::shared_ptr<StagedAssetLoadPayload> Shader::LoadStagedPayloadInternal(const std::filesystem::path& path) const {
  if (!std::filesystem::exists(path)) {
    EVOENGINE_ERROR("Not exist!")
    return {};
  }
  try {
    const std::ifstream stream(path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    auto payload = std::make_shared<ShaderStagedLoadPayload>();
    if (path.extension() == ".eveshader") {
      const YAML::Node in = YAML::Load(string_stream.str());
      if (in["shader_code"]) {
        payload->shader_code = in["shader_code"].as<std::string>();
      }
      if (in["shader_type"]) {
        payload->shader_type = in["shader_type"].as<unsigned>();
      }
    } else {
      payload->shader_type = static_cast<unsigned>(ShaderType::Unknown);
      payload->shader_code = string_stream.str();
    }
    return payload;
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to load staged shader payload: " + std::string(e.what()))
    return {};
  }
}

bool Shader::ApplyStagedPayloadInternal(const std::filesystem::path&,
                                        const std::shared_ptr<StagedAssetLoadPayload>& payload) {
  const auto shader_payload = std::dynamic_pointer_cast<ShaderStagedLoadPayload>(payload);
  if (!shader_payload) {
    return false;
  }
  shader_code = std::move(shader_payload->shader_code);
  shader_type = shader_payload->shader_type;
  return true;
}

bool Shader::RegisterAssetIoHandlers(const std::string& owner_name, const std::string& type_name) {
  return Serialization::RegisterAssetIoHandler<Shader>(
      [](const Shader& asset, const std::filesystem::path& path) {
        return asset.SaveInternal(path);
      },
      [](Shader& asset, const std::filesystem::path& path) {
        return asset.LoadInternal(path);
      },
      [](const Shader& asset, const std::filesystem::path&) {
        return asset.SupportsStagedLoading();
      },
      [](const Shader& asset, const std::filesystem::path& path) {
        return asset.LoadStagedPayloadInternal(path);
      },
      [](Shader& asset, const std::filesystem::path& path, const std::shared_ptr<StagedAssetLoadPayload>& payload) {
        return asset.ApplyStagedPayloadInternal(path, payload);
      },
      owner_name, type_name);
}

void Shader::RegisterShaderIncludePath(const std::filesystem::path& path) {
  Platform::GetInstance().RegisterShaderIncludePath(path);
}

const std::set<std::filesystem::path>& Shader::GetRegisteredShaderIncludePaths() {
  return Platform::GetInstance().GetRegisteredShaderIncludePaths();
}

bool Shader::Compiled() const {
  return shader_module != nullptr;
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
  IncludeResult fail_result_ = IncludeResult("", "Header does not exist!", 0, nullptr);
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
  } else {
    return &fail_result_;
  }
  auto [it, b] = includes_.emplace(std::make_pair(
      resolved_header_path,
      std::make_shared<IncludeResult>(resolved_header_path.string(), sources_.at(resolved_header_path).data(),
                                      sources_.at(resolved_header_path).size(), nullptr)));
  if (!b) {
    EVOENGINE_ERROR("Failed to insert IncludeResult into std::map!");
    return &fail_result_;
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

bool CompileGlsl(const ShaderType shader_type, const std::string& source, std::vector<uint32_t>& binaries,
                 const std::filesystem::path& path) {
  // 1. Look for compiled resource.
  const auto shader_binary_directory = GetShaderBinaryDirectory();
  const auto binary_search_path = shader_binary_directory / (std::to_string(std::hash<std::string>{}(source)) + ".yml");
  if (std::filesystem::exists(binary_search_path)) {
    const std::ifstream stream(binary_search_path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    const YAML::Node in = YAML::Load(string_stream.str());
    assert(in["CompiledBinaries"]);
    const auto in_binaries = in["CompiledBinaries"].as<YAML::Binary>();
    binaries.resize(in_binaries.size() / sizeof(uint32_t));
    std::memcpy(binaries.data(), in_binaries.data(), in_binaries.size());
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
        return false;
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
    GlslShaderIncluder glsl_shader_includer;
    if (!shader.preprocess(resources, default_version, default_profile, false, forward_compatible, message_flags,
                           &preprocessedStr, glsl_shader_includer)) {
      EVOENGINE_ERROR("Failed to preprocess shader: " + path.string() + "\n" + std::string(shader.getInfoLog()));
      return false;
    }
    const char* preprocessedSources[1] = {preprocessedStr.c_str()};
    shader.setStrings(preprocessedSources, 1);

    if (!shader.parse(resources, default_version, default_profile, false, forward_compatible, message_flags,
                      glsl_shader_includer)) {
      EVOENGINE_ERROR("Failed to parse shader: " + path.string() + "\n" + std::string(shader.getInfoLog()));
      return false;
    }
    glslang::TProgram program;
    program.addShader(&shader);
    if (!program.link(message_flags)) {
      EVOENGINE_ERROR("Failed to link shader: " + path.string() + "\n" + std::string(program.getInfoLog()));
      return false;
    }

    // Convert the intermediate generated by glslang to Spir-V
    glslang::TIntermediate& intermediate_ref = *program.getIntermediate(sh_language);
    glslang::SpvOptions options{};
    options.generateDebugInfo = true;
    options.validate = true;

    spv::SpvBuildLogger logger;
    GlslangToSpv(intermediate_ref, binaries, &logger, &options);

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "CompiledBinaries" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(binaries.data()), binaries.size() * sizeof(uint32_t));
    out << YAML::EndMap;
    std::filesystem::create_directories(shader_binary_directory);
    std::ofstream file_output(binary_search_path);
    file_output << out.c_str();
    file_output.close();
  }
  return true;
}

bool Shader::TryCompile(const ShaderType target_shader_type, const std::filesystem::path& path) {
  return TryCompile(target_shader_type, "", path);
}

bool Shader::TryCompile(const ShaderType target_shader_type, const std::string& header,
                        const std::filesystem::path& path) {
  std::stringstream shader_code_stream;
  shader_code_stream << header;
  shader_code_stream << FileUtils::LoadFileAsString(path);
  shader_type = static_cast<unsigned>(target_shader_type);
  shader_code = shader_code_stream.str();
  return TryCompile(path);
}

bool Shader::TryCompile(const ShaderType target_shader_type, const std::string& target_shader_code) {
  shader_type = static_cast<unsigned>(target_shader_type);
  shader_code = target_shader_code;
  return TryCompile();
}

bool Shader::TryCompile(const std::filesystem::path& path) {
  if (!Platform::Initialized())
    return false;
  VkShaderModuleCreateInfo create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  std::vector<uint32_t> binaries;
  if (CompileGlsl(static_cast<ShaderType>(shader_type), shader_code, binaries, path)) {
    create_info.pCode = binaries.data();
    create_info.codeSize = binaries.size() * sizeof(uint32_t);
    shader_module = std::make_unique<ShaderModule>(create_info);
    version_++;
    return true;
  }
  return false;
}

const std::unique_ptr<ShaderModule>& Shader::GetShaderModule() const {
  return shader_module;
}

ShaderType Shader::GetShaderType() const {
  return static_cast<ShaderType>(shader_type);
}

const std::string& Shader::PeekShaderCode() const {
  return shader_code;
}

std::string& Shader::RefShaderCode() {
  return shader_code;
}

unsigned& Shader::RefShaderType() {
  return shader_type;
}

std::shared_ptr<Shader> Shader::CreateTemporary(const ShaderType target_shader_type,
                                                const std::string& target_shader_code) {
  const auto ret_val = AssetManager::CreateTemporaryAsset<Shader>();
  ret_val->TryCompile(target_shader_type, target_shader_code);
  return ret_val;
}

std::shared_ptr<Shader> Shader::CreateTemporary(const ShaderType target_shader_type, const std::string& header,
                                                const std::filesystem::path& path) {
  const auto ret_val = AssetManager::CreateTemporaryAsset<Shader>();
  ret_val->TryCompile(target_shader_type, header, path);
  return ret_val;
}

std::shared_ptr<Shader> Shader::CreateTemporary(const ShaderType target_shader_type,
                                                const std::filesystem::path& path) {
  const auto ret_val = AssetManager::CreateTemporaryAsset<Shader>();
  ret_val->TryCompile(target_shader_type, path);
  return ret_val;
}
