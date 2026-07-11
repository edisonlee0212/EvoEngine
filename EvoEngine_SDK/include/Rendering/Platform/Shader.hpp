
#pragma once
#include "GraphicsResources.hpp"
#include "IAsset.hpp"

namespace evo_engine {

/**
 * @enum ShaderType
 * @brief Enum class representing various types of shaders.
 */
enum class ShaderType {
  Vertex,                 /**< Vertex shader */
  TessellationControl,    /**< Tessellation control shader */
  TessellationEvaluation, /**< Tessellation evaluation shader */
  Geometry,               /**< Geometry shader */
  Task,                   /**< Task shader */
  Mesh,                   /**< Mesh shader */
  Fragment,               /**< Fragment (pixel) shader */
  Compute,                /**< Compute shader */
  RayGen,                 /**< Ray generation shader */
  ClosestHit,             /**< Closest hit shader */
  Miss,                   /**< Miss shader */
  AnyHit,                 /**< Any-hit shader */
  Intersection,           /**< Intersection shader */
  Callable,               /**< Callable shader */
  Unknown                 /**< Unknown shader type */
};

struct ShaderCompileCacheStats {
  uint64_t memory_hits = 0;
  uint64_t disk_hits = 0;
  uint64_t disk_misses = 0;
  uint64_t compilations = 0;
  uint64_t coalesced_waits = 0;
  uint64_t corrupt_entries = 0;
  uint64_t failures = 0;
};

/**
 * @class Shader
 * @brief Represents a graphics shader asset in the engine.
 *
 * The Shader class manages the lifecycle of shaders, including compilation,
 * serialization, deserialization, and inspection through the editor layer.
 */
class Shader final : public IAsset {
  std::unique_ptr<ShaderModule> shader_module = {};                  /**< Pointer to the compiled shader module */
  std::string shader_code = {};                                      /**< Source code of the shader */
  unsigned shader_type = static_cast<unsigned>(ShaderType::Unknown); /**< Type of the shader (as enum value) */

 protected:
  /**
   * @brief Saves the shader to the specified path.
   *
   * @param path The file path where the shader will be saved.
   * @return True if the save operation was successful, false otherwise.
   */
  [[nodiscard]] bool SaveInternal(const std::filesystem::path& path) const;

  /**
   * @brief Loads the shader from the specified path.
   *
   * @param path The file path from where the shader will be loaded.
   * @return True if the load operation was successful, false otherwise.
   */
  [[nodiscard]] bool LoadInternal(const std::filesystem::path& path);

  /**
   * @brief Shader source/YAML can be read and parsed before main-thread finalization.
   */
  [[nodiscard]] bool SupportsStagedLoading() const;

  /**
   * @brief Builds a CPU-side shader source payload.
   */
  [[nodiscard]] std::shared_ptr<StagedAssetLoadPayload> LoadStagedPayloadInternal(
      const std::filesystem::path& path) const;

  /**
   * @brief Applies the staged shader source payload.
   */
  bool ApplyStagedPayloadInternal(const std::filesystem::path& path,
                                  const std::shared_ptr<StagedAssetLoadPayload>& payload);

 public:
  static bool RegisterAssetIoHandlers(const std::string& owner_name = {}, const std::string& type_name = "Shader");

  [[nodiscard]] const std::string& PeekShaderCode() const;

  [[nodiscard]] std::string& RefShaderCode();

  [[nodiscard]] unsigned& RefShaderType();

  /**
   * @brief Registers a new include path for shaders.
   *
   * @param path The file system path to be added as include path.
   */
  static void RegisterShaderIncludePath(const std::filesystem::path& path);

  /**
   * @brief Retrieves all the registered shader include paths.
   *
   * @return A set of file system paths currently registered.
   */
  static std::set<std::filesystem::path> GetRegisteredShaderIncludePaths();

  /** Compiles GLSL to SPIR-V without creating a Vulkan shader module. */
  [[nodiscard]] static bool CompileToSpirv(ShaderType shader_type, const std::string& source,
                                           std::vector<uint32_t>& binaries, const std::filesystem::path& path = {});

  [[nodiscard]] static ShaderCompileCacheStats GetCompileCacheStats();
  static void ResetCompileCacheStats();
  static void ClearInMemoryCompileCache();

  /**
   * @brief Checks whether the shader is compiled successfully.
   *
   * @return True if the shader has been compiled, false otherwise.
   */
  [[nodiscard]] bool Compiled() const;

  /**
   * @brief Attempts to compile the shader from the given file path.
   *
   * @param target_shader_type The type of the shader to compile.
   * @param path The file path to the shader source code.
   * @return True if compilation succeeded, false otherwise.
   */
  [[maybe_unused]] bool TryCompile(ShaderType target_shader_type, const std::filesystem::path& path);

  /**
   * @brief Attempts to compile the shader from a given header and file path.
   *
   * @param target_shader_type The type of shader to compile.
   * @param header Header content to prepend to the shader source code.
   * @param path The file path to the shader source code.
   * @return True if compilation succeeded, false otherwise.
   */
  [[maybe_unused]] bool TryCompile(ShaderType target_shader_type, const std::string& header,
                                   const std::filesystem::path& path);

  /**
   * @brief Attempts to compile the shader from the provided source code.
   *
   * @param target_shader_type The type of the shader to compile.
   * @param target_shader_code The source code of the shader.
   * @return True if compilation succeeded, false otherwise.
   */
  [[maybe_unused]] bool TryCompile(ShaderType target_shader_type, const std::string& target_shader_code);

  /**
   * @brief Attempts to compile the shader with previously set parameters.
   *
   * @return True if compilation succeeded, false otherwise.
   */
  [[maybe_unused]] bool TryCompile(const std::filesystem::path& path = std::filesystem::path());

  /**
   * @brief Retrieves the shader module.
   *
   * @return A const reference to the unique pointer of the shader module.
   */
  [[nodiscard]] const std::unique_ptr<ShaderModule>& GetShaderModule() const;

  /**
   * @brief Retrieves the type of the shader.
   *
   * @return The type of the shader as a ShaderType enum.
   */
  [[nodiscard]] ShaderType GetShaderType() const;

  /**
   * @brief Creates a temporary shader from given source code.
   *
   * @param target_shader_type The type of the shader to create.
   * @param target_shader_code The source code of the shader.
   * @return A shared pointer to the temporary shader object.
   */
  static std::shared_ptr<Shader> CreateTemporary(ShaderType target_shader_type, const std::string& target_shader_code);

  /**
   * @brief Creates a temporary shader from a header and source code file path.
   *
   * @param target_shader_type The type of the shader to create.
   * @param header Header content to prepend to the shader source code.
   * @param path The file path to the shader source code.
   * @return A shared pointer to the temporary shader object.
   */
  static std::shared_ptr<Shader> CreateTemporary(ShaderType target_shader_type, const std::string& header,
                                                 const std::filesystem::path& path);

  /**
   * @brief Creates a temporary shader from a source code file path.
   *
   * @param target_shader_type The type of the shader to create.
   * @param path The file path to the shader source code.
   * @return A shared pointer to the temporary shader object.
   */
  static std::shared_ptr<Shader> CreateTemporary(ShaderType target_shader_type, const std::filesystem::path& path);
};

}  // namespace evo_engine
