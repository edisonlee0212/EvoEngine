#pragma once

// =============================================================================
//  IPlantBundle — plant-agnostic contract for the GPU L-system engine.
//
//  Phase 0 stub. Compiled into the plugin only when LSYSTEM_GPU_PIPELINE is
//  defined (see EvoEngine_Plugins/LSystem/CMakeLists.txt).
//
//  A "plant bundle" is a self-contained directory under
//      EvoEngine_Plugins/LSystem/plants/<name>/
//  declaring its module types, production rules, mesh-shader emitters, and
//  default descriptor asset. Tassel ships first; conifer and vegetative
//  maize will plug in via this same interface in later phases without
//  touching LSystemGPUEngine.
//
//  See docs/gpu_pipeline.md (sections 3, 7, 9) for the full architecture.
//
//  Nothing here is wired into the runtime yet. The structs and the abstract
//  base define the boundary that Phase 5 will populate; Phases 1–3 may
//  reference them as forward declarations only.
// =============================================================================

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace evo_engine {
class IAsset;
}  // namespace evo_engine

namespace l_system_plugin::gpu {

// ----- Module type declaration -------------------------------------------------

/// Width of one element in a per-type state SSBO, in bytes.
/// Phase 1 will codegen GLSL struct definitions from this; for now it is a
/// hint used by the engine to size buffers.
struct StateFieldDecl {
  std::string name;          ///< field name (matches GLSL struct member)
  std::string glsl_type;     ///< "float", "vec4", "uint", "uvec4", ...
  uint32_t array_count = 1;  ///< 1 for scalars; >1 for inline arrays
};

struct ModuleTypeDecl {
  std::string name;                       ///< e.g. "Apex", "Internode", "SpikeletPair"
  uint32_t type_id = 0;                   ///< stable id used in module_type[N] tag buffer
  std::vector<StateFieldDecl> fields;     ///< per-instance state schema
  bool renderable = false;                ///< true if a mesh-shader emitter exists for it
};

// ----- Rule declaration --------------------------------------------------------

/// A successor template. Each fired rule emits exactly one or zero of these,
/// depending on the predicate. successor_count_max in RuleDecl is the static
/// upper bound used to reserve append slots in derive.comp.
struct SuccessorTemplateDecl {
  uint32_t module_type_id = 0;       ///< target ModuleTypeDecl::type_id
  uint32_t init_predicate_id = 0;    ///< index into shader-side initializer table
};

struct RuleDecl {
  std::string name;                                   ///< human-readable label
  uint32_t source_module_type_id = 0;                 ///< rule fires on this type
  uint32_t predicate_id = 0;                          ///< index into shader predicate table
  uint32_t successor_count_max = 0;                   ///< compile-time upper bound (see §5)
  std::vector<SuccessorTemplateDecl> successors;      ///< populated up to successor_count_max
};

// ----- Mesh-shader emitter declaration ----------------------------------------

struct EmitterDecl {
  uint32_t module_type_id = 0;          ///< which module type this emitter renders
  std::string task_shader_path;         ///< relative to bundle's shaders/ folder
  std::string mesh_shader_path;
  std::string fragment_shader_path;     ///< optional; falls back to engine default
  std::string material_ref;             ///< asset reference: bark, leaf, etc.
};

// ----- Bundle base class ------------------------------------------------------

class IPlantBundle {
 public:
  virtual ~IPlantBundle() = default;

  /// Stable identifier; used as the registry key and as the bundle directory name.
  virtual std::string Name() const = 0;

  virtual std::vector<ModuleTypeDecl> ModuleTypes() const = 0;
  virtual std::vector<RuleDecl>       Rules() const = 0;
  virtual std::vector<EmitterDecl>    Emitters() const = 0;

  /// Default descriptor asset (e.g. MaizeTasselDescriptor) used when an
  /// instance is spawned without a user-provided one.
  virtual std::shared_ptr<evo_engine::IAsset> DefaultDescriptor() const = 0;
};

// ----- Bundle registry --------------------------------------------------------
//
// Phase 5 will provide a static registration macro (PLANT_BUNDLE_REGISTRY)
// analogous to EvoEngine's AssetRegistration<T>. The registry lives inside
// LSystemGPUEngine and is enumerated at engine startup.

}  // namespace l_system_plugin::gpu
