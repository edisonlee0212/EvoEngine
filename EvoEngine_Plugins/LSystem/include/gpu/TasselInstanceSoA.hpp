#pragma once

// =============================================================================
//  TasselInstanceSoA — GPU-facing structure-of-arrays for tassel internodes.
//
//  Phase 1a of the GPU L-system migration. Replaces the per-instance
//  ``glm::mat4`` packing that ``MaizeTassel::RebuildGeometry`` currently
//  performs in ``ParticleInfo::instance_matrix``.
//
//  Layout mirrors the GLSL ``TasselInternodeInstance`` declared in
//      Internals/LSystemResources/Shaders/Graphics/Mesh/Tassel/tassel_internode.mesh
//
//  std430 alignment: every member is ``vec4`` so padding is zero and stride
//  is a clean 48 bytes. This dodges the std430 ``vec3`` stride-16 trap that
//  bit the engine previously (see
//  ``EcoSysLab/src/GpuProfileSimulator.cpp``'s flat-uint index trick).
//
//  Why SoA instead of the existing ``ParticleInfo{mat4, vec4}``:
//
//    * Phase 2 growth updates only ``length`` and ``half_thickness`` each
//      frame. With the current ``mat4`` pack the CPU must rebuild the full
//      model matrix per internode per growth step. With this layout the
//      growth compute shader touches 8 bytes per internode (two floats in
//      ``pos_length`` / ``color_thick``) instead of 64 bytes of matrix.
//    * The rotation quaternion is stored directly and evaluated in the mesh
//      shader via a 12-fmul quaternion-rotate, cheaper than uploading a
//      full matrix when only position or length changed.
//
//  Nothing here is wired into the runtime yet; Phase 1b registers the
//  pipeline state object in ``RenderLayer`` and flips the code path on the
//  ``LSYSTEM_GPU_PIPELINE`` macro.
// =============================================================================

#include <cstddef>
#include <cstdint>

#include <glm/glm.hpp>

#if defined(LSYSTEM_GPU_PIPELINE)

namespace l_system_plugin::gpu {

/// One internode rendered as a unit cylinder along local +Y.
///
/// Field packing:
///   pos_length  = (world_pos.x, world_pos.y, world_pos.z, length)
///   rot         = quaternion (x, y, z, w) applied to the unit cylinder's
///                 local +Y axis so it points along the internode's growth
///                 direction. The CPU packer pre-applies the
///                 ``cylinder_axis_fix`` rotation (see MaizeTassel.cpp) so
///                 the mesh shader does a single ``quat_rotate`` per vertex.
///   color_thick = (color.r, color.g, color.b, half_thickness)
struct TasselInternodeInstance {
  glm::vec4 pos_length;
  glm::vec4 rot;
  glm::vec4 color_thick;
};

static_assert(sizeof(TasselInternodeInstance) == 48,
              "TasselInternodeInstance must match std430 stride; update both "
              "this struct and tassel_internode.mesh together.");
static_assert(alignof(TasselInternodeInstance) == 4 ||
                  alignof(TasselInternodeInstance) == 16,
              "Unexpected alignment for TasselInternodeInstance.");
static_assert(offsetof(TasselInternodeInstance, pos_length) == 0, "");
static_assert(offsetof(TasselInternodeInstance, rot) == 16, "");
static_assert(offsetof(TasselInternodeInstance, color_thick) == 32, "");

/// Cylinder tessellation constant shared between CPU and GLSL. Must match
/// the ``RADIAL`` constant in tassel_internode.mesh.
inline constexpr uint32_t kTasselInternodeRadialSegments = 6;

inline constexpr uint32_t kTasselInternodeVerticesPerInstance =
    2u * kTasselInternodeRadialSegments;   // two rings
inline constexpr uint32_t kTasselInternodeTrianglesPerInstance =
    2u * kTasselInternodeRadialSegments;   // two per radial sector

}  // namespace l_system_plugin::gpu

#endif  // LSYSTEM_GPU_PIPELINE
