// ABI adapted from Godot renderer_rd/environment/gi.h and shaders/environment/sdfgi_*.glsl,
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
#pragma once

#include <cstddef>
#include <cstdint>

namespace evo_engine {

inline constexpr uint32_t kSdfgiMaxCascades = 8;
inline constexpr uint32_t kSdfgiSolidCellCapacity = 128 * 128 * 128 / 4;
inline constexpr uint32_t kSdfgiFailureSolidOverflow = 1;
inline constexpr uint32_t kSdfgiFailureLightOverflow = 2;
inline constexpr uint32_t kSdfgiFailurePayloadCoverage = 4;

struct alignas(16) SdfgiCascadeData {
  float offset[3]{};
  float to_cell{};
  int32_t probe_world_offset[3]{};
  uint32_t pad{};
  float pad2[4]{};
};
static_assert(sizeof(SdfgiCascadeData) == 48);

struct alignas(16) SdfgiGatherCascadeData {
  float position[3]{};
  float to_probe{};
  int32_t probe_world_offset[3]{};
  float to_cell{};
  float pad[3]{};
  float exposure_normalization{};
};
static_assert(sizeof(SdfgiGatherCascadeData) == 48);

struct alignas(16) SdfgiPreprocessPushConstant {
  int32_t scroll[3]{};
  int32_t grid_size{};
  int32_t probe_offset[3]{};
  int32_t step_size{};
  uint32_t half_size{};
  uint32_t occlusion_index{};
  int32_t cascade{};
  uint32_t pad{};
};
static_assert(sizeof(SdfgiPreprocessPushConstant) == 48);

struct alignas(16) SdfgiDirectLightPushConstant {
  float grid_size[3]{};
  uint32_t max_cascades{};
  uint32_t cascade{};
  uint32_t light_count{};
  uint32_t process_offset{};
  uint32_t process_increment{};
  int32_t probe_axis_size{};
  float bounce_feedback{};
  float y_mult{};
  uint32_t use_occlusion{};
};
static_assert(sizeof(SdfgiDirectLightPushConstant) == 48);

struct alignas(16) SdfgiIntegratePushConstant {
  float grid_size[3]{};
  uint32_t max_cascades{};
  uint32_t probe_axis_size{};
  uint32_t cascade{};
  uint32_t history_index{};
  uint32_t history_size{};
  uint32_t ray_count{};
  float ray_bias{};
  int32_t image_size[2]{};
  int32_t world_offset[3]{};
  uint32_t sky_flags{};
  int32_t scroll[3]{};
  float sky_energy{};
  float sky_color_or_orientation[3]{};
  float y_mult{};
  float sky_lod_inverse_gamma[2]{};
  uint32_t store_ambient_texture{};
  uint32_t pad{};
};
static_assert(sizeof(SdfgiIntegratePushConstant) == 112);
static_assert(offsetof(SdfgiIntegratePushConstant, sky_lod_inverse_gamma) == 96);

struct alignas(16) SdfgiLight {
  float color[3]{};
  float energy{};
  float direction[3]{};
  uint32_t has_shadow{};
  float position[3]{};
  float attenuation{};
  uint32_t type{};
  float cos_spot_angle{};
  float inv_spot_attenuation{};
  float radius{};
  float area_width[4]{};
  float area_height[4]{};
  float area_projector_rect[4]{};
  float host_photometry[4]{};
};
static_assert(sizeof(SdfgiLight) == 128);

struct alignas(16) SdfgiSolidCell {
  uint32_t position{};
  uint32_t albedo{};
  uint32_t light{};
  uint32_t light_aniso{};
};
static_assert(sizeof(SdfgiSolidCell) == 16);

struct alignas(16) SdfgiDispatchData {
  uint32_t x{};
  uint32_t y{};
  uint32_t z{};
  uint32_t total_count{};
};
static_assert(sizeof(SdfgiDispatchData) == 16);

struct alignas(16) SdfgiFieldStatus {
  uint32_t ready{};
  uint32_t failure_flags{};
  uint32_t generation{};
  uint32_t solid_cell_capacity{};
};
static_assert(sizeof(SdfgiFieldStatus) == 16);

struct alignas(16) SdfgiGatherData {
  float grid_size[3]{};
  uint32_t max_cascades{};
  uint32_t use_occlusion{};
  int32_t probe_axis_size{};
  float probe_to_uvw{};
  float normal_bias{};
  float lightprobe_tex_pixel_size[3]{};
  float energy{};
  float lightprobe_uv_offset[3]{};
  float y_mult{};
  float occlusion_clamp[3]{};
  uint32_t pad3{};
  float occlusion_renormalize[3]{};
  uint32_t pad4{};
  float cascade_probe_size[3]{};
  uint32_t pad5{};
  SdfgiGatherCascadeData cascades[8]{};
  float anchor_origin[3]{};
  uint32_t generation{};
};
static_assert(sizeof(SdfgiGatherData) == 512);

struct alignas(16) SdfgiCascadeBlock {
  SdfgiCascadeData data[8]{};
};
static_assert(sizeof(SdfgiCascadeBlock) == 384);

struct alignas(16) SdfgiVoxelData {
  float view_projection[4][4]{};
  float cascade_min_cell[4]{};
  float region_offset_y_mult[4]{};
};
static_assert(sizeof(SdfgiVoxelData) == 96);

struct alignas(16) SdfgiVoxelPushConstant {
  float model[4][4]{};
  float normal_basis[9]{};
  uint32_t material_index{};
  uint32_t pad[2]{};
};
static_assert(sizeof(SdfgiVoxelPushConstant) == 112);
static_assert(offsetof(SdfgiVoxelPushConstant, material_index) == 100);

static_assert(offsetof(SdfgiLight, host_photometry) == 112);
static_assert(offsetof(SdfgiGatherData, cascades) == 112);
static_assert(offsetof(SdfgiGatherData, anchor_origin) == 496);
static_assert(offsetof(SdfgiIntegratePushConstant, sky_flags) == 60);
static_assert(offsetof(SdfgiIntegratePushConstant, store_ambient_texture) == 104);

}  // namespace evo_engine
