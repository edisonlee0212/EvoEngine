// ABI adapted from Godot renderer_rd/environment/gi.h and shaders/environment/sdfgi_*.glsl,
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.

struct SdfgiCascadeData {
  float3 offset;
  float to_cell;
  int3 probe_world_offset;
  uint pad;
  float4 pad2;
};

struct SdfgiGatherCascadeData {
  float3 position;
  float to_probe;
  int3 probe_world_offset;
  float to_cell;
  float3 pad;
  float exposure_normalization;
};

struct SdfgiPreprocessPushConstant {
  int3 scroll;
  int grid_size;
  int3 probe_offset;
  int step_size;
  uint half_size;
  uint occlusion_index;
  int cascade;
  uint pad;
};

struct SdfgiDirectLightPushConstant {
  float3 grid_size;
  uint max_cascades;
  uint cascade;
  uint light_count;
  uint process_offset;
  uint process_increment;
  int probe_axis_size;
  float bounce_feedback;
  float y_mult;
  uint use_occlusion;
};

struct SdfgiIntegratePushConstant {
  float3 grid_size;
  uint max_cascades;
  uint probe_axis_size;
  uint cascade;
  uint history_index;
  uint history_size;
  uint ray_count;
  float ray_bias;
  int2 image_size;
  int3 world_offset;
  uint sky_flags;
  int3 scroll;
  float sky_energy;
  float3 sky_color_or_orientation;
  float y_mult;
  float2 sky_irradiance_border_size;
  uint store_ambient_texture;
  uint pad;
};

struct SdfgiLight {
  float3 color;
  float energy;
  float3 direction;
  uint has_shadow;
  float3 position;
  float attenuation;
  uint type;
  float cos_spot_angle;
  float inv_spot_attenuation;
  float radius;
  float4 area_width;
  float4 area_height;
  float4 area_projector_rect;
  float4 host_photometry;
};

struct SdfgiSolidCell {
  uint position;
  uint albedo;
  uint light;
  uint light_aniso;
};

struct SdfgiDispatchData {
  uint x;
  uint y;
  uint z;
  uint total_count;
};

struct SdfgiFieldStatus {
  uint ready;
  uint failure_flags;
  uint generation;
  uint solid_cell_capacity;
};

struct SdfgiGatherData {
  float3 grid_size;
  uint max_cascades;
  uint use_occlusion;
  int probe_axis_size;
  float probe_to_uvw;
  float normal_bias;
  float3 lightprobe_tex_pixel_size;
  float energy;
  float3 lightprobe_uv_offset;
  float y_mult;
  float3 occlusion_clamp;
  uint pad3;
  float3 occlusion_renormalize;
  uint pad4;
  float3 cascade_probe_size;
  uint pad5;
  SdfgiGatherCascadeData cascades[8];
  float3 anchor_origin;
  uint generation;
};

struct SdfgiCascadeBlock {
  SdfgiCascadeData data[8];
};

struct SdfgiVoxelData {
  float4 view_projection[4];
  float4 region_min;
  float4 region_max;
};

static const uint SDFGI_MAX_CASCADES = 8;
static const uint SDFGI_FAILURE_SOLID_OVERFLOW = 1;
static const uint SDFGI_FAILURE_LIGHT_OVERFLOW = 2;
