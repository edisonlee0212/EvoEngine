public struct Camera {
  public float4x4 projection;
  public float4x4 view;
  public float4x4 projection_view;
  public float4x4 inverse_projection;
  public float4x4 inverse_view;
  public float4x4 inverse_projection_view;
  public float4x4 previous_projection_view;
  public float4x4 previous_inverse_projection;
  public float4x4 previous_inverse_view;
  public float4x4 unjittered_projection_view;
  public float4x4 previous_unjittered_projection_view;
  public float4 clear_color;
  public float4 jitter;
  public float resolution_x;
  public float resolution_y;
  public float fade_ratio;
  public float fade_factor;
  public int skybox_tex_index;
  public int irradiance_map_index;
  public int prefiltered_map_index;
  public int background_source;

  // Ray tracing
  public uint camera_block_reserved3;
  public float gamma;
  public uint sample_size;
  public uint bounce;
  public float firefly_clamp_threshold;
  public uint auto_spp_enabled;
  public uint auto_spp_min_samples;
  public uint auto_spp_max_samples;
  public float auto_spp_convergence_threshold;
  public uint camera_block_reserved4;
  public uint ray_debug_view;
  public uint raster_lighting_flags;
  public uint ray_output_flags;
  public uint camera_block_reserved0;
  public uint camera_block_reserved1;
  public uint camera_block_reserved2;
  public float4 shadow_split_distances;
};

public static const uint EE_CAMERA_RAY_OUTPUT_ALBEDO = 1u << 0u;
public static const uint EE_CAMERA_RAY_OUTPUT_NORMAL = 1u << 1u;
public static const uint EE_CAMERA_RAY_OUTPUT_RAY_COUNT = 1u << 2u;
public static const uint EE_CAMERA_RAY_OUTPUT_PATH_LENGTH = 1u << 3u;
public static const uint EE_CAMERA_RAY_OUTPUT_TIME = 1u << 4u;
public static const uint EE_CAMERA_RAY_OUTPUT_DEBUG = 1u << 5u;
public bool EE_CAMERA_RAY_OUTPUT_ENABLED(const Camera camera, const uint output_flag) {
  return (camera.ray_output_flags & output_flag) != 0u;
}

[[vk::binding(2, 0)]]
public StructuredBuffer<Camera, Std430DataLayout> EE_CAMERAS;

public float3 EE_DEPTH_TO_CLIP_POS(float2 tex_coords, float ndcDepth);
public float3 EE_DEPTH_TO_WORLD_POS(int camera_index, float2 tex_coords, float ndcDepth);
public float3 EE_DEPTH_TO_VIEW_POS(int camera_index, float2 tex_coords, float ndcDepth);

public float3 EE_CAMERA_UP(int camera_index) {
  return EE_CAMERAS[camera_index].view[1].xyz;
}

public float3 EE_CAMERA_POSITION(int camera_index) {
  return EE_CAMERAS[camera_index].inverse_view[3].xyz;
}

public float EE_CAMERA_NEAR(int camera_index) {
  float a = EE_CAMERAS[camera_index].projection[2][2];
  float b = EE_CAMERAS[camera_index].projection[3][2];
  return abs(b / a);
}

public float EE_CAMERA_FAR(int camera_index) {
  float a = EE_CAMERAS[camera_index].projection[2][2];
  float b = EE_CAMERAS[camera_index].projection[3][2];
  return abs(b / (a + 1.f));
}

public float EE_CAMERA_RESOLUTION_X(int camera_index) {
  return EE_CAMERAS[camera_index].resolution_x;
}

public float EE_CAMERA_RESOLUTION_Y(int camera_index) {
  return EE_CAMERAS[camera_index].resolution_y;
}

public float EE_CAMERA_FADE_RATIO(int camera_index) {
  return EE_CAMERAS[camera_index].fade_ratio;
}

public float EE_LINEARIZE_DEPTH(int camera_index, float ndcDepth) {
  float near = EE_CAMERA_NEAR(camera_index);
  float far = EE_CAMERA_FAR(camera_index);
  return near * far / (far - ndcDepth * (far - near));
}

public float3 EE_DEPTH_TO_WORLD_POS(int camera_index, float2 tex_coords, float ndcDepth) {
  float4 clipPos = float4(EE_DEPTH_TO_CLIP_POS(tex_coords, ndcDepth), 1.0);
  float4 worldPos = mul(clipPos, EE_CAMERAS[camera_index].inverse_projection_view);
  worldPos = worldPos / worldPos.w;
  return worldPos.xyz;
}

public float3 EE_DEPTH_TO_VIEW_POS(int camera_index, float2 tex_coords, float ndcDepth) {
  float4 clipPos = float4(EE_DEPTH_TO_CLIP_POS(tex_coords, ndcDepth), 1.0);
  float4 viewPos = mul(clipPos, EE_CAMERAS[camera_index].inverse_projection);
  viewPos = viewPos / viewPos.w;
  return viewPos.xyz;
}

public float3 EE_DEPTH_TO_CLIP_POS(float2 tex_coords, float ndcDepth) {
  float4 clipPos = float4(tex_coords * 2 - float2(1), ndcDepth, 1.0);
  return clipPos.xyz;
}

public float EE_PIXEL_DISTANCE(int camera_index, in float3 worldPosA, in float3 worldPosB) {
  float4 coordA = mul(float4(worldPosA, 1.0), EE_CAMERAS[camera_index].projection_view);
  float4 coordB = mul(float4(worldPosB, 1.0), EE_CAMERAS[camera_index].projection_view);
  float2 screenSize = float2(EE_CAMERA_RESOLUTION_X(camera_index), EE_CAMERA_RESOLUTION_Y(camera_index));
  coordA = coordA / coordA.w;
  coordB = coordB / coordB.w;
  return distance(coordA.xy * screenSize / 2.0, coordB.xy * screenSize / 2.0);
}
