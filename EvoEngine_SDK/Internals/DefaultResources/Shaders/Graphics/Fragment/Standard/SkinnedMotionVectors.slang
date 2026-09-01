import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;
import EvoEngine.RasterMotionVectors;
import EvoEngine.GltfRasterMaterial;

float NormalizedLinearDepth(float4 clip) {
  float4x4 projection = EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].projection;
  float far_distance = abs(projection[3][2] / (projection[2][2] + 1.0f));
  return clamp(clip.w / max(far_distance, 1e-5f), 0.0f, 1.0f);
}

[shader("fragment")] float4 main(RasterMotionVectorOutput input) : SV_Target {
  GltfTexCoords tex_coords = EE_GLTF_MAKE_TEX_COORDS_WITH_GRADIENTS(
      input.tex_coord0, input.tex_coord1, ddx(input.tex_coord0), ddx(input.tex_coord1), ddy(input.tex_coord0),
      ddy(input.tex_coord1));
  GltfRasterMaterial surface =
      EE_EVALUATE_GLTF_RASTER_SURFACE(uint(EE_INSTANCES[input.instance_index].material_index), tex_coords, input.color);
  if (EE_GLTF_RASTER_SHOULD_DISCARD(surface))
    discard;
  if (input.previous_pose_valid == 0u || input.current_clip.w <= 0.0001f || input.previous_clip.w <= 0.0001f)
    return float4(65000.0f, 65000.0f, 0.0f, 0.0f);
  float3 current_ndc = input.current_clip.xyz / input.current_clip.w;
  float3 previous_ndc = input.previous_clip.xyz / input.previous_clip.w;
  float2 resolution = float2(EE_CAMERA_RESOLUTION_X(EE_BASIC_CONSTANTS.camera_index),
                             EE_CAMERA_RESOLUTION_Y(EE_BASIC_CONSTANTS.camera_index));
  float2 current_pixel = (current_ndc.xy * 0.5f + float2(0.5f)) * resolution;
  float2 previous_pixel = (previous_ndc.xy * 0.5f + float2(0.5f)) * resolution;
  return float4(previous_pixel - current_pixel,
                NormalizedLinearDepth(input.previous_clip) - NormalizedLinearDepth(input.current_clip), 0.0f);
}
