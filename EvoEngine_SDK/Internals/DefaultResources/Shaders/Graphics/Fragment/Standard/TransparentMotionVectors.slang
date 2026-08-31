import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;
import EvoEngine.RasterMotionVectors;
import EvoEngine.GltfRasterMaterial;

[shader("fragment")] float4 main(RasterMotionVectorOutput input) : SV_Target {
  GltfTexCoords tex_coords = EE_GLTF_MAKE_TEX_COORDS_WITH_GRADIENTS(
      input.tex_coord0, input.tex_coord1, input.tex_coord2, input.tex_coord3, ddx(input.tex_coord0),
      ddx(input.tex_coord1), ddx(input.tex_coord2), ddx(input.tex_coord3), ddy(input.tex_coord0), ddy(input.tex_coord1),
      ddy(input.tex_coord2), ddy(input.tex_coord3));
  GltfRasterMaterial surface =
      EE_EVALUATE_GLTF_RASTER_SURFACE(uint(EE_INSTANCES[input.instance_index].material_index), tex_coords, input.color);
  if (EE_GLTF_RASTER_SHOULD_DISCARD(surface))
    discard;
  if (input.previous_pose_valid == 0u || input.current_clip.w <= 0.0001f || input.previous_clip.w <= 0.0001f)
    return float4(65000.0f, 65000.0f, 0.0f, 0.0f);
  float2 resolution = float2(EE_CAMERA_RESOLUTION_X(EE_BASIC_CONSTANTS.camera_index),
                             EE_CAMERA_RESOLUTION_Y(EE_BASIC_CONSTANTS.camera_index));
  float2 current_pixel = (input.current_clip.xy / input.current_clip.w * 0.5f + float2(0.5f)) * resolution;
  float2 previous_pixel = (input.previous_clip.xy / input.previous_clip.w * 0.5f + float2(0.5f)) * resolution;
  return float4(previous_pixel - current_pixel, 0.0f, 0.0f);
}
