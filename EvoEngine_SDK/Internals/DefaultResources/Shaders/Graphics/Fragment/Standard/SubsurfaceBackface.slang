import EvoEngine.PerFrame;
import EvoEngine.RasterVertex;
import EvoEngine.GBuffer;
import EvoEngine.GltfAlphaMask;
import EvoEngine.Subsurface;

[[vk::binding(17, 2)]] Sampler2D inDepth;
[[vk::binding(24, 2)]] Texture2D<uint4> inMetadata;

[shader("fragment")]
float4 main(StandardRasterOutput input, bool is_front_face : SV_IsFrontFace) : SV_Target0 {
  const uint2 pixel = uint2(input.position.xy);
  const uint4 metadata = inMetadata.Load(int3(pixel, 0));
  uint width, height;
  inMetadata.GetDimensions(width, height);
  const float front_depth = inDepth.SampleLevel((float2(pixel) + 0.5f) / float2(width, height), 0).x;
  if (!EE_SSS_BACKFACE_MATCH(metadata.x, input.instance_index, front_depth, input.position.z,
                            is_front_face == (input.transform_handedness >= 0))) discard;
  const GltfAlphaTexCoords uv = EE_GLTF_MAKE_ALPHA_TEX_COORDS(input.tex_coord01.xy, input.tex_coord01.zw,
      ddx(input.tex_coord01.xy), ddx(input.tex_coord01.zw), ddy(input.tex_coord01.xy), ddy(input.tex_coord01.zw));
  const uint material_index = uint(EE_INSTANCES[input.instance_index].material_index);
  if (EE_GLTF_MATERIALS[material_index].alpha_mode == EE_GLTF_ALPHA_MODE_MASK &&
      !EE_GLTF_ALPHA_MASK_PASSES(material_index, uv, input.color.a)) discard;
  return float4(input.position.z, EE_GBUFFER_OCT_ENCODE(normalize(input.normal)), 1);
}
