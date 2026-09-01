import EvoEngine.PerFrame;
import EvoEngine.RasterVertex;
import EvoEngine.RawGBuffer;
import EvoEngine.GltfAlphaMask;

[shader("fragment")]
RawGBufferOutput main(StandardRasterOutput input, bool is_front_face : SV_IsFrontFace) {
  const uint instance_index = input.instance_index;
  const Instance instance = EE_INSTANCES[instance_index];
  const uint material_index = uint(instance.material_index);
  const GltfAlphaTexCoords tex_coords = EE_GLTF_MAKE_ALPHA_TEX_COORDS(
      input.tex_coord01.xy, input.tex_coord01.zw, ddx(input.tex_coord01.xy), ddx(input.tex_coord01.zw),
      ddy(input.tex_coord01.xy), ddy(input.tex_coord01.zw));
  if (!EE_GLTF_ALPHA_MASK_PASSES(material_index, tex_coords, input.color.a)) {
    discard;
  }
  const float facing_sign = (is_front_face ? 1.0f : -1.0f) * input.transform_handedness;
  return EE_BUILD_RAW_GBUFFER(input.tex_coord01.xy, input.tex_coord01.zw, facing_sign * input.normal,
                              facing_sign * input.tangent, input.tangent_handedness * facing_sign, instance_index,
                              material_index, uint(instance.info_index), input.color, false);
}
