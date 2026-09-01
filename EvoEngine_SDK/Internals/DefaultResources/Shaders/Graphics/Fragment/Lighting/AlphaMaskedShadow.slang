import EvoEngine.PerFrame;
import EvoEngine.ShadowRaster;
import EvoEngine.GltfAlphaMask;

[shader("fragment")]
void main(ShadowRasterOutput input) {
  const GltfAlphaTexCoords tex_coords = EE_GLTF_MAKE_ALPHA_TEX_COORDS(
      input.tex_coord0, input.tex_coord1, ddx(input.tex_coord0), ddx(input.tex_coord1), ddy(input.tex_coord0),
      ddy(input.tex_coord1));
  if (!EE_GLTF_ALPHA_MASK_PASSES(uint(EE_INSTANCES[input.instance_index].material_index), tex_coords, input.color.a)) {
    discard;
  }
}
