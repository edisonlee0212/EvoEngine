// =============================================================================
//  tassel_internode.frag — Phase 1b smoke fragment shader.
//
//  Purpose: minimal G-buffer-compatible fragment that proves the
//  tassel_internode mesh-shader pipeline links and renders. Writes the
//  per-instance debug color from the SoA directly into the material slot
//  and the geometric normal into the normal slot.
//
//  Deliberately bypasses material lookup, lighting, and texture sampling.
//  Phase 1b smoke = "PSO survives driver validation + visible cylinders
//  on screen". Material integration is deferred to Phase 1b-final.
// =============================================================================

#extension GL_ARB_shader_draw_parameters : enable
#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_mesh_shader : require

// Inputs from tassel_internode.mesh — locations and field order MUST match
// the MS_V_OUT block exactly.
layout(location = 0) in VS_OUT {
  vec3 FragPos;
  vec3 Normal;
  vec3 Tangent;
  vec2 TexCoord;
  vec4 Color;
}
fs_in;

layout(location = 5) perprimitiveEXT in FS_PRIM_IN {
  int MaterialIndex;
}
fs_prim;

// Match Branches.frag G-buffer layout: attachment 0 = packed normal,
// attachment 1 = albedo / debug payload.
layout(location = 0) out vec4 outNormal;
layout(location = 1) out vec4 outMaterial;

void main() {
  // Front-face-aware normal so back faces shade consistently.
  vec3 n = normalize((gl_FrontFacing ? 1.0 : -1.0) * fs_in.Normal);

  // outMaterial.w >= 2 selects the deferred-lighting "per-instance tinted
  // color stored in rgb" branch (see StandardDeferredLighting.frag). Without
  // this sentinel the lighting pass treats matSample as (texcoord, mat_idx,
  // info_idx) and looks up EE_MATERIAL_PROPERTIES[0] -> default white albedo.
  outNormal = vec4(n, 0.0);
  outMaterial = vec4(fs_in.Color.rgb, 2.0);
}
