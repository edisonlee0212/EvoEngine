#extension GL_EXT_mesh_shader : require
#extension GL_ARB_shading_language_include : enable

#define EE_STRAND_SHADOW_LIGHT_MATRIX EE_SPOT_LIGHTS[EE_CAMERA_INDEX].light_space_matrix
#include "StrandShadowMesh.slangh"
