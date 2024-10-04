#extension GL_EXT_nonuniform_qualifier : enable

#define EE_RENDER_INFO_BLOCK_SET 0
#define EE_RENDER_INFO_BLOCK_BINDING 0
#include "RenderInfo.glsl"
#define EE_ENVIRONMENT_BLOCK_SET 0
#define EE_ENVIRONMENT_BLOCK_BINDING 1
#include "Environment.glsl"
#define EE_CAMERAS_BLOCK_SET 0
#define EE_CAMERAS_BLOCK_BINDING 2
#include "Cameras.glsl"
#define EE_MATERIALS_BLOCK_SET 0
#define EE_MATERIALS_BLOCK_BINDING 3
#include "Materials.glsl"
#define EE_INSTANCES_BLOCK_SET 0
#define EE_INSTANCES_BLOCK_BINDING 4
#include "Instances.glsl"

#define EE_KERNEL_BLOCK_SET 0
#define EE_KERNEL_BLOCK_BINDING 5
#include "Kernel.glsl"

#define EE_LIGHTING_BLOCK_SET 0
#define EE_DIRECTIONAL_LIGHT_BLOCK_BINDING 6
#define EE_POINT_LIGHT_BLOCK_BINDING 7
#define EE_SPOT_LIGHT_BLOCK_BINDING 8
#include "Lights.glsl"

#define EE_TEXTURES_BLOCK_SET 0
#define EE_TEXTURE_2DS_BINDING 9
#define EE_CUBEMAPS_BINDING 10
#include "Textures.glsl"