#extension GL_ARB_shader_draw_parameters : enable
#extension GL_ARB_shading_language_include : enable

#include "PerFrame.glsl"

layout (location = 0) in VS_OUT {
	vec3 frag_pos;
	vec3 normal;
	vec3 tangent;
	vec4 color;
} fs_in;

layout(push_constant) uniform STRANDS_RENDER_CONSTANTS {
  uint camera_index;
  uint strands_size;
  uint strand_segments_size;
};

layout (location = 0) out vec4 out_color;

void main(){
	out_color = vec4(fs_in.color);
}