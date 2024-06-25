layout (location = 0) in vec3 aPos;
layout (location = 11) in vec4 aColor;
layout (location = 12) in mat4 aInstanceMatrix;

uniform mat4 model;

out VS_OUT {
	vec4 aColor;
} vs_out;

void main()
{
	vs_out.aColor = aColor;
	gl_Position = UE_CAMERA_PROJECTION * UE_CAMERA_VIEW * vec4(vec3(aInstanceMatrix * model * vec4(aPos, 1.0)), 1.0);
}