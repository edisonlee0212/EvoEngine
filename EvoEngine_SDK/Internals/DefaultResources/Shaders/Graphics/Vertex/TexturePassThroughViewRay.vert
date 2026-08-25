layout (location = 0) in vec2 inPosition;
layout (location = 1) in vec2 inTexCoord;

out VS_OUT {
	vec2 TexCoord;
	vec2 ViewRay;
} vs_out;

void main()
{
	vs_out.TexCoord = inTexCoord;
	gl_Position = vec4(inPosition.x, inPosition.y, 0.0, 1.0);
	vs_out.ViewRay = vec2(inPosition.x * EE_CAMERA_RESOLUTION_RATIO(EE_CAMERA_INDEX), inPosition.y) * EE_CAMERA_TAN_HALF_FOV(EE_CAMERA_INDEX);
}  