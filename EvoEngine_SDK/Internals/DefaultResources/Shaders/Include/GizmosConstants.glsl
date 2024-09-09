
layout(push_constant) uniform EE_GIZMOS_CONSTANTS{
	mat4 EE_MODEL_MATRIX;
	vec4 EE_GIZMO_COLOR;
	float EE_GIZMO_SIZE;
	int EE_CAMERA_INDEX;
};


mat4 EE_GET_SCALE_MATRIX() {
	mat4 temp = mat4(1.0);
	temp[0] = temp[0] * EE_GIZMO_SIZE;
	temp[1] = temp[1] * EE_GIZMO_SIZE;
	temp[2] = temp[2] * EE_GIZMO_SIZE;
	return temp;
}