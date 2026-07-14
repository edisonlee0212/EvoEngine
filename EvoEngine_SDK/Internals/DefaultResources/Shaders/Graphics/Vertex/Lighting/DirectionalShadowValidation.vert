layout(location = 0) in vec3 inPosition;

layout(push_constant) uniform EE_CSM_VALIDATION_PUSH_CONSTANT {
  mat4 transform;
};

void main() {
  gl_Position = transform * vec4(inPosition, 1.0f);
}
