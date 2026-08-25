#extension GL_ARB_shading_language_include : enable

layout(location = 0) out vec4 FragColor;
layout(location = 0) in VS_OUT {
  vec2 TexCoord;
}
fs_in;

layout(set = 0, binding = 0) uniform sampler2D originalColor;
layout(set = 0, binding = 1) uniform sampler2D ambientOcclusion;

void main() {
  vec2 texCoord = fs_in.TexCoord;
  vec4 color = texture(originalColor, texCoord);
  vec3 result = color.rgb * (texture(ambientOcclusion, texCoord).r);
  FragColor = vec4(result, color.a);
}