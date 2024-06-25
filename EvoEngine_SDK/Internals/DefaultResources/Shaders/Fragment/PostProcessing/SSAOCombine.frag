layout (location = 0) out vec4 FragColor;
in VS_OUT {
	vec2 TexCoord;
} fs_in;

uniform sampler2D originalColor;
uniform sampler2D ao;
void main()
{
	vec3 result = texture(originalColor, fs_in.TexCoord).rgb * (1.0 - texture(ao, fs_in.TexCoord).r);
	FragColor = vec4(result, 1.0);
}