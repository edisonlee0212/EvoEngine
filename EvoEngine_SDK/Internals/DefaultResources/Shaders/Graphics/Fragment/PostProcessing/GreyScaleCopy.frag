layout (location = 0) out vec4 FragColor;

in VS_OUT {
	vec2 TexCoord;
} fs_in;

uniform sampler2D inputTex;

void main()
{
	float color = texture(inputTex, fs_in.TexCoord).r;
	FragColor = vec4(color, color, color, 1.0);
}