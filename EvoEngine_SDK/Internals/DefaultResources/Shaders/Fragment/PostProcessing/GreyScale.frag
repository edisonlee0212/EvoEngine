layout (location = 0) out float FragColor;

in VS_OUT {
	vec2 TexCoord;
} fs_in;

uniform sampler2D inputTex;

void main()
{
	vec3 color = texture(inputTex, fs_in.TexCoord).rgb;
	FragColor = (color.x + color.y + color.z) / 3.0;
}