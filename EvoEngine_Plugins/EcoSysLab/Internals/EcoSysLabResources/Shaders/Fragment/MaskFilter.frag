out vec3 FragColor;

in VS_OUT {
	vec2 TexCoords;
} vs_in;

uniform sampler2D InputTex;
uniform sampler2D MaskTex;
uniform float IgnoreMaxHeight;
uniform float IgnoreWidth;
void main()
{
	float delta = 1.0 / textureSize(InputTex, 0).y;
	vec3 originalColor = texture(InputTex, vs_in.TexCoords).rgb;
	vec3 maskColor = texture(MaskTex, vs_in.TexCoords).rgb;
	if(maskColor != vec3(1.0, 1.0, 1.0))
	{
		originalColor = vec3(0.0);
	}
	if(vs_in.TexCoords.x > 0.5 - IgnoreWidth && vs_in.TexCoords.x < 0.5 + IgnoreWidth && vs_in.TexCoords.y > 0.0 && vs_in.TexCoords.y < IgnoreMaxHeight) originalColor = vec3(0.0);
	FragColor = originalColor;
}