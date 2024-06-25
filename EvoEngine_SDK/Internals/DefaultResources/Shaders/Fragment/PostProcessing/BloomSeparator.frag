layout (location = 0) out vec4 FragColor;
layout (location = 1) out vec4 BrightColor;
in VS_OUT {
	vec2 TexCoord;
} fs_in;
uniform float threshold;
uniform sampler2D image;
void main()
{
	vec3 result = texture(image, fs_in.TexCoord).rgb;
	// check whether result is higher than some threshold, if so, output as bloom threshold color
	float brightness = dot(result, vec3(0.2126, 0.7152, 0.0722));
	if(brightness > threshold)
		BrightColor = vec4(result, 1.0);
	else
		BrightColor = vec4(0.0, 0.0, 0.0, 1.0);
	FragColor = vec4(result, 1.0);
}