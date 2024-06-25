layout (location = 0) out vec4 FragColor;

in VS_OUT {
	vec2 TexCoord;
} fs_in;

uniform sampler2D image;
uniform vec4 bezier;
uniform int sampleStep;
uniform float intensity;
uniform int diffusion;
uniform bool horizontal;
uniform float clamp;

float Bezier(float dt){
	float t = 1.0 - dt;
	float t1 = 1.0 - t;
	return t * (3.0 * t1 * t1 * (bezier.xy + bezier.zw) + t * t * vec2(1, 1)).y;
}


void main()
{
	vec2 tex_offset = 1.0 / textureSize(image, 0); // gets size of single texel
	vec4 result = texture(image, fs_in.TexCoord) * intensity;
	float sum = 1.0;
	if(horizontal)
	{
		for(int i = 1; i < diffusion + 1; ++i)
		{
			float bezierVal = Bezier(float(i - 1) / diffusion);
			result += texture(image, fs_in.TexCoord + vec2(tex_offset.x * i * sampleStep, 0.0)).rgba * bezierVal * intensity;
			result += texture(image, fs_in.TexCoord - vec2(tex_offset.x * i * sampleStep, 0.0)).rgba * bezierVal * intensity;
			sum += 2 * bezierVal;
		}
	}
	else
	{
		for(int i = 1; i < diffusion + 1; ++i)
		{
			float bezierVal = Bezier(float(i - 1) / diffusion);
			result += texture(image, fs_in.TexCoord + vec2(0.0, tex_offset.y * i * sampleStep)).rgba * bezierVal * intensity;
			result += texture(image, fs_in.TexCoord - vec2(0.0, tex_offset.y * i * sampleStep)).rgba * bezierVal * intensity;
			sum += 2 * bezierVal;
		}
	}

	result /= sum;
	FragColor = result;
}