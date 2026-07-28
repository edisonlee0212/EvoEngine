


// This shader performs downsampling on a texture,
// as taken from Call Of Duty method, presented at ACM Siggraph 2014.
// This particular method was customly designed to eliminate
// "pulsating artifacts and temporal stability issues".

// Remember to add bilinear minification filter for this texture!
// Remember to use a floating-point texture format (for HDR)!
// Remember to use edge clamping for this texture!
layout(set = 1, binding = 0) uniform sampler2D srcTexture;

layout (location = 0) in VS_OUT {
	vec2 TexCoord;
} fs_in;

layout (location = 0) out vec4 upsample;
layout(push_constant) uniform EE_BLOOM_CONSTANTS{
	float filterRadius;
};
void main()
{
	vec2 texCoord = fs_in.TexCoord;
	// The filter kernel is applied with a radius, specified in texture
	// coordinates, so that the radius will vary across mip resolutions.
	float x = filterRadius;
	float y = filterRadius;

	// Take 9 samples around current texel:
	// a - b - c
	// d - e - f
	// g - h - i
	// === ('e' is the current texel) ===
	vec4 a = texture(srcTexture, vec2(texCoord.x - x, texCoord.y + y)).rgba;
	vec4 b = texture(srcTexture, vec2(texCoord.x,     texCoord.y + y)).rgba;
	vec4 c = texture(srcTexture, vec2(texCoord.x + x, texCoord.y + y)).rgba;

	vec4 d = texture(srcTexture, vec2(texCoord.x - x, texCoord.y)).rgba;
	vec4 e = texture(srcTexture, vec2(texCoord.x,     texCoord.y)).rgba;
	vec4 f = texture(srcTexture, vec2(texCoord.x + x, texCoord.y)).rgba;

	vec4 g = texture(srcTexture, vec2(texCoord.x - x, texCoord.y - y)).rgba;
	vec4 h = texture(srcTexture, vec2(texCoord.x,     texCoord.y - y)).rgba;
	vec4 i = texture(srcTexture, vec2(texCoord.x + x, texCoord.y - y)).rgba;

	// Apply weighted distribution, by using a 3x3 tent filter:
	//  1   | 1 2 1 |
	// -- * | 2 4 2 |
	// 16   | 1 2 1 |
	upsample = e * 4.0f;
	upsample += (b + d + f + h)* 2.0f;
	upsample += (a + c + g + i);
	upsample *= 1.0f / 16.0f;
}