
layout(set = EE_TEXTURES_BLOCK_SET, binding = EE_TEXTURE_2DS_BINDING) uniform sampler2D[] EE_TEXTURE_2DS;
layout(set = EE_TEXTURES_BLOCK_SET, binding = EE_CUBEMAPS_BINDING) uniform samplerCube[] EE_CUBEMAPS;

// General branchless texture sampling with fallback
vec4 EE_SAMPLE_TEXTURE_2D(
    int texIndex,           // texture index (may be -1)
    vec2 uv,                // texture coordinates
    vec4 fallbackValue    // returned when texIndex == -1
) {
    if (texIndex < 0) {
        return fallbackValue;
    }
    return texture(EE_TEXTURE_2DS[texIndex], uv);
}

// General branchless texture sampling with fallback
vec4 EE_SAMPLE_TEXTURE_2D(
    int texIndex,           // texture index (may be -1)
    vec2 uv                // texture coordinates
) {
    return EE_SAMPLE_TEXTURE_2D(texIndex, uv, vec4(0, 0, 0, 0));
}

vec3 EE_SAMPLE_NORMAL(
    int texIndex,
    vec2 uv,
    vec3 fallbackNormal,   // usually fs_in.Normal
    vec3 tangent           // fs_in.Tangent
) {
    if (texIndex < 0) {
        return normalize(fallbackNormal);
    }

    // Sample tangent-space normal
    vec3 sampled = texture(EE_TEXTURE_2DS[texIndex], uv).rgb;
    sampled = sampled * 2.0 - 1.0;

    // Build TBN
    vec3 N = fallbackNormal;
    vec3 T = tangent;
    vec3 B = normalize(cross(N, T));  // ensure orthogonal

    mat3 TBN = mat3(T, B, N);

    // Transform sampled normal
    vec3 mapped = normalize(TBN * sampled);

    return normalize(mapped);
}