
layout(set = EE_TEXTURES_BLOCK_SET, binding = EE_TEXTURE_2DS_BINDING) uniform sampler2D[] EE_TEXTURE_2DS;
layout(set = EE_TEXTURES_BLOCK_SET, binding = EE_CUBEMAPS_BINDING) uniform samplerCube[] EE_CUBEMAPS;

// General branchless texture sampling with fallback
vec4 EE_SAMPLE_TEXTURE_2D(
    int texIndex,           // texture index (may be -1)
    vec2 uv,                // texture coordinates
    vec4 fallbackValue    // returned when texIndex == -1
) {
    // 1. Determine if the map exists (1.0 or 0.0, branchless)
    float hasMap = texIndex != -1 ? 1.0 : 0.0;

    // 2. Use texture 0 as safe dummy if texIndex == -1
    int validIndex = max(texIndex, 0);

    // 3. Sample texture (still done even if unused — avoids branches)
    vec4 texel = texture(EE_TEXTURE_2DS[validIndex], uv);

    // 4. Blend between fallback and sampled (branchless)
    return mix(fallbackValue, texel, hasMap);
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
    // Determine if a normal map exists
    float hasMap = texIndex != -1 ? 1.0 : 0.0;

    // Use texture 0 if texIndex == -1
    int validIndex = max(texIndex, 0);

    // Sample tangent-space normal
    vec3 sampled = texture(EE_TEXTURE_2DS[validIndex], uv).rgb;
    sampled = sampled * 2.0 - 1.0;

    // Build TBN
    vec3 N = fallbackNormal;
    vec3 T = tangent;
    vec3 B = normalize(cross(N, T));  // ensure orthogonal

    mat3 TBN = mat3(T, B, N);

    // Transform sampled normal
    vec3 mapped = normalize(TBN * sampled);

    // Mix based on whether we have a normal map
    return normalize(mix(N, mapped, hasMap));
}