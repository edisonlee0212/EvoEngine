precision highp float;

layout (location = 0) out vec4 FragColor;

layout (location = 0) in VS_OUT {
    vec2 TexCoord;
} fs_in;

layout(set = 0, binding = 0) uniform sampler2D image;

void main()
{
    vec2 tex_offset = 1.0 / textureSize(image, 0); // gets size of single texel
    vec4 result = texture(image, fs_in.TexCoord).rgba * weight[0]; // current fragment's contribution
    if(horizontal != 0)
    {
        for(int i = 1; i < 5; ++i)
        {
            result += texture(image, fs_in.TexCoord + vec2(tex_offset.x * i, 0.0)).rgba * weight[i];
            result += texture(image, fs_in.TexCoord - vec2(tex_offset.x * i, 0.0)).rgba * weight[i];
        }
    }
    else
    {
        for(int i = 1; i < 5; ++i)
        {
            result += texture(image, fs_in.TexCoord + vec2(0.0, tex_offset.y * i)).rgba * weight[i];
            result += texture(image, fs_in.TexCoord - vec2(0.0, tex_offset.y * i)).rgba * weight[i];
        }
    }
    FragColor = result;
}