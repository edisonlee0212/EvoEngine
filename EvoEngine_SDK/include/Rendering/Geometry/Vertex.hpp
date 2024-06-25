#pragma once

namespace evo_engine
{
    struct Vertex
    {
        glm::vec3 position = glm::vec3(0.0f);
        float vertex_info1 = 0.0f;
    	glm::vec3 normal = glm::vec3(0.0f);
        float vertex_info2 = 0.0f;
        glm::vec3 tangent = glm::vec3(0.0f);
        float vertex_info3 = 0.0f;
        glm::vec4 color = glm::vec4(1.0f);
        glm::vec2 tex_coord = glm::vec2(0.0f);
        glm::vec2 vertex_info4 = glm::vec2(0.0f);
    };

    struct SkinnedVertex
    {
        glm::vec3 position = glm::vec3(0.0f);
        float vertex_info1 = 0.0f;
        glm::vec3 normal = glm::vec3(0.0f);
        float vertex_info2 = 0.0f;
        glm::vec3 tangent = glm::vec3(0.0f);
        float vertex_info3 = 0.0f;
        glm::vec4 color = glm::vec4(1.0f);
        glm::vec2 tex_coord = glm::vec2(0.0f);
        glm::vec2 vertex_info4 = glm::vec2(0.0f);

        glm::ivec4 bond_id = {};
        glm::vec4 weight = {};
        glm::ivec4 bond_id2 = {};
        glm::vec4 weight2 = {};
    };
    
    struct StrandPoint
    {
        glm::vec3 position = glm::vec3(0.0f);
        float thickness = 0.0f;
        glm::vec3 normal = glm::vec3(0.0f);
        float tex_coord = 0.0f;
        glm::vec4 color = glm::vec4(1.0f);
    };
}