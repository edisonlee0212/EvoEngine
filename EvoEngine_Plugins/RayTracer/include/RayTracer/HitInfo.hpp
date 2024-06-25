#pragma once;
namespace evo_engine {
    struct HitInfo {
        glm::vec3 position = glm::vec3(0.0f);
        glm::vec3 normal = glm::vec3(0.0f);
        glm::vec3 tangent = glm::vec3(0.0f);
        glm::vec4 color = glm::vec4(1.0f);
        glm::vec2 tex_coord = glm::vec2(0.0f);
        glm::vec3 data = glm::vec4(0.0f);
        glm::vec2 data2 = glm::vec4(0.0f);
    };
}