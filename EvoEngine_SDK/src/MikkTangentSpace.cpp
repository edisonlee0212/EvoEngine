#include "MikkTangentSpace.hpp"

#include "ThirdParty/MikkTSpace/mikktspace.h"

#include <algorithm>

namespace {
template <typename VertexType>
glm::vec2 ReadTexCoord(const VertexType& vertex, const int tex_coord) {
  switch (tex_coord) {
    case 1:
      return vertex.tex_coord_1;
    case 2:
      return vertex.tex_coord_2;
    case 3:
      return vertex.tex_coord_3;
    default:
      return vertex.tex_coord;
  }
}

template <typename VertexType>
struct MikkMeshData {
  std::vector<VertexType>* vertices;
  std::vector<glm::uvec3>* triangles;
  std::vector<glm::vec4> corner_tangents;
  int tex_coord;
};

template <typename VertexType>
const VertexType& GetCornerVertex(const SMikkTSpaceContext* context, const int face, const int corner) {
  const auto& data = *static_cast<MikkMeshData<VertexType>*>(context->m_pUserData);
  return data.vertices->at(data.triangles->at(face)[corner]);
}

template <typename VertexType>
int GetFaceCount(const SMikkTSpaceContext* context) {
  return static_cast<int>(static_cast<MikkMeshData<VertexType>*>(context->m_pUserData)->triangles->size());
}

int GetVertexCount(const SMikkTSpaceContext*, int) {
  return 3;
}

template <typename VertexType>
void GetPosition(const SMikkTSpaceContext* context, float output[], const int face, const int corner) {
  const auto& value = GetCornerVertex<VertexType>(context, face, corner).position;
  output[0] = value.x;
  output[1] = value.y;
  output[2] = value.z;
}

template <typename VertexType>
void GetNormal(const SMikkTSpaceContext* context, float output[], const int face, const int corner) {
  const auto& value = GetCornerVertex<VertexType>(context, face, corner).normal;
  output[0] = value.x;
  output[1] = value.y;
  output[2] = value.z;
}

template <typename VertexType>
void GetTexCoord(const SMikkTSpaceContext* context, float output[], const int face, const int corner) {
  const auto& data = *static_cast<MikkMeshData<VertexType>*>(context->m_pUserData);
  const auto value = ReadTexCoord(GetCornerVertex<VertexType>(context, face, corner), data.tex_coord);
  output[0] = value.x;
  output[1] = value.y;
}

template <typename VertexType>
void SetTangent(const SMikkTSpaceContext* context, const float tangent[], const float sign, const int face,
                const int corner) {
  auto& data = *static_cast<MikkMeshData<VertexType>*>(context->m_pUserData);
  data.corner_tangents[static_cast<size_t>(face) * 3 + corner] =
      glm::vec4(tangent[0], tangent[1], tangent[2], sign < 0.0f ? -1.0f : 1.0f);
}

glm::vec3 FallbackTangent(const glm::vec3& normal) {
  const auto axis = glm::abs(normal.x) < 0.9f ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
  const auto tangent = glm::cross(axis, normal);
  const auto length_squared = glm::dot(tangent, tangent);
  return length_squared > 0.0f ? tangent * glm::inversesqrt(length_squared) : glm::vec3(1.0f, 0.0f, 0.0f);
}

bool CompatibleTangents(const glm::vec4& lhs, const glm::vec4& rhs) {
  return lhs.w == rhs.w && glm::dot(glm::vec3(lhs), glm::vec3(rhs)) >= 1.0f - 1e-5f;
}

template <typename VertexType>
void Generate(std::vector<VertexType>& vertices, std::vector<glm::uvec3>& triangles, const int tex_coord) {
  if (vertices.empty() || triangles.empty()) {
    return;
  }
  MikkMeshData<VertexType> data{&vertices, &triangles, std::vector<glm::vec4>(triangles.size() * 3),
                                glm::clamp(tex_coord, 0, 3)};
  SMikkTSpaceInterface mikk_interface{};
  mikk_interface.m_getNumFaces = GetFaceCount<VertexType>;
  mikk_interface.m_getNumVerticesOfFace = GetVertexCount;
  mikk_interface.m_getPosition = GetPosition<VertexType>;
  mikk_interface.m_getNormal = GetNormal<VertexType>;
  mikk_interface.m_getTexCoord = GetTexCoord<VertexType>;
  mikk_interface.m_setTSpaceBasic = SetTangent<VertexType>;
  SMikkTSpaceContext context{&mikk_interface, &data};
  genTangSpaceDefault(&context);

  struct TangentGroup {
    glm::vec4 tangent;
    uint32_t vertex_index;
  };
  const size_t source_vertex_count = vertices.size();
  std::vector<std::vector<TangentGroup>> groups(source_vertex_count);
  for (size_t face = 0; face < triangles.size(); ++face) {
    for (int corner = 0; corner < 3; ++corner) {
      const uint32_t source_index = triangles[face][corner];
      auto tangent = data.corner_tangents[face * 3 + corner];
      if (glm::dot(glm::vec3(tangent), glm::vec3(tangent)) <= 0.0f) {
        tangent = glm::vec4(FallbackTangent(vertices[source_index].normal), 1.0f);
      }
      auto& source_groups = groups[source_index];
      auto match = std::find_if(source_groups.begin(), source_groups.end(), [&](const TangentGroup& group) {
        return CompatibleTangents(group.tangent, tangent);
      });
      if (match == source_groups.end()) {
        const uint32_t target_index = source_groups.empty() ? source_index : static_cast<uint32_t>(vertices.size());
        if (!source_groups.empty()) {
          vertices.emplace_back(vertices[source_index]);
        }
        vertices[target_index].tangent = glm::vec3(tangent);
        vertices[target_index].vertex_info3 = tangent.w;
        source_groups.push_back({tangent, target_index});
        match = std::prev(source_groups.end());
      }
      triangles[face][corner] = match->vertex_index;
    }
  }
}
}  // namespace

void evo_engine::GenerateMikkTangents(std::vector<Vertex>& vertices, std::vector<glm::uvec3>& triangles,
                                      const int tex_coord) {
  Generate(vertices, triangles, tex_coord);
}

void evo_engine::GenerateMikkTangents(std::vector<SkinnedVertex>& vertices, std::vector<glm::uvec3>& triangles,
                                      const int tex_coord) {
  Generate(vertices, triangles, tex_coord);
}
