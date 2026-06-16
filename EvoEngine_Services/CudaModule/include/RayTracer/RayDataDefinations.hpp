#pragma once

#include "Optix7.hpp"

#include "Vertex.hpp"

#include "CurveSplineDefinations.hpp"
#include "Enums.hpp"
#include "MaterialProperties.hpp"
#include "PointCloudSample.hpp"
#include "optix_device.h"
#ifdef __CUDACC__
#include <texture_indirect_functions.h>
#endif
namespace evo_engine {
static __forceinline__ __device__ float3 GetHitPoint() {
  const float t = optixGetRayTmax();
  const float3 ray_origin = optixGetWorldRayOrigin();
  const float3 ray_direction = optixGetWorldRayDirection();

  return ray_origin + ray_direction * t;
}

struct Curves {
  evo_engine::StrandPoint *strand_points = nullptr;
  // The starting index of point where this segment starts;
  int *segments = nullptr;
  // The start and end's U for current segment for entire strand.
  // glm::vec2 *m_strandU = nullptr;
  // The index of strand this segment belongs.
  // int *m_strandIndices = nullptr;

  // Get curve hit-point in world coordinates.
  __device__ HitInfo GetHitInfo() const {
    HitInfo hit_info;
    const unsigned int primitive_index = optixGetPrimitiveIndex();
    auto type = optixGetPrimitiveType();
    float3 hit_point_internal = GetHitPoint();
    // interpolators work in object space
    hit_point_internal = optixTransformPointFromWorldToObjectSpace(hit_point_internal);
    hit_info.position = glm::vec3(hit_point_internal.x, hit_point_internal.y, hit_point_internal.z);

    switch (type) {
      case OPTIX_PRIMITIVE_TYPE_ROUND_LINEAR: {
        LinearBSplineSegment interpolator(&strand_points[segments[primitive_index]]);
        const auto u = optixGetCurveParameter();
        hit_info.normal = surfaceNormal(interpolator, u, hit_info.position);
        hit_info.tex_coord = interpolator.texCoord(u);
        hit_info.color = interpolator.color(u);
      } break;
      case OPTIX_PRIMITIVE_TYPE_ROUND_QUADRATIC_BSPLINE: {
        QuadraticBSplineSegment interpolator(&strand_points[segments[primitive_index]]);
        const auto u = optixGetCurveParameter();
        hit_info.normal = surfaceNormal(interpolator, u, hit_info.position);
        hit_info.tex_coord = interpolator.texCoord(u);
        hit_info.color = interpolator.color(u);
      } break;
      case OPTIX_PRIMITIVE_TYPE_ROUND_CUBIC_BSPLINE: {
        CubicBSplineSegment interpolator(&strand_points[segments[primitive_index]]);
        const auto u = optixGetCurveParameter();
        hit_info.normal = surfaceNormal(interpolator, u, hit_info.position);
        hit_info.tex_coord = interpolator.texCoord(u);
        hit_info.color = interpolator.color(u);
      } break;
    }
    hit_info.tangent = glm::cross(hit_info.normal, glm::vec3(hit_info.normal.y, hit_info.normal.z, hit_info.normal.x));
    return hit_info;
  }

  // Compute surface normal of quadratic primitive in world space.
  __forceinline__ __device__ glm::vec3 NormalLinear(const int primitive_index) const {
    LinearBSplineSegment interpolator(&strand_points[segments[primitive_index]]);
    float3 hit_point_internal = GetHitPoint();
    // interpolators work in object space
    hit_point_internal = optixTransformPointFromWorldToObjectSpace(hit_point_internal);
    glm::vec3 hit_point = glm::vec3(hit_point_internal.x, hit_point_internal.y, hit_point_internal.z);
    const auto normal = surfaceNormal(interpolator, optixGetCurveParameter(), hit_point);
    return normal;
  }

  // Compute surface normal of quadratic primitive in world space.
  __forceinline__ __device__ glm::vec3 NormalQuadratic(const int primitive_index) const {
    QuadraticBSplineSegment interpolator(&strand_points[segments[primitive_index]]);
    float3 hit_point_internal = GetHitPoint();
    // interpolators work in object space
    hit_point_internal = optixTransformPointFromWorldToObjectSpace(hit_point_internal);
    glm::vec3 hit_point = glm::vec3(hit_point_internal.x, hit_point_internal.y, hit_point_internal.z);
    const auto normal = surfaceNormal(interpolator, optixGetCurveParameter(), hit_point);
    return normal;
  }

  // Compute surface normal of cubic primitive in world space.
  __forceinline__ __device__ glm::vec3 NormalCubic(const int primitive_index) const {
    CubicBSplineSegment interpolator(&strand_points[segments[primitive_index]]);
    float3 hit_point_internal = GetHitPoint();
    // interpolators work in object space
    hit_point_internal = optixTransformPointFromWorldToObjectSpace(hit_point_internal);
    glm::vec3 hit_point = glm::vec3(hit_point_internal.x, hit_point_internal.y, hit_point_internal.z);
    const auto normal = surfaceNormal(interpolator, optixGetCurveParameter(), hit_point);
    return normal;
  }

  // Compute normal
  __forceinline__ __device__ glm::vec3 ComputeNormal(OptixPrimitiveType type, const int primitive_index) const {
    switch (type) {
      case OPTIX_PRIMITIVE_TYPE_ROUND_LINEAR:
        return NormalLinear(primitive_index);
      case OPTIX_PRIMITIVE_TYPE_ROUND_QUADRATIC_BSPLINE:
        return NormalQuadratic(primitive_index);
      case OPTIX_PRIMITIVE_TYPE_ROUND_CUBIC_BSPLINE:
        return NormalCubic(primitive_index);
    }
    return glm::vec3(0.0f);
  }
};

struct TriangularMesh {
  evo_engine::Vertex *vertices = nullptr;
  glm::uvec3 *triangles = nullptr;

  __device__ HitInfo GetHitInfo() const {
    HitInfo hit_info;
    const auto triangle_barycentrics = optixGetTriangleBarycentrics();
    const auto primitive_id = optixGetPrimitiveIndex();
    const auto triangle_indices = triangles[primitive_id];
    const auto &vx = vertices[triangle_indices.x];
    const auto &vy = vertices[triangle_indices.y];
    const auto &vz = vertices[triangle_indices.z];
    hit_info.tex_coord = (1.f - triangle_barycentrics.x - triangle_barycentrics.y) * vx.tex_coord +
                         triangle_barycentrics.x * vy.tex_coord + triangle_barycentrics.y * vz.tex_coord;
    hit_info.position = (1.f - triangle_barycentrics.x - triangle_barycentrics.y) * vx.position +
                        triangle_barycentrics.x * vy.position + triangle_barycentrics.y * vz.position;
    hit_info.normal = (1.f - triangle_barycentrics.x - triangle_barycentrics.y) * vx.normal +
                      triangle_barycentrics.x * vy.normal + triangle_barycentrics.y * vz.normal;
    hit_info.tangent = (1.f - triangle_barycentrics.x - triangle_barycentrics.y) * vx.tangent +
                       triangle_barycentrics.x * vy.tangent + triangle_barycentrics.y * vz.tangent;

    auto z = 1.f - triangle_barycentrics.x - triangle_barycentrics.y;
    if (triangle_barycentrics.x > z && triangle_barycentrics.x > triangle_barycentrics.y) {
      hit_info.color = vy.color;
      hit_info.vertex_info1 = vy.vertex_info1;
      hit_info.vertex_info2 = vy.vertex_info2;
      hit_info.vertex_info3 = vy.vertex_info3;
      hit_info.vertex_info4 = vy.vertex_info4;
    } else if (triangle_barycentrics.y > z) {
      hit_info.color = vz.color;
      hit_info.vertex_info1 = vz.vertex_info1;
      hit_info.vertex_info2 = vz.vertex_info2;
      hit_info.vertex_info3 = vz.vertex_info3;
      hit_info.vertex_info4 = vz.vertex_info4;
    } else {
      hit_info.color = vx.color;
      hit_info.vertex_info1 = vx.vertex_info1;
      hit_info.vertex_info2 = vx.vertex_info2;
      hit_info.vertex_info3 = vx.vertex_info3;
      hit_info.vertex_info4 = vx.vertex_info4;
    }
    hit_info.triangle_index = primitive_id;
    return hit_info;
  }

  __device__ glm::uvec3 GetIndices(const int &primitive_id) const {
    return triangles[primitive_id];
  }

  __device__ glm::vec2 GetTexCoord(const float2 &triangle_barycentrics, const glm::uvec3 &triangle_indices) const {
    return (1.f - triangle_barycentrics.x - triangle_barycentrics.y) * vertices[triangle_indices.x].tex_coord +
           triangle_barycentrics.x * vertices[triangle_indices.y].tex_coord +
           triangle_barycentrics.y * vertices[triangle_indices.z].tex_coord;
  }

  __device__ glm::vec3 GetTransformedPosition(const glm::mat4 &global_transform, const float2 &triangle_barycentrics,
                                              const glm::uvec3 &triangle_indices) const {
    return global_transform *
           glm::vec4((1.f - triangle_barycentrics.x - triangle_barycentrics.y) * vertices[triangle_indices.x].position +
                         triangle_barycentrics.x * vertices[triangle_indices.y].position +
                         triangle_barycentrics.y * vertices[triangle_indices.z].position,
                     1.0f);
  }

  __device__ glm::vec3 GetPosition(const float2 &triangle_barycentrics, const glm::uvec3 &triangle_indices) const {
    return (1.f - triangle_barycentrics.x - triangle_barycentrics.y) * vertices[triangle_indices.x].position +
           triangle_barycentrics.x * vertices[triangle_indices.y].position +
           triangle_barycentrics.y * vertices[triangle_indices.z].position;
  }

  __device__ glm::vec3 GetColor(const float2 &triangle_barycentrics, const glm::uvec3 &triangle_indices) const {
    auto z = 1.f - triangle_barycentrics.x - triangle_barycentrics.y;
    if (triangle_barycentrics.x > z && triangle_barycentrics.x > triangle_barycentrics.y) {
      return vertices[triangle_indices.y].color;
    } else if (triangle_barycentrics.y > z) {
      return vertices[triangle_indices.z].color;
    }
    return vertices[triangle_indices.x].color;
  }

  __device__ glm::vec3 GetTransformedNormal(const glm::mat4 &global_transform, const float2 &triangle_barycentrics,
                                            const glm::uvec3 &triangle_indices) const {
    return global_transform *
           glm::vec4((1.f - triangle_barycentrics.x - triangle_barycentrics.y) * vertices[triangle_indices.x].normal +
                         triangle_barycentrics.x * vertices[triangle_indices.y].normal +
                         triangle_barycentrics.y * vertices[triangle_indices.z].normal,
                     0.0f);
  }

  __device__ glm::vec3 GetTransformedTangent(const glm::mat4 &globalTransform, const float2 &triangleBarycentrics,
                                             const glm::uvec3 &triangleIndices) const {
    return globalTransform *
           glm::vec4((1.f - triangleBarycentrics.x - triangleBarycentrics.y) * vertices[triangleIndices.x].tangent +
                         triangleBarycentrics.x * vertices[triangleIndices.y].tangent +
                         triangleBarycentrics.y * vertices[triangleIndices.z].tangent,
                     0.0f);
  }

  __device__ glm::vec3 GetNormal(const float2 &triangleBarycentrics, const glm::uvec3 &triangleIndices) const {
    return (1.f - triangleBarycentrics.x - triangleBarycentrics.y) * vertices[triangleIndices.x].normal +
           triangleBarycentrics.x * vertices[triangleIndices.y].normal +
           triangleBarycentrics.y * vertices[triangleIndices.z].normal;
  }

  __device__ glm::vec3 GetTangent(const float2 &triangleBarycentrics, const glm::uvec3 &triangleIndices) const {
    return (1.f - triangleBarycentrics.x - triangleBarycentrics.y) * vertices[triangleIndices.x].tangent +
           triangleBarycentrics.x * vertices[triangleIndices.y].tangent +
           triangleBarycentrics.y * vertices[triangleIndices.z].tangent;
  }
};

struct SurfaceMaterial {
  evo_engine::MaterialProperties material_properties;

  cudaTextureObject_t albedo_texture;
  cudaTextureObject_t normal_texture;
  cudaTextureObject_t metallic_texture;
  cudaTextureObject_t roughness_texture;

  __device__ glm::vec4 GetAlbedo(const glm::vec2 &tex_coord) const {
    if (!albedo_texture)
      return glm::vec4(material_properties.albedo_color, 1.0f - material_properties.transmission);
    float4 textureAlbedo = ::tex2D<float4>(albedo_texture, tex_coord.x, tex_coord.y);
    return glm::vec4(textureAlbedo.x, textureAlbedo.y, textureAlbedo.z, textureAlbedo.w);
  }

  __device__ float GetRoughness(const glm::vec2 &tex_coord) const {
    if (!roughness_texture)
      return material_properties.roughness;
    return ::tex2D<float4>(roughness_texture, tex_coord.x, tex_coord.y).x;
  }

  __device__ float GetMetallic(const glm::vec2 &tex_coord) const {
    if (!metallic_texture)
      return material_properties.metallic;
    return ::tex2D<float4>(metallic_texture, tex_coord.x, tex_coord.y).x;
  }

  __device__ void ApplyNormalTexture(glm::vec3 &normal, const glm::vec2 &tex_coord, const glm::vec3 &tangent) const {
    if (!normal_texture)
      return;
    float4 textureNormal = ::tex2D<float4>(normal_texture, tex_coord.x, tex_coord.y);
    glm::vec3 B = glm::cross(normal, tangent);
    glm::mat3 TBN = glm::mat3(tangent, B, normal);
    normal = glm::vec3(textureNormal.x, textureNormal.y, textureNormal.z) * 2.0f - glm::vec3(1.0f);
    normal = glm::normalize(TBN * normal);
  }

  __device__ float GetRadiusMax() const {
    return 0.5f;
  }
};

struct SurfaceCompressedBtf {
  BtfBase btf;
#pragma region Device functions

  __device__ void ComputeAngles(const glm::vec3 &direction, const glm::vec3 &normal, const glm::vec3 &tangent,
                                float &theta, float &phi) const {
    // transform view & illum vectors into local texture coordinates, i.e.
    // tangent space
    glm::vec3 transformed_dir;
    const glm::vec3 b = glm::cross(normal, tangent);
    transformed_dir[0] = glm::dot(tangent, direction);  // T[0]*view[0] + T[1]*view[1] + T[2]*view[2];
    transformed_dir[1] = glm::dot(b, direction);        // B[0]*view[0] + B[1]*view[1] + B[2]*view[2];
    transformed_dir[2] = glm::dot(normal, direction);   // N[0]*view[0] + N[1]*view[1] + N[2]*view[2];
    if (isnan(transformed_dir[0])) {
      theta = 0.f;
      phi = 0.f;
      return;
    }

    // assert(fabs(transformed_dir[2]) <= 1.01f);

    if (transformed_dir[2] < 0.0) {
      phi = 0.0;
      theta = 90.0;
      return;
    }

    theta = glm::degrees(acosf(transformed_dir[2]));

    phi = glm::degrees(atan2(transformed_dir[1], transformed_dir[0])) + 360.0f;

    if (phi > 360.f)
      phi -= 360.f;
  }

  __device__ void GetValue(const glm::vec2 &tex_coord, const glm::vec3 &view_dir, const glm::vec3 &illumination_dir,
                           const glm::vec3 &normal, const glm::vec3 tangent, glm::vec3 &out) const {
    out = glm::vec3(1.0f);
    float illumination_theta, illumination_phi, view_theta, view_phi;
    ComputeAngles(-view_dir, normal, tangent, view_theta, view_phi);
    ComputeAngles(illumination_dir, normal, tangent, illumination_theta, illumination_phi);
    btf.GetValueDeg(tex_coord, illumination_theta, illumination_phi, view_theta, view_phi, out);
    out /= 256.0f;
  }

#pragma endregion
};

struct SBT {
  unsigned long long handle;
  glm::mat4 global_transform;
  RendererType geometry_type;
  void *geometry;
  MaterialType material_type;
  void *material;

  __device__ HitInfo GetHitInfo(glm::vec3 &ray_direction, bool check_normal = true) const {
    HitInfo ret_val;
    if (geometry_type != RendererType::Curve) {
      auto *mesh = (TriangularMesh *)geometry;
      ret_val = mesh->GetHitInfo();

    } else {
      auto *curves = (Curves *)geometry;
      ret_val = curves->GetHitInfo();
    }
    ret_val.normal = glm::normalize(global_transform * glm::vec4(ret_val.normal, 0.0f));
    if (check_normal && glm::dot(ray_direction, ret_val.normal) > 0.0f) {
      ret_val.normal = -ret_val.normal;
    }
    ret_val.tangent = glm::normalize(global_transform * glm::vec4(ret_val.tangent, 0.0f));
    ret_val.position = global_transform * glm::vec4(ret_val.position, 1.0f);
    return ret_val;
  }
};

/*! SBT record for a raygen program */
struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) CameraRenderingRayGenRecord {
  __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  void *data;
};

/*! SBT record for a miss program */
struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) CameraRenderingRayMissRecord {
  __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  void *data;
};

/*! SBT record for a hitgroup program */
struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) CameraRenderingRayHitRecord {
  __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  SBT data;
};

/*! SBT record for a raygen program */
struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) IlluminationEstimationRayGenRecord {
  __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  void *data;
};

/*! SBT record for a miss program */
struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) IlluminationEstimationRayMissRecord {
  __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  void *data;
};

/*! SBT record for a hitgroup program */
struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) IlluminationEstimationRayHitRecord {
  __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  SBT data;
};

/*! SBT record for a raygen program */
struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) CameraSpectralRayGenRecord {
  __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  void *data;
};

/*! SBT record for a miss program */
struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) CameraSpectralRayMissRecord {
  __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  void *data;
};

/*! SBT record for a hitgroup program */
struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) CameraSpectralRayHitRecord {
  __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  SBT data;
};

/*! SBT record for a raygen program */
struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) IlluminationEstimationSpectralRayGenRecord {
  __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  void *data;
};

/*! SBT record for a miss program */
struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) IlluminationEstimationSpectralRayMissRecord {
  __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  void *data;
};

/*! SBT record for a hitgroup program */
struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) IlluminationEstimationSpectralRayHitRecord {
  __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  SBT data;
};
/*! SBT record for a raygen program */
struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) PointCloudScanningRayGenRecord {
  __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  void *data;
};

/*! SBT record for a miss program */
struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) PointCloudScanningRayMissRecord {
  __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  void *data;
};

/*! SBT record for a hitgroup program */
struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) PointCloudScanningRayHitRecord {
  __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  SBT data;
};
}  // namespace evo_engine
