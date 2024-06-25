//
// Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#pragma once

#include "CudaMath.hpp"
#include "Vertex.hpp"
#include "glm/glm.hpp"
//
// First order polynomial interpolator
//

__device__ __forceinline__ evo_engine::StrandPoint operator/(const evo_engine::StrandPoint &lhs, const float &rhs) {
    evo_engine::StrandPoint ret_val;
    ret_val.thickness = lhs.thickness / rhs;
    ret_val.position = lhs.position / rhs;
    ret_val.color = lhs.color / rhs;
    ret_val.tex_coord = lhs.tex_coord / rhs;
    return ret_val;
}

__device__ __forceinline__ evo_engine::StrandPoint operator*(const evo_engine::StrandPoint &lhs, const float &rhs) {
    evo_engine::StrandPoint ret_val;
    ret_val.thickness = lhs.thickness * rhs;
    ret_val.position = lhs.position * rhs;
    ret_val.color = lhs.color * rhs;
    ret_val.tex_coord = lhs.tex_coord * rhs;
    return ret_val;
}

__device__ __forceinline__ evo_engine::StrandPoint
operator+(const evo_engine::StrandPoint &lhs, const evo_engine::StrandPoint &rhs) {
    evo_engine::StrandPoint ret_val;
    ret_val.thickness = lhs.thickness + rhs.thickness;
    ret_val.position = lhs.position + rhs.position;
    ret_val.color = lhs.color + rhs.color;
    ret_val.tex_coord = lhs.tex_coord + rhs.tex_coord;
    return ret_val;
}

__device__ __forceinline__ evo_engine::StrandPoint
operator-(const evo_engine::StrandPoint &lhs, const evo_engine::StrandPoint &rhs) {
    evo_engine::StrandPoint ret_val;
    ret_val.thickness = lhs.thickness - rhs.thickness;
    ret_val.position = lhs.position - rhs.position;
    ret_val.color = lhs.color - rhs.color;
    ret_val.tex_coord = lhs.tex_coord - rhs.tex_coord;
    return ret_val;
}

struct LinearBSplineSegment {
    __device__ __forceinline__ LinearBSplineSegment() {}

    __device__ __forceinline__ LinearBSplineSegment(const evo_engine::StrandPoint *q) {
        p[0] = q[0];
        p[1] = q[1] - q[0];  // pre-transform p[] for fast evaluation
    }

    __device__ __forceinline__ glm::vec4 color(const float &u) const { return p[0].color + p[1].color * u; }

    __device__ __forceinline__ glm::vec2 texCoord(const float &u) const { return glm::vec2(p[0].tex_coord + p[1].tex_coord * u, 0.0f); }

    __device__ __forceinline__ float radius(const float &u) const { return p[0].thickness + p[1].thickness * u; }

    __device__ __forceinline__ glm::vec3 position(float u) const { return p[0].position + u * p[1].position; }

    __device__ __forceinline__ float min_radius(float u1, float u2) const {
        return fminf(radius(u1), radius(u2));
    }

    __device__ __forceinline__ float max_radius(float u1, float u2) const {
        if (!p[1].thickness)
            return p[0].thickness;  // a quick bypass for constant width
        return fmaxf(radius(u1), radius(u2));
    }

    __device__ __forceinline__ glm::vec3 velocity(float u) const { return p[1].position; }

    __device__ __forceinline__ float velocity_radius(float u) const { return p[1].thickness; }

    __device__ __forceinline__ glm::vec3 acceleration(float u) const { return glm::vec3(0.0f); }

    __device__ __forceinline__ float acceleration_radius(float u) const { return 0.0f; }

    __device__ __forceinline__ float derivative_of_radius(float u) const { return p[1].thickness; }

    evo_engine::StrandPoint p[2];  // pre-transformed "control points" for fast evaluation
};

//
// Second order polynomial interpolator
//
struct QuadraticBSplineSegment {
    __device__ __forceinline__ QuadraticBSplineSegment() {}

    __device__ __forceinline__ QuadraticBSplineSegment(const evo_engine::StrandPoint *q) {
        // pre-transform control-points for fast evaluation
        p[0] = q[1] / 2.0f + q[0] / 2.0f;
        p[1] = q[1] - q[0];
        p[2] = q[0] / 2.0f - q[1] + q[2] / 2.0f;
        p[0] = q[1] / 2.0f + q[0] / 2.0f;
        p[1] = q[1] - q[0];
        p[2] = q[0] / 2.0f - q[1] + q[2] / 2.0f;
    }

    __device__ __forceinline__ glm::vec4 color(const float &u) const {
        return p[0].color + u * p[1].color + u * u * p[2].color; }

    __device__ __forceinline__ glm::vec2 texCoord(const float& u) const {
        return glm::vec2(p[0].tex_coord + u * p[1].tex_coord + u * u * p[2].tex_coord, 0.0f);
    }

    __device__ __forceinline__ glm::vec3 position(float u) const {
        return p[0].position + u * p[1].position + u * u * p[2].position;
    }

    __device__ __forceinline__ float radius(float u) const {
        return p[0].thickness + u * (p[1].thickness + u * p[2].thickness);
    }

    __device__ __forceinline__ float min_radius(float u1, float u2) const {
        float root1 = clamp(-0.5f * p[1].thickness / p[2].thickness, u1, u2);
        return fminf(fminf(radius(u1), radius(u2)), radius(root1));
    }

    __device__ __forceinline__ float max_radius(float u1, float u2) const {
        if (!p[1].thickness && !p[2].thickness)
            return p[0].thickness;  // a quick bypass for constant width
        float root1 = clamp(-0.5f * p[1].thickness / p[2].thickness, u1, u2);
        return fmaxf(fmaxf(radius(u1), radius(u2)), radius(root1));
    }

    __device__ __forceinline__ glm::vec3 velocity(float u) const { return p[1].position + 2 * u * p[2].position; }

    __device__ __forceinline__ float velocity_radius(float u) const {
        return p[1].thickness + 2 * u * p[2].thickness;
    }

    __device__ __forceinline__ glm::vec3 acceleration(float u) const { return 2.0f * p[2].position; }

    __device__ __forceinline__ float acceleration_radius(float u) const { return 2.0f * p[2].thickness; }

    __device__ __forceinline__ float derivative_of_radius(float u) const {
        return p[1].thickness + 2 * u * p[2].thickness;
    }

    evo_engine::StrandPoint p[3];  // pre-transformed "control points" for fast evaluation
};

//
// Third order polynomial interpolator
//
struct CubicBSplineSegment {
    __device__ __forceinline__ CubicBSplineSegment() {}

    __device__ __forceinline__ CubicBSplineSegment(const evo_engine::StrandPoint *q) {
        // pre-transform control points for fast evaluation
        p[0] = (q[2] + q[0]) / 6 + q[1] * (4.0f / 6.0f);
        p[1] = q[2] - q[0];
        p[2] = q[2] - q[1];
        p[3] = q[3] - q[1];
    }

    __device__ __forceinline__ static glm::vec3 terms(float u) {
        float uu = u * u;
        float u3 = (1.0f / 6.0f) * uu * u;
        return {u3 + 0.5f * (u - uu), uu - 4 * u3, u3};
    }

    __device__ __forceinline__ glm::vec4 color(float u) const {
        glm::vec3 q = terms(u);
        return p[0].color + q.x * p[1].color + q.y * p[2].color + q.z * p[3].color;
    }

    __device__ __forceinline__ glm::vec2 texCoord(float u) const {
        glm::vec3 q = terms(u);
        return glm::vec2(p[0].tex_coord + q.x * p[1].tex_coord + q.y * p[2].tex_coord + q.z * p[3].tex_coord, 0.0f);
    }

    __device__ __forceinline__ glm::vec3 position(float u) const {
        glm::vec3 q = terms(u);
        return p[0].position + q.x * p[1].position + q.y * p[2].position + q.z * p[3].position;
    }

    __device__ __forceinline__ float radius(float u) const {
        return p[0].thickness +
               u * (p[1].thickness / 2.0f +
                    u * ((p[2].thickness - p[1].thickness / 2.0f) +
                         u * (p[1].thickness - 4.0f * p[2].thickness + p[3].thickness) / 6.0f));
    }

    __device__ __forceinline__ float min_radius(float u1, float u2) const {
        // a + 2 b u - c u^2
        float a = p[1].thickness;
        float b = 2.0f * p[2].thickness - p[1].thickness;
        float c = 4.0f * p[2].thickness - p[1].thickness - p[3].thickness;
        float rmin = fminf(radius(u1), radius(u2));
        if (fabsf(c) < 1e-5f) {
            float root1 = clamp(-0.5f * a / b, u1, u2);
            return fminf(rmin, radius(root1));
        } else {
            float det = b * b + a * c;
            det = det <= 0.0f ? 0.0f : sqrt(det);
            float root1 = clamp((b + det) / c, u1, u2);
            float root2 = clamp((b - det) / c, u1, u2);
            return fminf(rmin, fminf(radius(root1), radius(root2)));
        }
    }

    __device__ __forceinline__ float max_radius(float u1, float u2) const {
        if (!p[1].thickness && !p[2].thickness && !p[3].thickness)
            return p[0].thickness;  // a quick bypass for constant width
        // a + 2 b u - c u^2
        float a = p[1].thickness;
        float b = 2 * p[2].thickness - p[1].thickness;
        float c = 4 * p[2].thickness - p[1].thickness - p[3].thickness;
        float rmax = fmaxf(radius(u1), radius(u2));
        if (fabsf(c) < 1e-5f) {
            float root1 = clamp(-0.5f * a / b, u1, u2);
            return fmaxf(rmax, radius(root1));
        } else {
            float det = b * b + a * c;
            det = det <= 0.0f ? 0.0f : sqrt(det);
            float root1 = clamp((b + det) / c, u1, u2);
            float root2 = clamp((b - det) / c, u1, u2);
            return fmaxf(rmax, fmaxf(radius(root1), radius(root2)));
        }
    }

    __device__ __forceinline__ glm::vec3 velocity(float u) const {
        // adjust u to avoid problems with tripple knots.
        if (u == 0)
            u = 0.000001f;
        if (u == 1)
            u = 0.999999f;
        float v = 1 - u;
        return 0.5f * v * v * p[1].position + 2.0f * v * u * p[2].position + 0.5f * u * u * p[3].position;
    }

    __device__ __forceinline__ float velocity_radius(float u) const {
        // adjust u to avoid problems with tripple knots.
        if (u == 0)
            u = 0.000001f;
        if (u == 1)
            u = 0.999999f;
        float v = 1.0f - u;
        return 0.5f * v * v * p[1].thickness + 2.0f * v * u * p[2].thickness + 0.5f * u * u * p[3].thickness;
    }

    __device__ __forceinline__ glm::vec3 acceleration(float u) const {
        return 2.0f * p[2].position - p[1].position +
               (p[1].position - 4.0f * p[2].position + p[3].position) * u;
    }

    __device__ __forceinline__ float acceleration_radius(float u) const {
        return 2.0f * p[2].thickness - p[1].thickness +
               (p[1].thickness - 4.0f * p[2].thickness + p[3].thickness) * u;
    }

    __device__ __forceinline__ float derivative_of_radius(float u) const {
        float v = 1.0f - u;
        return 0.5f * v * v * p[1].thickness + 2.0f * v * u * p[2].thickness + 0.5f * u * u * p[3].thickness;
    }

    evo_engine::StrandPoint p[4];  // pre-transformed "control points" for fast evaluation
};

// Compute curve primitive surface normal in object space.
//
// Template parameters:
//   CurveType - A B-Spline evaluator class.
//   type - 0     ~ cylindrical approximation (correct if radius' == 0)
//          1     ~ conic       approximation (correct if curve'' == 0)
//          other ~ the bona fide surface normal
//
// Parameters:
//   bc - A B-Spline evaluator object.
//   u  - segment parameter of hit-point.
//   ps - hit-point on curve's surface in object space; usually
//        computed like this.
//        glm::vec3 ps = ray_orig + t_hit * ray_dir;
//        the resulting point is slightly offset away from the
//        surface. For this reason (Warning!) ps gets modified by this
//        method, projecting it onto the surface
//        in case it is not already on it. (See also inline
//        comments.)
//
template<typename CurveType, int type = 2>
__device__ __forceinline__ glm::vec3 surfaceNormal(const CurveType &bc, float u, glm::vec3 &ps) {
    glm::vec3 normal;
    if (u == 0.0f) {
        normal = -bc.velocity(0.0f);  // special handling for flat endcaps
    } else if (u == 1.0f) {
        normal = bc.velocity(1.0f);   // special handling for flat endcaps
    } else {
        // ps is a point that is near the curve's offset surface,
        // usually ray.origin + ray.direction * rayt.
        // We will push it exactly to the surface by projecting it to the plane(p,d).
        // The function derivation:
        // we (implicitly) transform the curve into coordinate system
        // {p, o1 = normalize(ps - p), o2 = normalize(curve'(t)), o3 = o1 x o2} in which
        // curve'(t) = (0, length(d), 0); ps = (r, 0, 0);
        glm::vec3 p = bc.position(u);
        float r = bc.radius(u);  // == length(ps - p) if ps is already on the surface
        glm::vec3 d = bc.velocity(u);
        float dr = bc.velocity_radius(u);
        float dd = dot(d, d);

        glm::vec3 o1 = ps - p;               // dot(modified_o1, d) == 0 by design:
        o1 -= (dot(o1, d) / dd) * d;  // first, project ps to the plane(p,d)
        o1 *= r / length(o1);           // and then drop it to the surface
        ps = p + o1;                      // fine-tuning the hit point
        if (type == 0) {
            normal = o1;  // cylindrical approximation
        } else {
            if (type != 1) {
                dd -= dot(bc.acceleration(u), o1);
            }
            normal = dd * o1 - (dr * r) * d;
        }
    }
    return glm::normalize(normal);
}

template<int type = 1>
__device__ __forceinline__ glm::vec3 surfaceNormal(const LinearBSplineSegment &bc, float u, glm::vec3 &ps) {
    glm::vec3 normal;
    if (u == 0.0f) {
        normal = ps - bc.p[0].position;  // special handling for round endcaps
    } else if (u >= 1.0f) {
        // reconstruct second control point (Note: the interpolator pre-transforms
        // the control-points to speed up repeated evaluation.
        const glm::vec3 p1 = bc.p[1].position + bc.p[0].position;
        normal = ps - p1;  // special handling for round endcaps
    } else {
        // ps is a point that is near the curve's offset surface,
        // usually ray.origin + ray.direction * rayt.
        // We will push it exactly to the surface by projecting it to the plane(p,d).
        // The function derivation:
        // we (implicitly) transform the curve into coordinate system
        // {p, o1 = normalize(ps - p), o2 = normalize(curve'(t)), o3 = o1 x o2} in which
        // curve'(t) = (0, length(d), 0); ps = (r, 0, 0);
        glm::vec3 p = bc.position(u);
        float r = bc.radius(u);  // == length(ps - p) if ps is already on the surface
        glm::vec3 d = bc.velocity(u);
        float dr = bc.velocity_radius(u);
        float dd = dot(d, d);

        glm::vec3 o1 = ps - p;               // dot(modified_o1, d) == 0 by design:
        o1 -= (dot(o1, d) / dd) * d;  // first, project ps to the plane(p,d)
        o1 *= r / length(o1);           // and then drop it to the surface
        ps = p + o1;                      // fine-tuning the hit point
        if (type == 0) {
            normal = o1;  // cylindrical approximation
        } else {
            normal = dd * o1 - (dr * r) * d;
        }
    }
    return glm::normalize(normal);
}

// Compute curve primitive tangent in object space.
//
// Template parameters:
//   CurveType - A B-Spline evaluator class.
//
// Parameters:
//   bc - A B-Spline evaluator object.
//   u  - segment parameter of tangent location on curve.
//
template<typename CurveType>
__device__ __forceinline__ glm::vec3 curveTangent(const CurveType &bc, float u) {
    glm::vec3 tangent = bc.velocity3(u);
    return normalize(tangent);
}
