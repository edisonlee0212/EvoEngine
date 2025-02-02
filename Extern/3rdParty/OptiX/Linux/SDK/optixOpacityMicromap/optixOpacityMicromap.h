/*

 * SPDX-FileCopyrightText: Copyright (c) 2022 - 2024  NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#include <sutil/vec_math.h>

constexpr float CIRCLE_RADIUS = 0.75f;

enum RayType
{
    RAY_TYPE_RADIANCE = 0,
    RAY_TYPE_COUNT
};

//-----------------------------------------------------------------------------
//
// Helper functions to be used when pre-baking opacity into OMM and when 
// evaluating opacity within anyhit function
//
//-----------------------------------------------------------------------------

static __host__ __device__ __inline__ float2 computeUV( float2 bary, float2 uv0, float2 uv1, float2 uv2 )
{
    return ( 1.0f - bary.x - bary.y )*uv0 + bary.x*uv1 + bary.y*uv2;
}

static __host__ __device__ __inline__ bool inCircle( const float2 uv )
{
    return ( uv.x * uv.x + uv.y * uv.y ) < ( CIRCLE_RADIUS * CIRCLE_RADIUS );
};


//-----------------------------------------------------------------------------
//
// Types
//
//-----------------------------------------------------------------------------
struct Params
{
    uchar4*                image;
    unsigned int           image_width;
    unsigned int           image_height;
    float3                 cam_eye;
    float3                 cam_u, cam_v, cam_w;
    OptixTraversableHandle handle;
};


struct RayGenData
{
    // No data needed
};


struct MissData
{
    float3 bg_color;
};


struct HitGroupData
{
    float2* uvs;
};
