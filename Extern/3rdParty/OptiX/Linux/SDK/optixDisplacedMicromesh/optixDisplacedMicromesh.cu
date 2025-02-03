/*

 * SPDX-FileCopyrightText: Copyright (c) 2023 - 2024  NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <optix.h>
#include <optix_micromap.h>

#include "optixDisplacedMicromesh.h"
#include "random.h"

#include <sutil/vec_math.h>
#include <cuda/helpers.h>


extern "C" {
    __constant__ Globals params;
}


struct Onb
{
    __forceinline__ __device__ Onb( const float3& normal )
    {
        m_normal = normal;

        if( fabs( m_normal.x ) > fabs( m_normal.z ) )
        {
            m_binormal.x = -m_normal.y;
            m_binormal.y = m_normal.x;
            m_binormal.z = 0;
        }
        else
        {
            m_binormal.x = 0;
            m_binormal.y = -m_normal.z;
            m_binormal.z = m_normal.y;
        }

        m_binormal = normalize( m_binormal );
        m_tangent  = cross( m_binormal, m_normal );
    }

    __forceinline__ __device__ void inverse_transform( float3& p ) const
    {
        p = p.x * m_tangent + p.y * m_binormal + p.z * m_normal;
    }

    float3 m_tangent;
    float3 m_binormal;
    float3 m_normal;
};

static __forceinline__ __device__ void cosine_sample_hemisphere( const float u1, const float u2, float3& p )
{
    // Uniformly sample disk.
    const float r   = sqrtf( u1 );
    const float phi = 2.0f * M_PIf * u2;
    p.x             = r * cosf( phi );
    p.y             = r * sinf( phi );

    // Project up to hemisphere.
    p.z = sqrtf( fmaxf( 0.0f, 1.0f - p.x * p.x - p.y * p.y ) );
}


// Use named types for compatibility with nvrtc
// Otherwise these structs can be defined as unnamed structs directly in 'Payload'
// to avoid access via p0123.px and directly access px.
struct t_p0123 {
    unsigned int p0, p1, p2, p3;
};
struct t_cseed {
    float3 color;
    unsigned int seed;
};

struct Payload {

    union {
        t_p0123 p0123;
        t_cseed cseed;
    };

    __forceinline__ __device__ void setAll()
    {
        optixSetPayload_0( p0123.p0 );
        optixSetPayload_1( p0123.p1 );
        optixSetPayload_2( p0123.p2 );
        optixSetPayload_3( p0123.p3 );
    }
    __forceinline__ __device__ void getAll()
    {
        p0123.p0 = optixGetPayload_0();
        p0123.p1 = optixGetPayload_1();
        p0123.p2 = optixGetPayload_2();
        p0123.p3 = optixGetPayload_3();
    }
    __forceinline__ __device__ void setC()
    {
        optixSetPayload_0( p0123.p0 );
        optixSetPayload_1( p0123.p1 );
        optixSetPayload_2( p0123.p2 );
    }
    __forceinline__ __device__ void getC()
    {
        p0123.p0 = optixGetPayload_0();
        p0123.p1 = optixGetPayload_1();
        p0123.p2 = optixGetPayload_2();
    }
    __forceinline__ __device__ void getSeed()
    {
        p0123.p3 = optixGetPayload_3();
    }
};

constexpr unsigned int SBT_STRIDE_COLLAPSE = 0;

extern "C" __global__ void __raygen__rg()
{
    const uint3 idx = optixGetLaunchIndex();
    const uint3 dim = optixGetLaunchDimensions();

    const float3 eye = params.eye;
    const float3 U = params.U;
    const float3 V = params.V;
    const float3 W = params.W;

    Payload payload;
    payload.cseed.seed = tea<4>( idx.y * dim.x + idx.x, 12346789 + params.subframe_index );

    float3 final_c = make_float3( 0 );
#pragma unroll 1
    for( int x = 1; x <= params.spp; ++x )
    {
        const float2 d = 2.0f * make_float2(
            ( static_cast< float >( idx.x ) + rnd( payload.cseed.seed ) ) / static_cast< float >( dim.x ),
            ( static_cast< float >( idx.y ) + rnd( payload.cseed.seed ) ) / static_cast< float >( dim.y )
        ) - 1.0f;
        float3 direction = normalize( d.x * U + d.y * V + W );

        float time = 0;

        payload.cseed.color = make_float3( 0.5f );
        optixTrace( params.handle, eye, direction, 0, 1e16f, time, OptixVisibilityMask( 1 ),
                    OPTIX_RAY_FLAG_NONE,
                    0,                    // SBT offset, first ray type (only one here)
                    SBT_STRIDE_COLLAPSE,  // SBT stride, forcing a single HitGroup in combination with an sbt offset set to zero for every instance!
                    0,                    // missSBTIndex, used for camera rays
                    payload.p0123.p0, payload.p0123.p1, payload.p0123.p2, payload.p0123.p3 );
        final_c += payload.cseed.color;
    }
    final_c /= params.spp;

    if( !params.accum_buffer || !params.frame_buffer )
        return;

    unsigned int image_index = idx.y * params.width + idx.x;
    if( params.subframe_index > 0 )
    {
        const float  a                = 1.0f / static_cast<float>( params.subframe_index + 1 );
        const float3 accum_color_prev = make_float3( params.accum_buffer[image_index] );
        final_c                       = lerp( accum_color_prev, final_c, a );
    }
    params.accum_buffer[image_index] = make_float4( final_c, 1.0f );
    params.frame_buffer[image_index] = make_color( final_c );
}


extern "C" __global__ void __miss__ms()
{
    MissData* rt_data = reinterpret_cast< MissData* >( optixGetSbtDataPointer() );
    Payload p;
    p.cseed.color = make_float3( rt_data->bg_color.x, rt_data->bg_color.y, rt_data->bg_color.z );
    p.setC();
}

extern "C" __global__ void __miss__occlusion()
{
    optixSetPayload_0( 0 );
}

static __forceinline__ __device__ void ch_impl( const float3& normal, const float3& hitpoint )
{
    Payload p;
    p.getSeed();

    const float offset = 0.0001f;
    float shade = 1.0f;
    if( params.ao )
    {
        const float z1 = rnd( p.cseed.seed );
        const float z2 = rnd( p.cseed.seed );

        float3 w_in;
        cosine_sample_hemisphere( z1, z2, w_in );
        float3 wn = normalize( optixTransformNormalFromObjectToWorldSpace( normal ) );
        wn        = faceforward( wn, -optixGetWorldRayDirection(), wn );
        Onb onb( wn );
        onb.inverse_transform( w_in );

        unsigned int occluded = 1;
        optixTrace( params.handle, hitpoint + wn * offset, w_in, 0.0f, 1e16f, optixGetRayTime(), OptixVisibilityMask( 1 ),
                    OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT | OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT,
                    0, SBT_STRIDE_COLLAPSE,  // no hit group will even be executed (assuming no IS), hence, set stride and offset to 0
                    1,     // select MS program (__miss__occlusion)
                    occluded );  // this is inout here! If MS is called, it will override the payload

        if( occluded )
            shade = 0.f;
    }

    HitGroupData* rt_data = reinterpret_cast<HitGroupData*>( optixGetSbtDataPointer() );
    // convert normal to color and store in payload
    p.cseed.color = shade * ( 0.5f * normal + make_float3( 0.5f ) ) * rt_data->color;

    p.setAll();
}

extern "C" __global__ void __closesthit__ch_tri()
{
    // fetch current triangle vertices
    float3 vertices[3];
    float3 hitP;

    if( optixIsTriangleHit() )
    {
        optixGetTriangleVertexData( optixGetGASTraversableHandle(), optixGetPrimitiveIndex(), optixGetSbtGASIndex(),
                                    optixGetRayTime(), vertices );

        float2 barycentrics = optixGetTriangleBarycentrics();

        //////////////////////////////////////////////////////////////////////////
        // wire frame rendering for triangles
        if( barycentrics.x < .01f || barycentrics.y < .01f || 1 - barycentrics.x - barycentrics.y < .01f )
        {
            Payload p = {};
            p.cseed.color = make_float3( 1.f );
            p.setAll();
            return;
        }
        //////////////////////////////////////////////////////////////////////////

        hitP = ( 1 - barycentrics.x - barycentrics.y ) * vertices[0] + barycentrics.x * vertices[1] + barycentrics.y * vertices[2];
    }
    else if( optixIsDisplacedMicromeshTriangleHit() )
    {
        // returns the vertices of the current DMM micro triangle hit
        optixGetMicroTriangleVertexData( vertices );

        float2 hitBaseBarycentrics = optixGetTriangleBarycentrics();

        float2 microVertexBaseBarycentrics[3];
        optixGetMicroTriangleBarycentricsData( microVertexBaseBarycentrics );

        float2 microBarycentrics = optixBaseBarycentricsToMicroBarycentrics( hitBaseBarycentrics, microVertexBaseBarycentrics );

        //////////////////////////////////////////////////////////////////////////
        // wire frame rendering for micro triangles
        if( microBarycentrics.x < .01f || microBarycentrics.y < .01f || 1 - microBarycentrics.x - microBarycentrics.y < .01f )
        {
            Payload p ={};
            p.cseed.color = make_float3( 1.f );
            p.setAll();
            return;
        }
        //////////////////////////////////////////////////////////////////////////

        hitP = (1 - microBarycentrics.x - microBarycentrics.y) * vertices[0] + microBarycentrics.x * vertices[1] + microBarycentrics.y * vertices[2];
    }

    // compute triangle normal
    vertices[1] -= vertices[0];
    vertices[2] -= vertices[0];
    float3 normal = make_float3(
        vertices[1].y*vertices[2].z - vertices[1].z*vertices[2].y,
        vertices[1].z*vertices[2].x - vertices[1].x*vertices[2].z,
        vertices[1].x*vertices[2].y - vertices[1].y*vertices[2].x );

    ch_impl( normalize( normal ), hitP );
}
