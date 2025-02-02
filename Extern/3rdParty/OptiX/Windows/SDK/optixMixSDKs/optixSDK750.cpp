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

#include <cuda_runtime.h>

// The OPTIX_ENABLE_SDK_MIXING macro was introduced with optix SDK 8.1.0
// Therefore, mixing multiple 8.0.0 or older SDKs is not supported.
// However, a single 8.0.0 or older SDK may be mixed with SDK 8.1.0 or newer.
#include "OptiX SDK 7.5.0/include/optix.h"
#include "OptiX SDK 7.5.0/include/optix_function_table_definition.h"
#include "OptiX SDK 7.5.0/include/optix_stubs.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>


#define OPTIX_CHECK( call )                                                                                            \
    {                                                                                                                  \
        OptixResult res = call;                                                                                        \
        if( res != OPTIX_SUCCESS )                                                                                     \
        {                                                                                                              \
            fprintf( stderr, "Optix call (%s) failed with code %d\n", #call, res );                                    \
            exit( 2 );                                                                                                 \
        }                                                                                                              \
    }

#define CUDA_CHECK( call )                                                                                             \
    {                                                                                                                  \
        cudaError_t error = call;                                                                                      \
        if( error != cudaSuccess )                                                                                     \
        {                                                                                                              \
            fprintf( stderr, "CUDA call (%s) failed with code %d: %s\n", #call, error, cudaGetErrorString( error ) );  \
            exit( 2 );                                                                                                 \
        }                                                                                                              \
    }

#define CUDA_SYNC_CHECK()                                                                                              \
    {                                                                                                                  \
        cudaDeviceSynchronize();                                                                                       \
        cudaError_t error = cudaGetLastError();                                                                        \
        if( error != cudaSuccess )                                                                                     \
        {                                                                                                              \
            fprintf( stderr, "error (%s: line %d): %s\n", __FILE__, __LINE__, cudaGetErrorString( error ) );           \
            exit( 2 );                                                                                                 \
        }                                                                                                              \
    }

static void context_log_cb( unsigned int level, const char* tag, const char* message, void* /*cbdata */ )
{
    std::cerr << "[" << std::setw( 2 ) << level << "][" << std::setw( 12 ) << tag << "]: "
        << message << "\n";
}

struct Vertex
{
    float x, y, z, pad;
};
struct IndexedTriangle
{
    unsigned int v1, v2, v3, pad;
};
struct Instance
{
    float transform[12];
};

// We specifically made one triangle back face and one front faced (back is the left triangle)
static std::vector<Vertex> g_triangleSoup = {
    {-0.5f, 1.0f, 0.0f, 999.9f}, {0.0f, -1.0f, 0.f, 999.9f},  {-1.0f, -1.0f, 0.f, 999.9f}, {0.5f, 1.0f, 0.0f, 999.9f},
    {0.0f, -1.0f, 0.f, 999.9f},  {1.0f, -1.0f, 0.f, 999.9f},  {0.1f, 0.0f, 0.0f, 999.9f},  {0.0f, 1.0f, 0.f, 999.9f},
    {0.5f, 1.0f, 0.f, 999.9f},   {-0.1f, 0.0f, 0.0f, 999.9f}, {0.0f, 1.0f, 0.f, 999.9f},   {-0.5f, 1.0f, 0.f, 999.9f},

};

static std::vector<IndexedTriangle> g_indexedTriangleMesh = {
    // Note the last value isn't used, so put a big value there
    {0, 1, 2, 999},
    {3, 1, 5, 999},  // 4 is the same as 1, so reuse it to test
    {6, 7, 3, 999},  // 8 -> 3
    {9, 7, 0, 999}   // 10 -> 7, 11 -> 0
};

static std::vector<Instance> g_instances = {
    {{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0}},
    {{1, 0, 0, 2, 0, 1, 0, 0, 0, 0, 1, 0}},
};

void runSDK750()
{
    OPTIX_CHECK( optixInit() );

    // Initialize CUDA
    CUDA_CHECK( cudaFree( 0 ) );

    OptixDeviceContext context;
    CUcontext          cuCtx = 0;  // zero means take the current context
    OptixDeviceContextOptions options = {};
    options.logCallbackFunction = &context_log_cb;
    options.logCallbackLevel = 4;
    OPTIX_CHECK( optixDeviceContextCreate( cuCtx, &options, &context ) );

    // upload geometry

    CUdeviceptr d_vertices, d_indices;
    size_t      verticesSizeInBytes = g_triangleSoup.size() * sizeof( Vertex );
    size_t      indicesSizeInBytes = g_indexedTriangleMesh.size() * sizeof( IndexedTriangle );
    CUDA_CHECK( cudaMalloc( ( void** )&d_vertices, verticesSizeInBytes ) );
    CUDA_CHECK( cudaMalloc( ( void** )&d_indices, indicesSizeInBytes ) );
    CUDA_CHECK( cudaMemcpy( ( void* )d_vertices, g_triangleSoup.data(), verticesSizeInBytes, cudaMemcpyHostToDevice ) );
    CUDA_CHECK( cudaMemcpy( ( void* )d_indices, g_indexedTriangleMesh.data(), indicesSizeInBytes, cudaMemcpyHostToDevice ) );

    // build ias and gas

    OptixBuildInput triangleInput = {};

    triangleInput.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
    triangleInput.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
    triangleInput.triangleArray.vertexStrideInBytes = sizeof( Vertex );
    triangleInput.triangleArray.numVertices = ( unsigned int )g_triangleSoup.size();
    triangleInput.triangleArray.vertexBuffers = &d_vertices;

    triangleInput.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
    triangleInput.triangleArray.indexStrideInBytes = sizeof( IndexedTriangle );
    ;
    triangleInput.triangleArray.numIndexTriplets = ( unsigned int )g_indexedTriangleMesh.size();
    triangleInput.triangleArray.indexBuffer = d_indices;

    unsigned int triangleInputFlags[1] = { OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT };
    triangleInput.triangleArray.flags = triangleInputFlags;
    triangleInput.triangleArray.numSbtRecords = 1;

    CUdeviceptr d_instances;
    size_t      instancesSizeInBytes = g_instances.size() * sizeof( OptixInstance );
    CUDA_CHECK( cudaMalloc( ( void** )&d_instances, instancesSizeInBytes ) );

    OptixBuildInput instanceInput = {};

    instanceInput.type = OPTIX_BUILD_INPUT_TYPE_INSTANCES;
    instanceInput.instanceArray.instances = d_instances;
    instanceInput.instanceArray.numInstances = ( unsigned int )g_instances.size();

    OptixAccelBuildOptions accelOptions = {};

    accelOptions.buildFlags = OPTIX_BUILD_FLAG_NONE;
    accelOptions.operation = OPTIX_BUILD_OPERATION_BUILD;

    OptixAccelBufferSizes gasBufferSizes, iasBufferSizes;

    OPTIX_CHECK( optixAccelComputeMemoryUsage( context, &accelOptions, &triangleInput, 1, &gasBufferSizes ) );
    OPTIX_CHECK( optixAccelComputeMemoryUsage( context, &accelOptions, &instanceInput, 1, &iasBufferSizes ) );

    CUdeviceptr            d_tempBuffer, d_gasOutputBuffer, d_iasOutputBuffer;
    OptixTraversableHandle gasHandle, iasHandle;

    CUDA_CHECK( cudaMalloc( ( void** )&d_tempBuffer, std::max( gasBufferSizes.tempSizeInBytes, iasBufferSizes.tempSizeInBytes ) ) );
    CUDA_CHECK( cudaMalloc( ( void** )&d_gasOutputBuffer, gasBufferSizes.outputSizeInBytes ) );
    CUDA_CHECK( cudaMalloc( ( void** )&d_iasOutputBuffer, iasBufferSizes.outputSizeInBytes ) );

    OPTIX_CHECK( optixAccelBuild( context, 0, &accelOptions, &triangleInput, 1, d_tempBuffer, gasBufferSizes.tempSizeInBytes,
        d_gasOutputBuffer, gasBufferSizes.outputSizeInBytes, &gasHandle, nullptr, 0 ) );

    std::vector<OptixInstance> instances( g_instances.size() );

    for( unsigned int i = 0; i < g_instances.size(); ++i )
    {
        instances[i].flags = OPTIX_INSTANCE_FLAG_NONE;
        instances[i].instanceId = 42 + i;
        instances[i].sbtOffset = i;
        instances[i].visibilityMask = 1;
        memcpy( instances[i].transform, g_instances[i].transform, sizeof( float ) * 12 );
        instances[i].traversableHandle = gasHandle;
    }

    CUDA_CHECK( cudaMemcpy( ( void* )d_instances, instances.data(), instancesSizeInBytes, cudaMemcpyHostToDevice ) );

    OPTIX_CHECK( optixAccelBuild( context, 0, &accelOptions, &instanceInput, 1, d_tempBuffer, iasBufferSizes.tempSizeInBytes,
        d_iasOutputBuffer, iasBufferSizes.outputSizeInBytes, &iasHandle, nullptr, 0 ) );

    // relocate gas and ias

    OptixAccelRelocationInfo gasRelocationInfo, iasRelocationInfo;
    OPTIX_CHECK( optixAccelGetRelocationInfo( context, gasHandle, &gasRelocationInfo ) );
    OPTIX_CHECK( optixAccelGetRelocationInfo( context, iasHandle, &iasRelocationInfo ) );

    CUdeviceptr d_gasRelocateBuffer, d_iasRelocateBuffer;
    CUDA_CHECK( cudaMalloc( ( void** )&d_gasRelocateBuffer, gasBufferSizes.outputSizeInBytes ) );
    CUDA_CHECK( cudaMalloc( ( void** )&d_iasRelocateBuffer, iasBufferSizes.outputSizeInBytes ) );

    CUDA_CHECK( cudaMemcpy( ( void* )d_gasRelocateBuffer, ( const void* )d_gasOutputBuffer, gasBufferSizes.outputSizeInBytes, cudaMemcpyDeviceToDevice ) );
    CUDA_CHECK( cudaMemcpy( ( void* )d_iasRelocateBuffer, ( const void* )d_iasOutputBuffer, iasBufferSizes.outputSizeInBytes, cudaMemcpyDeviceToDevice ) );

    OPTIX_CHECK( optixAccelRelocate( context, 0, &gasRelocationInfo, 0, 0, d_gasRelocateBuffer, gasBufferSizes.outputSizeInBytes, &gasHandle ) );

    // SDK 7.5.0 Relocation API

    CUdeviceptr d_instanceTraversableHandles;
    CUDA_CHECK( cudaMalloc( ( void** )&d_instanceTraversableHandles, sizeof( OptixTraversableHandle ) ) );
    CUDA_CHECK( cudaMemcpy( ( void* )d_instanceTraversableHandles, &gasHandle, sizeof( OptixTraversableHandle ), cudaMemcpyHostToDevice ) );
    OPTIX_CHECK( optixAccelRelocate( context, 0, &iasRelocationInfo, d_instanceTraversableHandles, 1, d_iasRelocateBuffer, iasBufferSizes.outputSizeInBytes, &iasHandle ) );
}
