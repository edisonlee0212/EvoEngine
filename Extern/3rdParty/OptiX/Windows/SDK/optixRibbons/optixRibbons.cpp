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
#include <optix_function_table_definition.h>
#include <optix_stack_size.h>
#include <optix_stubs.h>

#include <cuda_runtime.h>

#include <sampleConfig.h>

#include <sutil/CUDAOutputBuffer.h>
#include <sutil/Exception.h>
#include <sutil/sutil.h>
#include <sutil/vec_math.h>

#include "optixRibbons.h"

#include <array>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

#include <sutil/Camera.h>
#include <sutil/Trackball.h>

template <typename T>
struct SbtRecord
{
    __align__( OPTIX_SBT_RECORD_ALIGNMENT ) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    T data;
};

typedef SbtRecord<RayGenData>   RayGenSbtRecord;
typedef SbtRecord<MissData>     MissSbtRecord;
typedef SbtRecord<HitGroupData> HitGroupSbtRecord;

void configureCamera( sutil::Camera& cam, const uint32_t width, const uint32_t height )
{
    cam.setEye( { -8.0f, 3.7f, 2.6f } );
    cam.setLookat( { -7.7f, 2.92f, 4.4f } );
 
    cam.setUp( {0.0f, 1.0f, 0.0f} );
    cam.setFovY( 35.0f );
    cam.setAspectRatio( (float)width / (float)height );
}


static void context_log_cb( unsigned int level, const char* tag, const char* message, void* /*cbdata */ )
{
    std::cerr << "[" << std::setw( 2 ) << level << "][" << std::setw( 12 ) << tag << "]: " << message << "\n";
}


void printUsageAndExit( const char* argv0 )
{
    std::cerr << "Usage  : " << argv0 << " [options]\n";
    std::cerr << "Options: --file  | -f <filename>     Specify file for image output.\n";
    std::cerr << "         --help  | -h                Print this usage message.\n";
    std::cerr << "         --dim=<width>x<height>      Set image dimensions; defaults to 1024x768.\n";
    std::cerr << "         --normals                   Use input normals for orienting ribbons.\n";
    exit( 1 );
}


int main( int argc, char* argv[] )
{
    //
    // Command-line parameter parsing
    //

    std::string outfile;
    int       width  = 1024;
    int       height = 768;
    bool      ribbon_normals = false;

    for( int i = 1; i < argc; ++i )
    {
        const std::string arg( argv[i] );
        if( arg == "--help" || arg == "-h" )
        {
            printUsageAndExit( argv[0] );
        }
        else if( arg == "--file" || arg == "-f" )
        {
            if( i < argc - 1 )
            {
                outfile = argv[++i];
            }
            else
            {
                printUsageAndExit( argv[0] );
            }
        }
        else if( arg.substr( 0, 6 ) == "--dim=" )
        {
            const std::string dims_arg = arg.substr( 6 );
            sutil::parseDimensions( dims_arg.c_str(), width, height );
        }
        else if( arg == "--normals" )
        {
            ribbon_normals = true;
        }
        else
        {
            std::cerr << "Unknown option '" << arg << "'\n";
            printUsageAndExit( argv[0] );
        }
    }

    try
    {
        //
        // Initialize CUDA and create OptiX context
        //
        OptixDeviceContext context = nullptr;
        {
            // Initialize CUDA
            CUDA_CHECK( cudaFree( 0 ) );

            // Initialize the OptiX API, loading all API entry points
            OPTIX_CHECK( optixInit() );

            // Specify context options
            OptixDeviceContextOptions options = {};
            options.logCallbackFunction       = &context_log_cb;
            options.logCallbackLevel          = 4;
#if !defined( NDEBUG )
            // This may incur significant performance cost and should only be done during development.
            options.validationMode = OPTIX_DEVICE_CONTEXT_VALIDATION_MODE_ALL;
#endif

            // Associate a CUDA context (and therefore a specific GPU) with this
            // device context
            CUcontext cuCtx = 0;  // zero means take the current context
            OPTIX_CHECK( optixDeviceContextCreate( cuCtx, &options, &context ) );
        }

        //
        // build acceleration structure for ribbons
        //
        OptixTraversableHandle gas_handle;
        CUdeviceptr            d_gas_output_buffer;

        const unsigned int buildFlags = OPTIX_BUILD_FLAG_ALLOW_RANDOM_VERTEX_ACCESS | OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;
        {
            OptixAccelBuildOptions accel_options = {};
            accel_options.buildFlags             = buildFlags;
            accel_options.operation              = OPTIX_BUILD_OPERATION_BUILD;

            // Ribbon build input, quadratic B-spline, given as lists of vertices, widths, indices,
            // and optional normals.
            // See optixRibbons.h for the input data used here.

            const size_t vertices_size = sizeof( float3 ) * num_vertices;
            CUdeviceptr  d_vertices    = 0;
            CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &d_vertices ), vertices_size ) );
            CUDA_CHECK( cudaMemcpy( reinterpret_cast<void*>( d_vertices ), vertices.data(), vertices_size, cudaMemcpyHostToDevice ) );


            const size_t widthsSize = sizeof( float ) * num_vertices;
            CUdeviceptr  d_widths   = 0;
            CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &d_widths ), widthsSize ) );
            CUDA_CHECK( cudaMemcpy( reinterpret_cast<void*>( d_widths ), widths.data(), widthsSize, cudaMemcpyHostToDevice ) );

            CUdeviceptr d_normals = 0;
            if( ribbon_normals )
            {
                const size_t normals_size = sizeof( float3 ) * num_vertices;
                CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &d_normals ), normals_size ) );
                CUDA_CHECK( cudaMemcpy( reinterpret_cast<void*>( d_normals ), normals.data(), normals_size, cudaMemcpyHostToDevice ) );
            }

            const size_t indicesSize = sizeof( int ) * num_indices;
            CUdeviceptr  d_indices   = 0;
            CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &d_indices ), indicesSize ) );
            CUDA_CHECK( cudaMemcpy( reinterpret_cast<void*>( d_indices ), indices.data(), indicesSize, cudaMemcpyHostToDevice ) );

            // Curve build input.
            OptixBuildInput curve_input = {};

            curve_input.type                 = OPTIX_BUILD_INPUT_TYPE_CURVES;
            curve_input.curveArray.curveType = OPTIX_PRIMITIVE_TYPE_FLAT_QUADRATIC_BSPLINE;

            curve_input.curveArray.numPrimitives        = num_indices;
            curve_input.curveArray.vertexBuffers        = &d_vertices;
            curve_input.curveArray.numVertices          = num_vertices;
            curve_input.curveArray.vertexStrideInBytes  = sizeof( float3 );
            curve_input.curveArray.widthBuffers         = &d_widths;
            curve_input.curveArray.widthStrideInBytes   = sizeof( float );
            curve_input.curveArray.normalBuffers        = ribbon_normals ? &d_normals : 0;
            curve_input.curveArray.normalStrideInBytes  = ribbon_normals ? sizeof( float3 ) : 0;
            curve_input.curveArray.indexBuffer          = d_indices;
            curve_input.curveArray.indexStrideInBytes   = sizeof( int );
            curve_input.curveArray.flag                 = OPTIX_GEOMETRY_FLAG_NONE;
            curve_input.curveArray.primitiveIndexOffset = 0;

            OptixAccelBufferSizes gas_buffer_sizes;
            OPTIX_CHECK( optixAccelComputeMemoryUsage( context, &accel_options, &curve_input,
                                                       1,  // Number of build inputs
                                                       &gas_buffer_sizes ) );

            CUdeviceptr d_temp_buffer_gas;
            CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &d_temp_buffer_gas ), gas_buffer_sizes.tempSizeInBytes ) );
            CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &d_gas_output_buffer ), gas_buffer_sizes.outputSizeInBytes ) );

            OPTIX_CHECK( optixAccelBuild( context, 0,  // CUDA stream
                                          &accel_options, &curve_input,
                                          1,  // num build inputs
                                          d_temp_buffer_gas, gas_buffer_sizes.tempSizeInBytes, d_gas_output_buffer,
                                          gas_buffer_sizes.outputSizeInBytes, &gas_handle,
                                          nullptr,  // emitted property list
                                          0 ) );    // num emitted properties

            // We can now free the scratch space buffer used during build and the vertex
            // inputs, since they are not needed by our trivial shading method
            CUDA_CHECK( cudaFree( reinterpret_cast<void*>( d_temp_buffer_gas ) ) );
            CUDA_CHECK( cudaFree( reinterpret_cast<void*>( d_vertices ) ) );
            CUDA_CHECK( cudaFree( reinterpret_cast<void*>( d_widths ) ) );
            if( ribbon_normals )
                CUDA_CHECK( cudaFree( reinterpret_cast<void*>( d_normals ) ) );
            CUDA_CHECK( cudaFree( reinterpret_cast<void*>( d_indices ) ) );
        }

        //
        // Create modules
        //
        OptixModule                 shading_module           = nullptr;
        OptixModule                 geometry_module          = nullptr;
        OptixPipelineCompileOptions pipeline_compile_options = {};
        {
            OptixModuleCompileOptions module_compile_options = {};
#if !defined( NDEBUG )
            module_compile_options.optLevel   = OPTIX_COMPILE_OPTIMIZATION_LEVEL_0;
            module_compile_options.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
#endif
            pipeline_compile_options.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
            pipeline_compile_options.numPayloadValues      = 3;
            pipeline_compile_options.numAttributeValues    = 2; // u, v ribbon parameters
            pipeline_compile_options.exceptionFlags        = OPTIX_EXCEPTION_FLAG_NONE;
            pipeline_compile_options.pipelineLaunchParamsVariableName = "params";
            pipeline_compile_options.usesPrimitiveTypeFlags = OPTIX_PRIMITIVE_TYPE_FLAGS_FLAT_QUADRATIC_BSPLINE;
         
            size_t      inputSize  = 0;
            const char* input      = sutil::getInputData( OPTIX_SAMPLE_NAME, OPTIX_SAMPLE_DIR, "optixRibbons.cu", inputSize );
            OPTIX_CHECK_LOG( optixModuleCreate( context, &module_compile_options, &pipeline_compile_options, input,
                                                inputSize, LOG, &LOG_SIZE, &shading_module ) );

            OptixBuiltinISOptions builtinISOptions = {};
            builtinISOptions.builtinISModuleType = OPTIX_PRIMITIVE_TYPE_FLAT_QUADRATIC_BSPLINE;
            builtinISOptions.buildFlags = buildFlags;
            OPTIX_CHECK( optixBuiltinISModuleGet( context, &module_compile_options, &pipeline_compile_options,
                                                  &builtinISOptions, &geometry_module ) );
        }

        //
        // Create program groups
        //
        OptixProgramGroup raygen_prog_group   = nullptr;
        OptixProgramGroup miss_prog_group     = nullptr;
        OptixProgramGroup hitgroup_prog_group = nullptr;
        {
            OptixProgramGroupOptions program_group_options = {};  // Initialize to zeros

            OptixProgramGroupDesc raygen_prog_group_desc    = {};  //
            raygen_prog_group_desc.kind                     = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
            raygen_prog_group_desc.raygen.module            = shading_module;
            raygen_prog_group_desc.raygen.entryFunctionName = "__raygen__basic";
            OPTIX_CHECK_LOG( optixProgramGroupCreate( context, &raygen_prog_group_desc,
                                                      1,  // num program groups
                                                      &program_group_options, LOG, &LOG_SIZE, &raygen_prog_group ) );

            OptixProgramGroupDesc miss_prog_group_desc  = {};
            miss_prog_group_desc.kind                   = OPTIX_PROGRAM_GROUP_KIND_MISS;
            miss_prog_group_desc.miss.module            = shading_module;
            miss_prog_group_desc.miss.entryFunctionName = "__miss__ms";
            OPTIX_CHECK_LOG( optixProgramGroupCreate( context, &miss_prog_group_desc,
                                                      1,  // num program groups
                                                      &program_group_options, LOG, &LOG_SIZE, &miss_prog_group ) );

            OptixProgramGroupDesc hitgroup_prog_group_desc        = {};
            hitgroup_prog_group_desc.kind                         = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
            hitgroup_prog_group_desc.hitgroup.moduleCH            = shading_module;
            hitgroup_prog_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__ch";
            hitgroup_prog_group_desc.hitgroup.moduleIS            = geometry_module;
            hitgroup_prog_group_desc.hitgroup.entryFunctionNameIS = 0; // automatically supplied for built-in module
            OPTIX_CHECK_LOG( optixProgramGroupCreate( context, &hitgroup_prog_group_desc,
                                                      1,  // num program groups
                                                      &program_group_options, LOG, &LOG_SIZE, &hitgroup_prog_group ) );
        }

        //
        // Link pipeline
        //
        OptixPipeline pipeline = nullptr;
        {
            const uint32_t    max_trace_depth  = 1;
            OptixProgramGroup program_groups[] = {raygen_prog_group, miss_prog_group, hitgroup_prog_group};

            OptixPipelineLinkOptions pipeline_link_options = {};
            pipeline_link_options.maxTraceDepth            = max_trace_depth;
            OPTIX_CHECK_LOG( optixPipelineCreate( context, &pipeline_compile_options, &pipeline_link_options,
                                                  program_groups, sizeof( program_groups ) / sizeof( program_groups[0] ),
                                                  LOG, &LOG_SIZE, &pipeline ) );

            OptixStackSizes stack_sizes = {};
            for( auto& prog_group : program_groups )
            {
                OPTIX_CHECK( optixUtilAccumulateStackSizes( prog_group, &stack_sizes, pipeline ) );
            }

            uint32_t direct_callable_stack_size_from_traversal;
            uint32_t direct_callable_stack_size_from_state;
            uint32_t continuation_stack_size;
            OPTIX_CHECK( optixUtilComputeStackSizes( &stack_sizes, max_trace_depth,
                                                     0,  // maxCCDepth
                                                     0,  // maxDCDEpth
                                                     &direct_callable_stack_size_from_traversal,
                                                     &direct_callable_stack_size_from_state, &continuation_stack_size ) );
            OPTIX_CHECK( optixPipelineSetStackSize( pipeline, direct_callable_stack_size_from_traversal,
                                                    direct_callable_stack_size_from_state, continuation_stack_size,
                                                    1  // maxTraversableDepth
                                                    ) );
        }

        //
        // Set up shader binding table
        //
        OptixShaderBindingTable sbt = {};
        {
            CUdeviceptr  raygen_record;
            const size_t raygen_record_size = sizeof( RayGenSbtRecord );
            CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &raygen_record ), raygen_record_size ) );
            RayGenSbtRecord rg_sbt;
            OPTIX_CHECK( optixSbtRecordPackHeader( raygen_prog_group, &rg_sbt ) );
            CUDA_CHECK( cudaMemcpy( reinterpret_cast<void*>( raygen_record ), &rg_sbt, raygen_record_size, cudaMemcpyHostToDevice ) );

            CUdeviceptr miss_record;
            size_t      miss_record_size = sizeof( MissSbtRecord );
            CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &miss_record ), miss_record_size ) );
            MissSbtRecord ms_sbt;
            ms_sbt.data = {0.0f, 0.2f, 0.6f};  // background color (blue)
            OPTIX_CHECK( optixSbtRecordPackHeader( miss_prog_group, &ms_sbt ) );
            CUDA_CHECK( cudaMemcpy( reinterpret_cast<void*>( miss_record ), &ms_sbt, miss_record_size, cudaMemcpyHostToDevice ) );

            CUdeviceptr hitgroup_record;
            size_t      hitgroup_record_size = sizeof( HitGroupSbtRecord );
            CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &hitgroup_record ), hitgroup_record_size ) );
            HitGroupSbtRecord hg_sbt;
            OPTIX_CHECK( optixSbtRecordPackHeader( hitgroup_prog_group, &hg_sbt ) );
            CUDA_CHECK( cudaMemcpy( reinterpret_cast<void*>( hitgroup_record ), &hg_sbt, hitgroup_record_size, cudaMemcpyHostToDevice ) );

            sbt.raygenRecord                = raygen_record;
            sbt.missRecordBase              = miss_record;
            sbt.missRecordStrideInBytes     = sizeof( MissSbtRecord );
            sbt.missRecordCount             = 1;
            sbt.hitgroupRecordBase          = hitgroup_record;
            sbt.hitgroupRecordStrideInBytes = sizeof( HitGroupSbtRecord );
            sbt.hitgroupRecordCount         = 1;
        }

        sutil::CUDAOutputBuffer<uchar4> output_buffer( sutil::CUDAOutputBufferType::CUDA_DEVICE, width, height );

        //
        // launch
        //
        {
            CUstream stream;
            CUDA_CHECK( cudaStreamCreate( &stream ) );

            sutil::Camera cam;
            configureCamera( cam, width, height );

            Params params;
            params.image        = output_buffer.map();
            params.image_width  = width;
            params.image_height = height;
            params.handle       = gas_handle;
            params.cam_eye      = cam.eye();
            cam.UVWFrame( params.cam_u, params.cam_v, params.cam_w );

            CUdeviceptr d_param;
            CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &d_param ), sizeof( Params ) ) );
            CUDA_CHECK( cudaMemcpy( reinterpret_cast<void*>( d_param ), &params, sizeof( params ), cudaMemcpyHostToDevice ) );

            OPTIX_CHECK( optixLaunch( pipeline, stream, d_param, sizeof( Params ), &sbt, width, height, /*depth=*/1 ) );
            CUDA_SYNC_CHECK();

            output_buffer.unmap();
            CUDA_CHECK( cudaFree( reinterpret_cast<void*>( d_param ) ) );
        }

        //
        // Display results
        //
        {
            sutil::ImageBuffer buffer;
            buffer.data         = output_buffer.getHostPointer();
            buffer.width        = width;
            buffer.height       = height;
            buffer.pixel_format = sutil::BufferImageFormat::UNSIGNED_BYTE4;
            if( outfile.empty() )
                sutil::displayBufferWindow( argv[0], buffer );
            else
                sutil::saveImage( outfile.c_str(), buffer, false );
        }

        //
        // Cleanup
        //
        {
            CUDA_CHECK( cudaFree( reinterpret_cast<void*>( sbt.raygenRecord ) ) );
            CUDA_CHECK( cudaFree( reinterpret_cast<void*>( sbt.missRecordBase ) ) );
            CUDA_CHECK( cudaFree( reinterpret_cast<void*>( sbt.hitgroupRecordBase ) ) );
            CUDA_CHECK( cudaFree( reinterpret_cast<void*>( d_gas_output_buffer ) ) );

            OPTIX_CHECK( optixPipelineDestroy( pipeline ) );
            OPTIX_CHECK( optixProgramGroupDestroy( hitgroup_prog_group ) );
            OPTIX_CHECK( optixProgramGroupDestroy( miss_prog_group ) );
            OPTIX_CHECK( optixProgramGroupDestroy( raygen_prog_group ) );
            OPTIX_CHECK( optixModuleDestroy( shading_module ) );
            OPTIX_CHECK( optixModuleDestroy( geometry_module ) );

            OPTIX_CHECK( optixDeviceContextDestroy( context ) );
        }
    }
    catch( std::exception& e )
    {
        std::cerr << "Caught exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
