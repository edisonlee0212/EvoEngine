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

#include <glad/glad.h>  // Needs to be included before gl_interop

#include <cuda_gl_interop.h>
#include <cuda_runtime.h>
#include <cuda_fp16.h>

#include <optix.h>
#include <optix_function_table_definition.h>
#include <optix_stubs.h>
#include <optix_micromap.h>

#include <sampleConfig.h>

#include <sutil/Aabb.h>
#include <sutil/CuBuffer.h>
#include <sutil/CUDAOutputBuffer.h>
#include <sutil/Camera.h>
#include <sutil/Exception.h>
#include <sutil/GLDisplay.h>
#include <sutil/Matrix.h>
#include <sutil/Trackball.h>
#include <sutil/sutil.h>
#include <sutil/vec_math.h>
#include <sutil/Scene.h>
#include <optix_stack_size.h>

#include <GLFW/glfw3.h>

#include "optixDisplacedMicromesh.h"

#include <cstdlib>
#include <cstdint>
#include <array>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>


//------------------------------------------------------------------------------
//
// Local types
//
//------------------------------------------------------------------------------

#define MB( X ) X.byteSize() / 1024 / 1024.f

template <typename T>
struct Record
{
    __align__( OPTIX_SBT_RECORD_ALIGNMENT ) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    T data;
};

typedef Record<RayGenData>   RayGenRecord;
typedef Record<MissData>     MissRecord;
typedef Record<HitGroupData> HitGroupRecord;

struct AS
{
    CuBuffer<>             d_data;
    OptixTraversableHandle handle;
};

struct MeshData
{
    unsigned int        numTriangles;
    std::vector<float3> positions;
    std::vector<float2> uvs;
    AS                  gas;
    AS                  ias;

    struct DmmArray
    {
        CuBuffer<>                              d_dmmArrayData;
        OptixDisplacementMicromapUsageCount     usage;
        OptixDisplacementMicromapHistogramEntry histogram;
    } dmmArray;
};

struct DisplacedMicromeshState
{
    OptixDeviceContext context = 0;

    CuBuffer<float4> accumBuffer;

    MeshData mesh;

    unsigned int                   geomFlags = OPTIX_GEOMETRY_FLAG_NONE;
    unsigned int                   buildFlags = 0;

    OptixModule                    module = 0;
    OptixModule                    builtinModule = 0;
    OptixPipelineCompileOptions    pipeline_compile_options = {};
    OptixPipeline                  pipeline = 0;

    OptixProgramGroup raygen_prog_group      = 0;
    OptixProgramGroup miss_group             = 0;
    OptixProgramGroup miss_group_occlusion   = 0;
    OptixProgramGroup hit_group              = 0;

    CUstream                       stream = 0;
    Globals                        params;
    CuBuffer<Globals>              d_params;

    OptixShaderBindingTable  sbt = {};
    CuBuffer<RayGenRecord>   d_sbtRg;
    CuBuffer<MissRecord>     d_sbtMs;
    CuBuffer<HitGroupRecord> d_sbtHit;

    bool enableDMMs = true;
    int dmmSubdivisionLevel = 3;
    float displacementScale = 1.0f;

    bool render = true;
    bool renderAO = true;
};

bool resize_dirty = false;
bool minimized    = false;

// Camera state
bool             camera_changed = true;
sutil::Camera    camera;
sutil::Trackball trackball;

// Mouse state
int32_t mouse_button = -1;

void createBuiltinModule( DisplacedMicromeshState& state, OptixPrimitiveType type );
void createModule( DisplacedMicromeshState& state );
void createProgramGroups( DisplacedMicromeshState& state );
void createPipeline( DisplacedMicromeshState& state );
void buildMeshAccel( DisplacedMicromeshState& state );
void createSBT( DisplacedMicromeshState& state );
void update( DisplacedMicromeshState& state );
void launch( DisplacedMicromeshState& state );
void renderFrame( sutil::CUDAOutputBuffer<uchar4>& output_buffer, DisplacedMicromeshState& state );

//------------------------------------------------------------------------------
//
// GLFW callbacks
//
//------------------------------------------------------------------------------

static void mouseButtonCallback( GLFWwindow* window, int button, int action, int mods )
{
    double xpos, ypos;
    glfwGetCursorPos( window, &xpos, &ypos );
    uint32_t ixpos = static_cast<uint32_t>( xpos );
    uint32_t iypos = static_cast<uint32_t>( ypos );

    if( action == GLFW_PRESS )
    {
        mouse_button = button;
        trackball.startTracking( ixpos, iypos );
    }
    else
    {
        mouse_button = -1;
    }
}


static void cursorPosCallback( GLFWwindow* window, double xpos, double ypos )
{
    DisplacedMicromeshState& state  = *static_cast<DisplacedMicromeshState*>( glfwGetWindowUserPointer( window ) );
    Globals&           params = state.params;

    if( mouse_button == GLFW_MOUSE_BUTTON_LEFT )
    {
        trackball.setViewMode( sutil::Trackball::LookAtFixed );
        trackball.updateTracking( static_cast< int >( xpos ), static_cast< int >( ypos ), params.width, params.height );
        camera_changed = true;
    }
    else if( mouse_button == GLFW_MOUSE_BUTTON_RIGHT )
    {
        trackball.setViewMode( sutil::Trackball::EyeFixed );
        trackball.updateTracking( static_cast< int >( xpos ), static_cast< int >( ypos ), params.width, params.height );
        camera_changed = true;
    }
}


static void windowSizeCallback( GLFWwindow* window, int32_t res_x, int32_t res_y )
{
    // Keep rendering at the current resolution when the window is minimized.
    if( minimized )
        return;

    // Output dimensions must be at least 1 in both x and y.
    sutil::ensureMinimumSize( res_x, res_y );

    Globals& params = static_cast< DisplacedMicromeshState* >( glfwGetWindowUserPointer( window ) )->params;
    params.width = res_x;
    params.height = res_y;
    camera_changed = true;
    resize_dirty = true;
}


static void windowIconifyCallback( GLFWwindow* window, int32_t iconified )
{
    minimized = ( iconified > 0 );
}


static void keyCallback( GLFWwindow* window, int32_t key, int32_t /*scancode*/, int32_t action, int32_t mods )
{
    DisplacedMicromeshState& state = *static_cast<DisplacedMicromeshState*>( glfwGetWindowUserPointer( window ) );
    if( action == GLFW_PRESS )
    {
        if( key == GLFW_KEY_Q || key == GLFW_KEY_ESCAPE )
        {
            glfwSetWindowShouldClose( window, true );
        }
    }
    else if( key == GLFW_KEY_KP_1 )
    {
        state.dmmSubdivisionLevel = max( 0, state.dmmSubdivisionLevel - 1 );
        std::cout << "DMM subdivision levels: " << state.dmmSubdivisionLevel << std::endl;
        buildMeshAccel( state );
        update( state );
    }
    else if( key == GLFW_KEY_KP_2 )
    {
        state.dmmSubdivisionLevel = min( 5, state.dmmSubdivisionLevel + 1 );
        std::cout << "DMM subdivision levels: " << state.dmmSubdivisionLevel << std::endl;
        buildMeshAccel( state );
        update( state );
    }
    else if( key == GLFW_KEY_KP_4 )
    {
        state.displacementScale /= 1.5f;
        std::cout << "displacement Scale: " << state.displacementScale << std::endl;
        buildMeshAccel( state );
        update( state );
    }
    else if( key == GLFW_KEY_KP_5 )
    {
        state.displacementScale *= 1.5f;
        std::cout << "displacement Scale: " << state.displacementScale << std::endl;
        buildMeshAccel( state );
        update( state );
    }
    else if( key == GLFW_KEY_D )
    {
        state.enableDMMs = !state.enableDMMs;
        std::cout << "enable DMMs: " << state.enableDMMs << std::endl;

        buildMeshAccel( state );
        /// Note that getting a built-in IS for triangle and displaced micro-mesh triangle primitives is possible, but not required.
        /// Skip overhead of getting the IS module, creating program groups and a pipeline as well as updating the SBT
        //createBuiltinModule( state, state.enableDMMs ? OPTIX_PRIMITIVE_TYPE_DISPLACED_MICROMESH_TRIANGLE : OPTIX_PRIMITIVE_TYPE_TRIANGLE );
        //createProgramGroups( state );
        //createPipeline( state );
        //createSBT( state );

        update( state );
    }
    else if( key == GLFW_KEY_A )
    {
        state.renderAO = !state.renderAO;

        createModule( state );
        createProgramGroups( state );
        createPipeline( state );
        createSBT( state );

        state.params.ao = state.renderAO;
        update( state );
    }
    else if( key == GLFW_KEY_R )
    {
        createModule( state );
        createProgramGroups( state );
        createPipeline( state );
        createSBT( state );

        update( state );
    }
}


static void scrollCallback( GLFWwindow* window, double xscroll, double yscroll )
{
    if( trackball.wheelEvent( ( int )yscroll ) )
        camera_changed = true;
}


//------------------------------------------------------------------------------
//
// Helper functions
//
//------------------------------------------------------------------------------

float randf()
{
    return static_cast<float>( rand() ) / static_cast<float>( RAND_MAX );
}

void printUsageAndExit( const char* argv0 )
{
    std::cerr << "Usage  : " << argv0 << " [options]\n";
    std::cerr << "Options: --file | -f <filename>      File for image output\n";
    std::cerr << "         --time | -t                 Animation time for image output (default 1)\n";
    std::cerr << "         --frames | -n               Number of animation frames for image output (default 16)\n";
    std::cerr << "         --no-gl-interop             Disable GL interop for display\n";
    std::cerr << "         --dim=<width>x<height>      Set image dimensions; defaults to 1024x768\n";
    std::cerr << "         --help | -h                 Print this usage message\n";
    exit( 0 );
}

void update( DisplacedMicromeshState& state )
{
    state.accumBuffer.allocIfRequired( state.params.width * state.params.height );
    state.accumBuffer.memset( 0 );
    state.params.accum_buffer = state.accumBuffer.get();
    state.params.subframe_index = 0u;

}

void initLaunchGlobals( DisplacedMicromeshState& state )
{
    state.params.frame_buffer   = nullptr;  // Will be set when output buffer is mapped
    state.params.subframe_index = 0u;
    state.params.spp            = 1u;
    state.params.ao             = true;

    CUDA_CHECK( cudaStreamCreate( &state.stream ) );
    state.d_params.alloc( 1 );
    update( state );
}

void handleResize( DisplacedMicromeshState& state, sutil::CUDAOutputBuffer<uchar4>& output_buffer )
{
    if( !resize_dirty )
        return;
    resize_dirty = false;

    output_buffer.resize( state.params.width, state.params.height );
    update( state );
}

void launch( DisplacedMicromeshState& state )
{
    state.d_params.uploadAsync( state.params, state.stream );

    OPTIX_CHECK( optixLaunch(
        state.pipeline,
        state.stream,
        state.d_params.getCU(),
        state.d_params.byteSize(),
        &state.sbt,
        state.params.width,
        state.params.height,
        1
    ) );
}

void renderFrame( sutil::CUDAOutputBuffer<uchar4>& output_buffer, DisplacedMicromeshState& state )
{
    uchar4* result_buffer_data = output_buffer.map();
    state.params.frame_buffer  = result_buffer_data;

    launch( state );

    output_buffer.unmap();
}

void displaySubframe( sutil::CUDAOutputBuffer<uchar4>& output_buffer, sutil::GLDisplay& gl_display, GLFWwindow* window )
{
    // Display
    int framebuf_res_x = 0;  // The display's resolution (could be HDPI res)
    int framebuf_res_y = 0;  //
    glfwGetFramebufferSize( window, &framebuf_res_x, &framebuf_res_y );
    gl_display.display(
        output_buffer.width(),
        output_buffer.height(),
        framebuf_res_x,
        framebuf_res_y,
        output_buffer.getPBO()
    );
}


void initCameraState()
{
    camera.setEye( make_float3( 50, 50, 0 ) + (make_float3( -1.f, -3.f, 2.f )*50) );
    camera.setLookat( make_float3( 50, 50, 0 ) );
    camera.setUp( make_float3( 0.0f, 0.0f, 1.0f ) );
    camera.setFovY( 35.0f );
    camera_changed = true;

    trackball.setCamera( &camera );
    trackball.setMoveSpeed( 10.0f );
    trackball.setReferenceFrame(
        make_float3( 0.0f, 1.0f, 0.0f ),
        make_float3( 1.0f, 0.0f, 0.0f ),
        make_float3( 0.0f, 0.0f, 1.0f )
    );
    trackball.setGimbalLock( true );
}

void handleCameraUpdate( DisplacedMicromeshState& state )
{
    if( !camera_changed )
        return;
    camera_changed = false;

    Globals& params = state.params;
    camera.setAspectRatio( static_cast<float>( params.width ) / static_cast<float>( params.height ) );
    params.eye = camera.eye();
    camera.UVWFrame( params.U, params.V, params.W );
    update( state );
}


static void context_log_cb( unsigned int level, const char* tag, const char* message, void* /*cbdata */ )
{
    std::cerr << "[" << std::setw( 2 ) << level << "][" << std::setw( 12 ) << tag << "]: " << message << "\n";
}

void createContext( DisplacedMicromeshState& state )
{
    // Initialize CUDA
    CUDA_CHECK( cudaFree( 0 ) );

    OptixDeviceContext context;
    CUcontext          cu_ctx = 0;  // zero means take the current context
    OPTIX_CHECK( optixInit() );
    OptixDeviceContextOptions options = {};
    options.logCallbackFunction       = &context_log_cb;
    options.logCallbackLevel          = 4;
    OPTIX_CHECK( optixDeviceContextCreate( cu_ctx, &options, &context ) );

    state.context = context;
}

struct DisplacementBlock64MicroTris64B
{
    // 45 displacement values, implicit vertices
    // 11 bits per displacement values, tightly packed
    // -> 64 bytes per block
    uint8_t data[64];

    // packs the 11 lower bits of the displacement value into the displacement block
    // vertexIdx must be in range [0,44]
    inline void setDisplacement( unsigned vertexIdx, uint16_t displacement )
    {
        constexpr unsigned bitWidth = 11;

        unsigned bitOfs   = bitWidth * vertexIdx;
        unsigned valueOfs = 0;

        while( valueOfs < bitWidth )
        {
            unsigned num = ( ~bitOfs & 7 ) + 1;
            if( bitWidth - valueOfs < num )
                num = bitWidth - valueOfs;

            unsigned mask  = ( 1u << num ) - 1u;
            int      idx   = bitOfs >> 3;
            int      shift = bitOfs & 7;

            unsigned bits = (unsigned)( displacement >> valueOfs ) & mask;  // extract bits from the input value
            data[idx] &= ~( mask << shift );                                // clear bits in memory
            data[idx] |= bits << shift;                                     // insert bits into memory

            valueOfs += num;
            bitOfs += num;
        }
    }

    // The displacement values of the block follow a hierarchical order:
    // The first three values correspond to the vertices of the sub triangle this block is applied to. (subdivision level 0)
    // The following three values correspond to the vertices when splitting the edges of the sub triangle. (subdivision level 1)
    // Afterwards all the remaining values of the vertices for subdivision level 2 follow and so on.

    // The hierarchical subdivision is executed by splitting upright triangles.
    // New vertices are introduced in the following order:
    // 1. splitting edge u
    // 2. splitting edge w
    // 3. splitting edge v
    //
    //              2                           2                |
    //             / \                         / \               |
    //            /   \                       /   \              |
    //           /     \                     /     \             |
    //          /       \                   /       \            |
    //         /         \                 /         \           |
    //     u  /           \  w            3-----------4          |
    //       /             \             / \         / \         |
    //      /               \           /   \       /   \        |
    //     /                 \         /     \     /     \       |
    //    /                   \       /       \   /       \      |
    //   /                     \     /         \ /         \     |
    //  0-----------------------1   0-----------5-----------1    |
    //              v                                            |

    // Hierarchical splitting is done by looping over the triangles in a hierarchical space filling order, introducing new vertices as shown above for all upright triangles.
    // First triangle A, then C, then D.
    // B is skipped as an non-upright triangle.
    //
    //              2                             2                              2                           2                           2               |
    //             / \                           / \                            / \                         / \                         / \              |
    //            /   \                         /   \                          /   \                       /   \                       /   \             |
    //           /     \                       /  ^  \                        /     \                     /     \                     12----13           |
    //          /       \                     /  / D  \                      /   D   \                   /   D   \                   / \   / \           |
    //         /         \                   /  <----  \                    /         \                 /         \                 /   \ /   \          |
    //        /           \                 3-----------4                  3-----------4               3-----------4               3-----14----4         |
    //       /             \               / \  ---->  / \                / \         / \             / \         / \             / \   / \   / \        |
    //      /               \             /   \  B /  /   \              /   \       /   \           /   \       /   \           /   \ /   \ /   \       |
    //     /                 \           /  ^  \  v  /  ^  \            6-----7     /     \         6-----7-----9-----10        6-----7-----9-----10     |
    //    /                   \         /  A \  \   /  C \  \          / \   / \   /   C   \       / \   / \   / \   / \       / \   / \   / \   / \     |
    //   /                     \       /  ---->  \ /  ---->  \        /   \ /   \ /         \     /   \ /   \ /   \ /   \     /   \ /   \ /   \ /   \    |
    //  0-----------------------1     0-----------5-----------1      0-----8-----5-----------1   0-----8-----5-----11----1   0-----8-----5-----11----1   |
    //
    // Equally, values for subdivision level 3 are added by bisecting the edges of the upright triangles in the space filling order of triangles:
    // A, C, D, F, I, K, L, M, O, P (10 * 3 new values)
    //
    //                  2                                   2                                   2                   |
    //                 / \                                 / \                                 / \                  |
    //                /   \                               xx--xx                              42--43                |
    //               /  P  \                             / \ / \                             / \ / \                |
    //              12------13                          12--xx--13                          12--44--13              |
    //             / \  N  / \                         / \ / \ / \                         / \ / \ / \              |
    //            /   \   /   \                       xx--xx--xx--xx                      39--40--36--37            |
    //           /  O  \ /  M  \                     / \ / \ / \ / \                     / \ / \ / \ / \            |
    //          3-------14------4                   3---xx--14--xx--4                   3---41--14--38--4           |
    //         / \  E  / \  G  / \                 / \ / \ / \ / \ / \                 / \ / \ / \ / \ / \          |
    //        /   \   /   \   /   \               xx--xx--xx--xx--xx--xx              21--22--24--25--33--34        |
    //       /  D  \ /  F  \ /  L  \             / \ / \ / \ / \ / \ / \             / \ / \ / \ / \ / \ / \        |
    //      6-------7-------9-------10          6---xx--7---xx--9---xx--10          6---23--7---26--9---35--10      |
    //     / \  B  / \  H  / \  J  / \         / \ / \ / \ / \ / \ / \ / \         / \ / \ / \ / \ / \ / \ / \      |
    //    /   \   /   \   /   \   /   \       15--16--xx--xx--xx--xx--xx--xx      15--16--18--19--27--28--30--31    |
    //   /  A  \ /  C  \ /  I  \ /  K  \     / \ / \ / \ / \ / \ / \ / \ / \     / \ / \ / \ / \ / \ / \ / \ / \    |
    //  0-------8-------5-------11------1   0---17--8---xx--5---xx--11--xx--1   0---17--8---20--5---29--11--32--1   |
    //                                                                                                              |
};


void buildDMMArray( OptixDeviceContext context, CUstream cuStream, CuBuffer<>& buildTemp, MeshData& mesh, unsigned int dmmSubdivisionLevel )
{
    // Based on the subdivision level [0,5], we compute the number of sub triangles.
    // In this sample, we fix the format to OPTIX_DISPLACEMENT_MICROMAP_FORMAT_64_MICRO_TRIS_64_BYTES, which corresponds to 1 sub triangle at subdivision levels 0-3.
    // Level 4 requires 4 sub triangles, level 5 requires 16 sub triangles.
    const unsigned int dmmSubdivisionLevelSubTriangles = max( 0, (int)dmmSubdivisionLevel - 3 );
    const unsigned int numSubTrianglesPerBaseTriangle  = 1 << ( 2 * dmmSubdivisionLevelSubTriangles );
    constexpr int      subTriSizeByteSize = 64;  // 64B for format OPTIX_DISPLACEMENT_MICROMAP_FORMAT_64_MICRO_TRIS_64_BYTES

    const unsigned int numTriangles = mesh.numTriangles;
    const unsigned int numSubTriangles = numTriangles * numSubTrianglesPerBaseTriangle;

    // For the sake of simplicity we fill in the following DMM data (directions, displacement values, descriptors) on host.
    // If not read directly from file, one would otherwise compute/fill these in in a CUDA kernel.

    std::vector<DisplacementBlock64MicroTris64B> displacementValues( numSubTriangles ); // One displacement block per sub triangle!

    // mapping the max input value in float to the max value in 11b
    constexpr float maxDisplacement = 1.f;
    constexpr float maxOverMaxDisp = 0x7FF / maxDisplacement;

    auto proceduralSineDisplacement = [&maxOverMaxDisp]( const float2& uv ) -> uint16_t
    {
        // Compute procedural displacement for a simple sine wave
        // 1. map uv [0,1] to [-1,1] range
        float2 pos = 2 * uv - 1.f;
        // 2. distance to center
        float  d   = sqrtf( pos.x * pos.x + pos.y * pos.y );
        // 3. sine wave with output in range [0,1]
        float  h   = sinf( d * M_PIf * 10 ) * 0.5f + 0.5f;
        // Quantization from float to 11b values, note that the rounding is up to the application.
        return uint16_t( h * maxOverMaxDisp );
    };

    // Instead of filling a displacement block in the hierarchical order as specified above (memory layout of the block), a simple lookup table
    //  is used specify the displacement values in a u-major order (u edge connects vertices 0/2)
    //
    // Offset into vertex index LUT (u major to hierarchical order) for subdivision levels 0 to 3
    // 6  values for subdiv lvl 1
    // 15 values for subdiv lvl 2
    // 45 values for subdiv lvl 3
    static const uint16_t UMAJOR_TO_HIERARCHICAL_VTX_IDX_LUT_OFFSET[5] ={ 0, 3, 9, 24, 69 };
    // LUTs for levels [0,3]
    static const uint16_t UMAJOR_TO_HIERARCHICAL_VTX_IDX_LUT[69]       ={
        // level 0
        0, 2, 1,
        // level 1
        0, 3, 2, 5, 4, 1,
        // level 2
        0, 6, 3, 12, 2, 8, 7, 14, 13, 5, 9, 4, 11, 10, 1,
        // level 3
        0, 15, 6, 21, 3, 39, 12, 42, 2, 17, 16, 23, 22, 41, 40, 44, 43, 8, 18, 7, 24, 14, 36, 13, 20, 19, 26, 25,
        38, 37, 5, 27, 9, 33, 4, 29, 28, 35, 34, 11, 30, 10, 32, 31, 1 };

    for( unsigned int triIdx=0; triIdx<numTriangles; ++triIdx )
    {
        float2 baseUV0 = mesh.uvs[triIdx*3+0];
        float2 baseUV1 = mesh.uvs[triIdx*3+1];
        float2 baseUV2 = mesh.uvs[triIdx*3+2];

        for( unsigned int subTriIdx=0; subTriIdx < numSubTrianglesPerBaseTriangle; ++subTriIdx )
        {
            float2 subTriBary0, subTriBary1, subTriBary2;
            optixMicromapIndexToBaseBarycentrics( subTriIdx, dmmSubdivisionLevelSubTriangles, subTriBary0, subTriBary1, subTriBary2 );

            float2 subTriUV0 = (1 - subTriBary0.x - subTriBary0.y) * baseUV0 + subTriBary0.x * baseUV1 + subTriBary0.y * baseUV2;
            float2 subTriUV1 = (1 - subTriBary1.x - subTriBary1.y) * baseUV0 + subTriBary1.x * baseUV1 + subTriBary1.y * baseUV2;
            float2 subTriUV2 = (1 - subTriBary2.x - subTriBary2.y) * baseUV0 + subTriBary2.x * baseUV1 + subTriBary2.y * baseUV2;

            DisplacementBlock64MicroTris64B& block = displacementValues[triIdx * numSubTrianglesPerBaseTriangle + subTriIdx];
            block ={};

            unsigned perBlockSubdivisionLevel = min( 3u, dmmSubdivisionLevel );
            // fill the displacement block by looping over the vertices in u-major order and use a lookup table to set the corresponding bits in the displacement block
            unsigned uMajorVertIdx = 0;
            unsigned numSegments = 1 << perBlockSubdivisionLevel;
            for( unsigned iu=0; iu < numSegments+1; ++iu )
            {
                for( unsigned iv = 0; iv < numSegments+1 - iu; ++iv )
                {
                    float2 microVertexBary ={ float( iu ) / (numSegments), float( iv ) / (numSegments) };
                    float2 microVertexUV = (1 - microVertexBary.x - microVertexBary.y) * subTriUV0 + microVertexBary.x * subTriUV1 + microVertexBary.y * subTriUV2;

                    uint16_t disp = proceduralSineDisplacement( microVertexUV );

                    block.setDisplacement( UMAJOR_TO_HIERARCHICAL_VTX_IDX_LUT[UMAJOR_TO_HIERARCHICAL_VTX_IDX_LUT_OFFSET[perBlockSubdivisionLevel] + uMajorVertIdx], disp );
                    uMajorVertIdx++;
                }
            }
        }
    }


    //////////////////////////////////////////////////////////////////////////
    // The actual build of the displacement micromap array.
    // Only the displacement values are needed here along with the descriptors.
    // How these values are applied to triangles (displacement directions, indexing, potential scale/bias) is specified at the triangle build input (GAS build)
    MeshData::DmmArray& dmm  = mesh.dmmArray;

    OptixDisplacementMicromapArrayBuildInput bi ={};

    bi.flags = OPTIX_DISPLACEMENT_MICROMAP_FLAG_NONE;
    // We have a very simple distribution of subdivision levels and format usage.
    // All triangles of the mesh use the uncompressed format, and a fixed subdivision level.
    // As such, the histogram over the different formats/subdivision levels has only a single entry.
    // Also, none of the displacement micromaps are re-used between triangles, so we put 'numTriangles' displacement micromaps into an array.
    dmm.histogram.count            = numTriangles;
    dmm.histogram.format           = OPTIX_DISPLACEMENT_MICROMAP_FORMAT_64_MICRO_TRIS_64_BYTES;
    dmm.histogram.subdivisionLevel = dmmSubdivisionLevel;
    bi.numDisplacementMicromapHistogramEntries = 1;
    bi.displacementMicromapHistogramEntries    = &dmm.histogram;

    OptixMicromapBufferSizes bs ={};
    optixDisplacementMicromapArrayComputeMemoryUsage( context, &bi, &bs );

    // Provide the device data for the DMM array build
    std::vector<OptixDisplacementMicromapDesc> descriptors( numTriangles );
    for( unsigned int i = 0; i < numTriangles; ++i )
    {
        OptixDisplacementMicromapDesc& desc = descriptors[i];
        desc.byteOffset                     = i * subTriSizeByteSize * numSubTrianglesPerBaseTriangle;
        desc.format                         = OPTIX_DISPLACEMENT_MICROMAP_FORMAT_64_MICRO_TRIS_64_BYTES;
        desc.subdivisionLevel               = dmmSubdivisionLevel;
    }

    CuBuffer<OptixDisplacementMicromapDesc>   d_descriptors( descriptors );
    CuBuffer<DisplacementBlock64MicroTris64B> d_displacementsValues( displacementValues );
    bi.perDisplacementMicromapDescBuffer = d_descriptors.getCU();
    bi.displacementValuesBuffer          = d_displacementsValues.getCU();

    dmm.d_dmmArrayData.allocIfRequired( bs.outputSizeInBytes );
    buildTemp.allocIfRequired( bs.tempSizeInBytes );

    OptixMicromapBuffers uBuffers ={};
    uBuffers.output               = dmm.d_dmmArrayData.getCU();
    uBuffers.outputSizeInBytes    = dmm.d_dmmArrayData.byteSize();
    uBuffers.temp                 = buildTemp.getCU();
    uBuffers.tempSizeInBytes      = buildTemp.byteSize();

    optixDisplacementMicromapArrayBuild( context, cuStream, &bi, &uBuffers );
}

void buildAndCompact( OptixDeviceContext                  context,
                      CUstream                            cuStream,
                      CuBuffer<>&                         temp,
                      CuBuffer<>&                         output,
                      OptixTraversableHandle&             handle,
                      const OptixAccelBuildOptions&       asOptions,
                      const std::vector<OptixBuildInput>& bi )
{
    OptixAccelBufferSizes bufferSizes;
    OPTIX_CHECK( optixAccelComputeMemoryUsage( context, &asOptions, bi.data(), (unsigned int)bi.size(), &bufferSizes ) );

    if( asOptions.buildFlags & OPTIX_BUILD_FLAG_ALLOW_COMPACTION )
    {
        size_t outputOffset;
        size_t compactedSizeOffset;
        temp.allocIfRequired( temp.pool( bufferSizes.tempSizeInBytes, bufferSizes.outputSizeInBytes, OPTIX_ACCEL_BUFFER_BYTE_ALIGNMENT,
                                         outputOffset, sizeof( size_t ), sizeof( size_t ), compactedSizeOffset ) );

        OptixAccelEmitDesc emitProperty = {};
        emitProperty.type               = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
        emitProperty.result             = temp.getCU( compactedSizeOffset );

        OPTIX_CHECK( optixAccelBuild( context, cuStream, &asOptions, bi.data(), (unsigned int)bi.size(), temp.getCU(),
                                      bufferSizes.tempSizeInBytes, temp.getCU( outputOffset ),
                                      bufferSizes.outputSizeInBytes, &handle, &emitProperty, 1 ) );

        size_t compactedASSize = 0;
        CUDA_CHECK( cudaMemcpy( &compactedASSize, (void*)emitProperty.result, sizeof( size_t ), cudaMemcpyDeviceToHost ) );

        output.alloc( compactedASSize );
        OPTIX_CHECK( optixAccelCompact( context, cuStream, handle, output.getCU(), output.byteSize(), &handle ) );
    }
    else
    {
        temp.allocIfRequired( bufferSizes.tempSizeInBytes );
        output.allocIfRequired( bufferSizes.outputSizeInBytes );

        OPTIX_CHECK( optixAccelBuild( context, cuStream, &asOptions, bi.data(), (unsigned int)bi.size(), temp.getCU(),
                                      bufferSizes.tempSizeInBytes, output.getCU(), bufferSizes.outputSizeInBytes, &handle, 0, 0 ) );
    }
};

void buildMeshAccel( DisplacedMicromeshState& state )
{
    CuBuffer<> buildTemp;

    unsigned int numQuadsPerAxis = 4;  // number of quads per dimension (for a flat surface)
    unsigned int numQuads        = numQuadsPerAxis * numQuadsPerAxis;
    state.mesh.numTriangles      = numQuads * 2;

    if( state.mesh.positions.empty() )
    {
        state.mesh.positions.resize( state.mesh.numTriangles * 3 );
        state.mesh.uvs.resize( state.mesh.numTriangles * 3 );

        // vertex and uv generation for flat quads
        for( unsigned quadU = 0; quadU < numQuadsPerAxis; ++quadU )
        {
            for( unsigned quadV = 0; quadV < numQuadsPerAxis; ++quadV )
            {
                // 2 triangles per quad, 3 positions per triangle
                state.mesh.uvs[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 0] ={ (quadU+0) / (float)numQuadsPerAxis, (quadV+0) / (float)numQuadsPerAxis };
                state.mesh.uvs[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 1] ={ (quadU+1) / (float)numQuadsPerAxis, (quadV+0) / (float)numQuadsPerAxis };
                state.mesh.uvs[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 2] ={ (quadU+1) / (float)numQuadsPerAxis, (quadV+1) / (float)numQuadsPerAxis };
                state.mesh.uvs[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 3] ={ (quadU+0) / (float)numQuadsPerAxis, (quadV+0) / (float)numQuadsPerAxis };
                state.mesh.uvs[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 4] ={ (quadU+1) / (float)numQuadsPerAxis, (quadV+1) / (float)numQuadsPerAxis };
                state.mesh.uvs[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 5] ={ (quadU+0) / (float)numQuadsPerAxis, (quadV+1) / (float)numQuadsPerAxis };

                // a simple tesselated quad, use UVs for position generation
                state.mesh.positions[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 0] = make_float3( state.mesh.uvs[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 0] * 100, 0 );
                state.mesh.positions[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 1] = make_float3( state.mesh.uvs[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 1] * 100, 0 );
                state.mesh.positions[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 2] = make_float3( state.mesh.uvs[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 2] * 100, 0 );
                state.mesh.positions[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 3] = make_float3( state.mesh.uvs[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 3] * 100, 0 );
                state.mesh.positions[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 4] = make_float3( state.mesh.uvs[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 4] * 100, 0 );
                state.mesh.positions[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 5] = make_float3( state.mesh.uvs[(quadU*numQuadsPerAxis + quadV) * 2 * 3 + 5] * 100, 0 );
            }
        }
    }

    if( state.enableDMMs )
        buildDMMArray( state.context, state.stream, buildTemp, state.mesh, state.dmmSubdivisionLevel );

    OptixBuildInput           buildInput = {};
    std::vector<unsigned int> flags;

    // only needed for GAS build
    // this buffer needs to be alive over the GAS build
    CuBuffer<float3> d_positions( state.mesh.positions );

    unsigned int geomFlags = OPTIX_GEOMETRY_FLAG_NONE;

    {
        // Build an AS over the triangles.
        // These are the basic information for non-indexed static triangles
        buildInput.type                              = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
        buildInput.triangleArray.flags               = &geomFlags;
        buildInput.triangleArray.vertexFormat        = OPTIX_VERTEX_FORMAT_FLOAT3;
        buildInput.triangleArray.vertexStrideInBytes = d_positions.stride();
        buildInput.triangleArray.numVertices         = (unsigned int)d_positions.size();
        buildInput.triangleArray.vertexBuffers       = d_positions.getCUAsArray();
        buildInput.triangleArray.numSbtRecords       = 1;
    }

    // this buffer needs to be alive over the GAS build
    CuBuffer<float3> d_displacementDirections;

    // this value needs to be alive over the GAS build
    OptixDisplacementMicromapUsageCount usage = {};

    if( state.enableDMMs )
    {
        // Applying the DMMs to triangles.
        // DMMs in the DMM array need to be index.
        // Also, per triangle data for the displacement is specified, such as the displacement directions.
        MeshData::DmmArray& dmm  = state.mesh.dmmArray;
        OptixBuildInputDisplacementMicromap& disp = buildInput.triangleArray.displacementMicromap;

        disp.indexingMode              = OPTIX_DISPLACEMENT_MICROMAP_ARRAY_INDEXING_MODE_LINEAR;
        disp.displacementMicromapArray = dmm.d_dmmArrayData.getCU();

        // Displacement directions, 3 vectors (these do not need to be normalized!)
        // While the API accepts float values for convenience, OptiX uses the half format internally. Float inputs are converted to half.
        // So it is usually best to input half values directly to control precision.
        // Note that this is not an issue for this sample, using ~( 0, 0, 10 ) as directions everywhere.
        std::vector<float3> directions( state.mesh.numTriangles * 3, make_float3( 0, 0, 10 ) * state.displacementScale );
        d_displacementDirections.allocAndUpload( directions );
        disp.vertexDirectionsBuffer = d_displacementDirections.getCU();
        disp.vertexDirectionFormat  = OPTIX_DISPLACEMENT_MICROMAP_DIRECTION_FORMAT_FLOAT3;

        // Since we create exactly one displacement micromap per triangle and we apply a displacement micromap to every triangle, there
        // is a one to one mapping between the DMM usage and the DMM histogram
        // we could even do a reinterpret_cast from dmm histogram to dmm usage here
        usage.count                             = dmm.histogram.count;
        usage.format                            = dmm.histogram.format;
        usage.subdivisionLevel                  = dmm.histogram.subdivisionLevel;
        disp.numDisplacementMicromapUsageCounts = 1;
        disp.displacementMicromapUsageCounts    = &usage;
    }


    OptixAccelBuildOptions gas_accel_options = {};

    state.buildFlags = OPTIX_BUILD_FLAG_NONE;
    state.buildFlags |= OPTIX_BUILD_FLAG_ALLOW_RANDOM_VERTEX_ACCESS;
    state.buildFlags |= OPTIX_BUILD_FLAG_ALLOW_COMPACTION;
    state.buildFlags |= OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;
    gas_accel_options.operation = OPTIX_BUILD_OPERATION_BUILD;

    gas_accel_options.buildFlags = state.buildFlags;

    std::vector<OptixBuildInput> buildInputs = { buildInput };
    buildAndCompact( state.context, state.stream, buildTemp, state.mesh.gas.d_data, state.mesh.gas.handle, gas_accel_options, buildInputs );

    state.params.handle = state.mesh.gas.handle;
}

void createModule( DisplacedMicromeshState& state )
{
    OptixModuleCompileOptions module_compile_options = {};
#if !defined( NDEBUG )
    module_compile_options.optLevel   = OPTIX_COMPILE_OPTIMIZATION_LEVEL_0;
    module_compile_options.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
#endif

    state.pipeline_compile_options.usesMotionBlur                   = false;
    state.pipeline_compile_options.traversableGraphFlags            = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
    state.pipeline_compile_options.numPayloadValues                 = 4;
    state.pipeline_compile_options.numAttributeValues               = 2;
    state.pipeline_compile_options.pipelineLaunchParamsVariableName = "params";
    state.pipeline_compile_options.usesPrimitiveTypeFlags = OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE | OPTIX_PRIMITIVE_TYPE_FLAGS_DISPLACED_MICROMESH_TRIANGLE;

    state.pipeline_compile_options.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;

    size_t      inputSize = 0;
    const char* input = sutil::getInputData( OPTIX_SAMPLE_NAME, OPTIX_SAMPLE_DIR, "optixDisplacedMicromesh.cu", inputSize );

    OPTIX_CHECK_LOG( optixModuleCreate( state.context, &module_compile_options, &state.pipeline_compile_options, input,
                                        inputSize, LOG, &LOG_SIZE, &state.module ) );  // LOG, LOG_SIZE are part of OPTIX_CHECK_LOG
}

void createBuiltinModule( DisplacedMicromeshState& state, OptixPrimitiveType type )
{
    OptixModuleCompileOptions module_compile_options = {};
    OptixBuiltinISOptions builtin_is_options = {};
    builtin_is_options.builtinISModuleType = type;
    builtin_is_options.buildFlags = state.buildFlags;

    // Note that getting a built-in IS for triangle and displaced micro-mesh triangle primitives is possible, but not required.
    OPTIX_CHECK_LOG( optixBuiltinISModuleGet( state.context, &module_compile_options, &state.pipeline_compile_options,
                                              &builtin_is_options, &state.builtinModule ) );
}


void createProgramGroups( DisplacedMicromeshState& state )
{
    OptixProgramGroupOptions  program_group_options = {};

    {
        OptixProgramGroupDesc raygen_prog_group_desc    = {};
        raygen_prog_group_desc.kind                     = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
        raygen_prog_group_desc.raygen.module            = state.module;
        raygen_prog_group_desc.raygen.entryFunctionName = "__raygen__rg";

        OPTIX_CHECK_LOG( optixProgramGroupCreate( state.context, &raygen_prog_group_desc,
                                                  1,  // num program groups
                                                  &program_group_options, LOG, &LOG_SIZE, &state.raygen_prog_group ) );  // LOG, LOG_SIZE are part of OPTIX_CHECK_LOG
    }

    {
        OptixProgramGroupDesc miss_prog_group_desc  = {};
        miss_prog_group_desc.kind                   = OPTIX_PROGRAM_GROUP_KIND_MISS;
        miss_prog_group_desc.miss.module            = state.module;
        miss_prog_group_desc.miss.entryFunctionName = "__miss__ms";
        OPTIX_CHECK_LOG( optixProgramGroupCreate( state.context, &miss_prog_group_desc,
                                                  1,  // num program groups
                                                  &program_group_options, LOG, &LOG_SIZE, &state.miss_group ) );  // LOG, LOG_SIZE are part of OPTIX_CHECK_LOG
    }

    {
        OptixProgramGroupDesc miss_prog_group_desc  = {};
        miss_prog_group_desc.kind                   = OPTIX_PROGRAM_GROUP_KIND_MISS;
        miss_prog_group_desc.miss.module            = state.module;
        miss_prog_group_desc.miss.entryFunctionName = "__miss__occlusion";
        OPTIX_CHECK_LOG( optixProgramGroupCreate( state.context, &miss_prog_group_desc,
                                                  1,  // num program groups
                                                  &program_group_options, LOG, &LOG_SIZE, &state.miss_group_occlusion ) );  // LOG, LOG_SIZE are part of OPTIX_CHECK_LOG
    }

    {
        OptixProgramGroupDesc hit_prog_group_desc        = {};
        hit_prog_group_desc.kind                         = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
        hit_prog_group_desc.hitgroup.moduleIS            = state.builtinModule;
        hit_prog_group_desc.hitgroup.moduleCH            = state.module;
        hit_prog_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__ch_tri";
        OPTIX_CHECK_LOG( optixProgramGroupCreate( state.context, &hit_prog_group_desc,
                                                  1,  // num program groups
                                                  &program_group_options, LOG, &LOG_SIZE, &state.hit_group ) );  // LOG, LOG_SIZE are part of OPTIX_CHECK_LOG
    }
}


void createPipeline( DisplacedMicromeshState& state )
{
    OptixProgramGroup program_groups[] =
    {
        state.raygen_prog_group,
        state.miss_group,
        state.miss_group_occlusion,
        state.hit_group,
    };

    OptixPipelineLinkOptions pipeline_link_options = {};
    pipeline_link_options.maxTraceDepth = 2;

    OPTIX_CHECK_LOG( optixPipelineCreate( state.context, &state.pipeline_compile_options, &pipeline_link_options,
                                          program_groups, sizeof( program_groups ) / sizeof( program_groups[0] ), LOG,
                                          &LOG_SIZE, &state.pipeline ) );  // LOG, LOG_SIZE are part of OPTIX_CHECK_LOG

    // We need to specify the max traversal depth.  Calculate the stack sizes, so we can specify all
    // parameters to optixPipelineSetStackSize.
    OptixStackSizes stack_sizes = {};
    OPTIX_CHECK( optixUtilAccumulateStackSizes( state.raygen_prog_group, &stack_sizes, state.pipeline ) );
    OPTIX_CHECK( optixUtilAccumulateStackSizes( state.miss_group, &stack_sizes, state.pipeline ) );
    OPTIX_CHECK( optixUtilAccumulateStackSizes( state.miss_group_occlusion, &stack_sizes, state.pipeline ) );
    OPTIX_CHECK( optixUtilAccumulateStackSizes( state.hit_group, &stack_sizes, state.pipeline ) );

    uint32_t max_trace_depth = pipeline_link_options.maxTraceDepth;
    uint32_t max_cc_depth = 0, max_dc_depth = 0;
    uint32_t direct_callable_stack_size_from_traversal, direct_callable_stack_size_from_state, continuation_stack_size;
    OPTIX_CHECK( optixUtilComputeStackSizes( &stack_sizes, max_trace_depth, max_cc_depth, max_dc_depth, &direct_callable_stack_size_from_traversal,
                                             &direct_callable_stack_size_from_state, &continuation_stack_size ) );

    // GAS only traversal
    const uint32_t max_traversable_graph_depth = 1;
    OPTIX_CHECK( optixPipelineSetStackSize( state.pipeline, direct_callable_stack_size_from_traversal, direct_callable_stack_size_from_state,
                                            continuation_stack_size, max_traversable_graph_depth ) );
}


void createSBT( DisplacedMicromeshState& state )
{
    {
        RayGenRecord sbtRg[1] = {};
        OPTIX_CHECK( optixSbtRecordPackHeader( state.raygen_prog_group, &sbtRg[0] ) );

        state.d_sbtRg.allocIfRequired( 1 );
        state.d_sbtRg.upload( sbtRg );

        state.sbt.raygenRecord = state.d_sbtRg.getCU();
    }

    {
        MissRecord sbtMs[2];
        OPTIX_CHECK( optixSbtRecordPackHeader( state.miss_group, &sbtMs[0] ) );
        OPTIX_CHECK( optixSbtRecordPackHeader( state.miss_group_occlusion, &sbtMs[1] ) );
        sbtMs[0].data.bg_color = make_float4( 0.0f );
        sbtMs[1].data.bg_color = make_float4( 0.0f ); // this is never read!

        state.d_sbtMs.allocIfRequired( 2 );
        state.d_sbtMs.upload( sbtMs );

        state.sbt.missRecordBase          = state.d_sbtMs.getCU();
        state.sbt.missRecordStrideInBytes = state.d_sbtMs.stride();
        state.sbt.missRecordCount         = (unsigned int)state.d_sbtMs.size();
    }

    {
        HitGroupRecord sbtHit[1];
        OPTIX_CHECK( optixSbtRecordPackHeader( state.hit_group, &sbtHit[0] ) );
        sbtHit[0].data.color = make_float3( 1 );

        state.d_sbtHit.allocIfRequired( 1 );
        state.d_sbtHit.upload( sbtHit );

        state.sbt.hitgroupRecordBase = state.d_sbtHit.getCU();
        state.sbt.hitgroupRecordStrideInBytes = state.d_sbtHit.stride();
        state.sbt.hitgroupRecordCount = (unsigned int)state.d_sbtHit.size();
    }
}


void cleanupState( DisplacedMicromeshState& state )
{
    OPTIX_CHECK( optixPipelineDestroy( state.pipeline ) );
    OPTIX_CHECK( optixProgramGroupDestroy( state.raygen_prog_group ) );
    OPTIX_CHECK( optixProgramGroupDestroy( state.miss_group ) );
    OPTIX_CHECK( optixProgramGroupDestroy( state.miss_group_occlusion ) );
    OPTIX_CHECK( optixProgramGroupDestroy( state.hit_group ) );
    OPTIX_CHECK( optixModuleDestroy( state.module ) );
    OPTIX_CHECK( optixDeviceContextDestroy( state.context ) );
}


//------------------------------------------------------------------------------
//
// Main
//
//------------------------------------------------------------------------------

int main( int argc, char* argv[] )
{
    DisplacedMicromeshState state = {};
    state.params.width  = 512;
    state.params.height = 512;
    sutil::CUDAOutputBufferType output_buffer_type = sutil::CUDAOutputBufferType::GL_INTEROP;

    //
    // Parse command line options
    //
    std::string outfile;

    for( int i = 1; i < argc; ++i )
    {
        const std::string arg = argv[i];
        if( arg == "--help" || arg == "-h" )
        {
            printUsageAndExit( argv[0] );
        }
        else if( arg == "--no-gl-interop" )
        {
            output_buffer_type = sutil::CUDAOutputBufferType::CUDA_DEVICE;
        }
        else if( arg == "--subdivision-levels" || arg == "-s" )
        {
            if( i >= argc - 1 )
                printUsageAndExit( argv[0] );

            int subdivLevels = atoi( argv[++i] );
            if( subdivLevels >= 0 && subdivLevels <= 5 )
                state.dmmSubdivisionLevel = subdivLevels;
        }
        else if( arg == "--file" || arg == "-f" )
        {
            if( i >= argc - 1 )
                printUsageAndExit( argv[0] );
            outfile = argv[++i];
            output_buffer_type = sutil::CUDAOutputBufferType::CUDA_DEVICE;
        }
        else if( arg.substr( 0, 6 ) == "--dim=" )
        {
            const std::string dims_arg = arg.substr( 6 );
            int               w, h;
            sutil::parseDimensions( dims_arg.c_str(), w, h );
            state.params.width  = w;
            state.params.height = h;
        }
        else
        {
            std::cerr << "Unknown option '" << argv[i] << "'\n";
            printUsageAndExit( argv[0] );
        }
    }

    try
    {
        initCameraState();

        //
        // Set up OptiX state
        //
        createContext( state );

        unsigned int RTCoresVersion = 0;
        OPTIX_CHECK( optixDeviceContextGetProperty( state.context, OPTIX_DEVICE_PROPERTY_RTCORE_VERSION,
                                                    &RTCoresVersion, sizeof( unsigned int ) ) );
        if( RTCoresVersion < 10 )
        {
            std::cerr << "The optixDisplacedMicromesh sample requires a RTX-enabled graphics card to run on.\n";
            exit( 0 );
        }

        buildMeshAccel( state );
        createModule( state );
        createBuiltinModule( state, state.enableDMMs ? OPTIX_PRIMITIVE_TYPE_DISPLACED_MICROMESH_TRIANGLE : OPTIX_PRIMITIVE_TYPE_TRIANGLE );
        createProgramGroups( state );
        createPipeline( state );
        createSBT( state );
        initLaunchGlobals( state );


        const bool interactive = outfile.empty();

        if( interactive )
        {
            std::cout << "////////////////////////////////////////////////////////////////////////////////////////////////////////////\n";
            std::cout << "Keys:\n";
            std::cout << "         KP_1/2                      Decrease/increase DMM subdivision levels\n";
            std::cout << "         KP_4/5                      Decrease/increase displacement scale\n";
            std::cout << "         D                           Toggle: DMM usage\n";
            std::cout << "         A                           Toggle: AO rendering\n";
            std::cout << "////////////////////////////////////////////////////////////////////////////////////////////////////////////\n";
            GLFWwindow* window = sutil::initUI( "optixDisplacedMicromesh", state.params.width, state.params.height );
            glfwSetMouseButtonCallback( window, mouseButtonCallback );
            glfwSetCursorPosCallback( window, cursorPosCallback );
            glfwSetWindowSizeCallback( window, windowSizeCallback );
            glfwSetWindowIconifyCallback( window, windowIconifyCallback );
            glfwSetKeyCallback( window, keyCallback );
            glfwSetScrollCallback( window, scrollCallback );
            glfwSetWindowUserPointer( window, &state );

            //
            // Render loop
            //
            {
                sutil::CUDAOutputBuffer<uchar4> output_buffer( output_buffer_type, state.params.width, state.params.height );
                output_buffer.setStream( state.stream );
                sutil::GLDisplay gl_display;

                std::chrono::duration<double> state_update_time( 0.0 );
                std::chrono::duration<double> render_time( 0.0 );
                std::chrono::duration<double> display_time( 0.0 );

                do
                {
                    auto t0 = std::chrono::steady_clock::now();
                    glfwPollEvents();

                    handleCameraUpdate( state );
                    handleResize( state, output_buffer );

                    auto t1 = std::chrono::steady_clock::now();
                    state_update_time += t1 - t0;
                    t0 = t1;

                    renderFrame( output_buffer, state );
                    t1 = std::chrono::steady_clock::now();
                    render_time += t1 - t0;
                    t0 = t1;

                    displaySubframe( output_buffer, gl_display, window );
                    t1 = std::chrono::steady_clock::now();
                    display_time += t1 - t0;

                    sutil::displayStats( state_update_time, render_time, display_time );

                    glfwSwapBuffers( window );

                    ++state.params.subframe_index;
                } while( !glfwWindowShouldClose( window ) );

                CUDA_SYNC_CHECK();
            }

            sutil::cleanupUI( window );
        }
        else
        {
            handleCameraUpdate( state );

            if( output_buffer_type == sutil::CUDAOutputBufferType::GL_INTEROP )
            {
                sutil::initGLFW();  // For GL context
                sutil::initGL();
            }

            sutil::CUDAOutputBuffer<uchar4> output_buffer( output_buffer_type, state.params.width, state.params.height );
            output_buffer.setStream( state.stream );
            handleResize( state, output_buffer );

            state.params.spp = 4;

            renderFrame( output_buffer, state );
            CUDA_SYNC_CHECK();

            sutil::ImageBuffer buffer;
            buffer.data         = output_buffer.getHostPointer();
            buffer.width        = output_buffer.width();
            buffer.height       = output_buffer.height();
            buffer.pixel_format = sutil::BufferImageFormat::UNSIGNED_BYTE4;
            sutil::saveImage( outfile.c_str(), buffer, false );

            if( output_buffer_type == sutil::CUDAOutputBufferType::GL_INTEROP )
            {
                glfwTerminate();
            }
        }

        cleanupState( state );
    }
    catch( std::exception& e )
    {
        std::cerr << "Caught exception: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
