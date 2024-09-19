if(POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD)
endif()

# Use the older FindCUDA module (deprecated in newer CMake versions)
find_package(CUDA QUIET)

# Check if CUDA is found
if (NOT CUDA_FOUND)
    message(WARNING "CUDA not found. Skipping this plugin.")
    return()
endif()

message("Found CUDA.")

mark_as_advanced(CUDA_SDK_ROOT_DIR)
enable_language(CUDA)


set(RAY_TRACER_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

include(${RAY_TRACER_DIRECTORY}/cmake/configure_optix.cmake)

# Check if OptiX is found
if (NOT OptiX_FOUND)
    message(WARNING "OptiX not found. Skipping this plugin.")
    return()
endif()
message("Found OptiX.")

file(GLOB RAY_TRACER_SOURCES
	${CMAKE_CURRENT_BINARY_DIR}
	${RAY_TRACER_DIRECTORY}/src/*.cpp
	${RAY_TRACER_DIRECTORY}/src/MLVQ/*.cpp
	)

file(GLOB RAY_TRACER_CUDA_SOURCES
	${RAY_TRACER_DIRECTORY}/src/*.cu
	)

set(RAY_TRACER_INCLUDES
	${OptiX_INCLUDE}
	${CUDA_TOOLKIT_ROOT_DIR}/include
	${RAY_TRACER_DIRECTORY}/include
	${RAY_TRACER_DIRECTORY}/include/MLVQ
	${RAY_TRACER_DIRECTORY}/include/RayTracer
	${RAY_TRACER_DIRECTORY}/include/Utilities
	)

include_directories(${EVOENGINE_SDK_INCLUDES})
include_directories(${RAY_TRACER_INCLUDES})

cuda_compile_and_embed(CAMERA_RENDERING_PTX ${RAY_TRACER_DIRECTORY}/src/ptx/CameraRendering.cu)
cuda_compile_and_embed(ILLUMINATION_ESTIMATION_PTX ${RAY_TRACER_DIRECTORY}/src/ptx/IlluminationEstimation.cu)
cuda_compile_and_embed(POINT_CLOUD_SCANNING_PTX ${RAY_TRACER_DIRECTORY}/src/ptx/PointCloudScanning.cu)

add_library(OptiXRayTracerPlugin
	STATIC
	#Default
	${CAMERA_RENDERING_PTX}
	${ILLUMINATION_ESTIMATION_PTX}
	${POINT_CLOUD_SCANNING_PTX}

	#C++
	${RAY_TRACER_SOURCES}
	${RAY_TRACER_CUDA_SOURCES}
	#${CMAKE_MODULE_PATH}/configure_optix.cmake
	#${CMAKE_MODULE_PATH}/FindOptiX.cmake
	)

set_property(TARGET OptiXRayTracerPlugin PROPERTY CUDA_ARCHITECTURES 61-real 61-virtual)

if (NOT WIN32)
	message(STATUS CUDALIB=${CUDA_TOOLKIT_ROOT_DIR})
	set(CUDA_CUDA_LIBRARY ${CUDA_TOOLKIT_ROOT_DIR}/lib64/stubs)
endif ()

# Linker settings for all platforms
if (NOT WIN32)
	target_link_libraries(OptiXRayTracerPlugin
		cuda
	)
else()
	target_link_libraries(OptiXRayTracerPlugin
		${CUDA_LIBRARIES}
		${CUDA_CUDA_LIBRARY}
	)
endif ()

target_compile_definitions(OptiXRayTracerPlugin
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	${EVOENGINE_PLUGINS_DEFS}
	)	
target_include_directories(OptiXRayTracerPlugin
	PRIVATE
	${EVOENGINE_SDK_INCLUDES}
	${EVOENGINE_PLUGINS_INCLUDES}
	${RAY_TRACER_INCLUDES}
	)

if (WIN32)
	set(CUDA_PROPAGATE_HOST_FLAGS ON)
endif ()

set(EVOENGINE_PLUGINS_INCLUDES ${EVOENGINE_PLUGINS_INCLUDES} ${RAY_TRACER_INCLUDES} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_DEFS ${EVOENGINE_PLUGINS_DEFS} OPTIX_RAY_TRACER_PLUGIN PARENT_SCOPE)
