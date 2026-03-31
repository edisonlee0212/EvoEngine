if(POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD)
endif()

# Use the older FindCUDA module (deprecated in newer CMake versions)
find_package(CUDA QUIET)

# Check if CUDA is found
if (NOT CUDA_FOUND)
    message(WARNING "CUDA not found. Skipping this Service.")
    return()
endif()

message("Found CUDA.")

set(CMAKE_CUDA_ARCHITECTURES 75-real 75-virtual)
mark_as_advanced(CUDA_SDK_ROOT_DIR)
enable_language(CUDA)


set(RAY_TRACER_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

include(${RAY_TRACER_DIRECTORY}/cmake/configure_optix.cmake)

# Check if OptiX is found
if (NOT OptiX_FOUND)
    message(WARNING "OptiX not found. Skipping this Service.")
    return()
endif()
message("Found OptiX.")

file(GLOB_RECURSE RAY_TRACER_HEADERS
	${RAY_TRACER_DIRECTORY}/src/*.hpp
	)

file(GLOB_RECURSE RAY_TRACER_SOURCES
	${CMAKE_CURRENT_BINARY_DIR}
	${RAY_TRACER_DIRECTORY}/src/*.cpp
	)

file(GLOB_RECURSE RAY_TRACER_CUDA_SOURCES
	${RAY_TRACER_DIRECTORY}/src/*.cu
	)

set(CUDA_MODULE_INCLUDES
	${OptiX_INCLUDE}
	${CUDA_TOOLKIT_ROOT_DIR}/include
	${RAY_TRACER_DIRECTORY}/include
	${RAY_TRACER_DIRECTORY}/include/MLVQ
	${RAY_TRACER_DIRECTORY}/include/RayTracer
	${RAY_TRACER_DIRECTORY}/include/Utilities
	)

include_directories(${EVOENGINE_SDK_INCLUDES})
include_directories(${CUDA_MODULE_INCLUDES})

cuda_compile_and_embed(camera_rendering_ptx ${RAY_TRACER_DIRECTORY}/src/ptx/CameraRendering.cu)
cuda_compile_and_embed(illumination_estimation_ptx ${RAY_TRACER_DIRECTORY}/src/ptx/IlluminationEstimation.cu)
cuda_compile_and_embed(point_cloud_scanning_ptx ${RAY_TRACER_DIRECTORY}/src/ptx/PointCloudScanning.cu)

add_library(CudaModuleService
	STATIC
	#Default
	${camera_rendering_ptx}
	${illumination_estimation_ptx}
	${point_cloud_scanning_ptx}

	#C++
	${RAY_TRACER_HEADERS}
	${RAY_TRACER_SOURCES}
	${RAY_TRACER_CUDA_SOURCES}
	)

set_property(TARGET CudaModuleService PROPERTY CUDA_ARCHITECTURES native)

if (NOT WIN32)
	message(STATUS CUDALIB=${CUDA_TOOLKIT_ROOT_DIR})
	set(CUDA_CUDA_LIBRARY ${CUDA_TOOLKIT_ROOT_DIR}/lib64/stubs)
endif ()

# Linker settings for all platforms
if (NOT WIN32)
	target_link_libraries(CudaModuleService
		EvoEngine_SDK
		${EvoEngine_Services}
		cuda
	)
else()
	target_link_libraries(CudaModuleService
		EvoEngine_SDK
		${EvoEngine_Services}
		${CUDA_LIBRARIES}
		${CUDA_CUDA_LIBRARY}
	)
endif ()

target_compile_definitions(CudaModuleService
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	${EVOENGINE_SERVICES_DEFS}
	)	
target_include_directories(CudaModuleService
	PRIVATE
	${EVOENGINE_SDK_INCLUDES}
	${EVOENGINE_SERVICES_INCLUDES}
	${CUDA_MODULE_INCLUDES}
	)

if (WIN32)
	set(CUDA_PROPAGATE_HOST_FLAGS ON)
endif ()

set(EVOENGINE_SERVICES_INCLUDES ${EVOENGINE_SERVICES_INCLUDES} ${CUDA_MODULE_INCLUDES} PARENT_SCOPE)
set(EVOENGINE_SERVICES_DEFS ${EVOENGINE_SERVICES_DEFS} CUDA_MODULE_SERVICE PARENT_SCOPE)
