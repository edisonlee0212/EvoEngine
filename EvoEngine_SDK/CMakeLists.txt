# ------------------------------------------------------------------
# EvoEngine SDK - code
# ------------------------------------------------------------------

set(EVOENGINE_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(EVOENGINE_SDK_DIR ${EVOENGINE_SDK_DIR} PARENT_SCOPE)

file(GLOB_RECURSE EVOENGINE_SDK_SOURCES ${EVOENGINE_SDK_DIR}/src/*.cpp)
add_library(EvoEngine_SDK
	STATIC
	${IMGUI_SOURCES}
	${EVOENGINE_SDK_SOURCES}
	)

set(EVOENGINE_SDK_INCLUDES
	${3RDPARTY_INCLUDES}
	${EVOENGINE_SDK_DIR}/include
	${EVOENGINE_SDK_DIR}/include/Core
	${EVOENGINE_SDK_DIR}/include/Core/ECS
	${EVOENGINE_SDK_DIR}/include/Core/Math
	${EVOENGINE_SDK_DIR}/include/Layers
	${EVOENGINE_SDK_DIR}/include/Assets
	${EVOENGINE_SDK_DIR}/include/Physics
	${EVOENGINE_SDK_DIR}/include/RayTracing
	${EVOENGINE_SDK_DIR}/include/Rendering
	${EVOENGINE_SDK_DIR}/include/Rendering/RenderInstances
	${EVOENGINE_SDK_DIR}/include/Rendering/PBR
	${EVOENGINE_SDK_DIR}/include/Rendering/Animation
	${EVOENGINE_SDK_DIR}/include/Rendering/Renderer
	${EVOENGINE_SDK_DIR}/include/Rendering/Geometry
	${EVOENGINE_SDK_DIR}/include/Rendering/Texture
	${EVOENGINE_SDK_DIR}/include/Rendering/Platform
	${EVOENGINE_SDK_DIR}/include/Rendering/PostProcessing
	${EVOENGINE_SDK_DIR}/include/Editor
	${EVOENGINE_SDK_DIR}/include/Utilities
	${EVOENGINE_SDK_DIR}/include/Utilities/NsightAftermath
	${CMAKE_CURRENT_BINARY_DIR}
	)
set(EVOENGINE_SDK_INCLUDES ${EVOENGINE_SDK_INCLUDES} PARENT_SCOPE)

target_include_directories(EvoEngine_SDK
	PRIVATE
	${EVOENGINE_SDK_INCLUDES}
	)

set(EVOENGINE_SDK_PCH ${EVOENGINE_SDK_DIR}/include/EvoEngine_SDK_PCH.hpp)
set(EVOENGINE_SDK_PCH ${EVOENGINE_SDK_PCH} PARENT_SCOPE)

target_precompile_headers(EvoEngine_SDK
	PRIVATE
	${EVOENGINE_SDK_PCH}
	)

# ------------------------------------------------------------------
# EvoEngine SDK - linkings
# ------------------------------------------------------------------
target_link_libraries(EvoEngine_SDK
	${3RDPARTY_LIBS}
	)

set(EVOENGINE_SDK_INTERNALS_DIR ${CMAKE_CURRENT_BINARY_DIR}/Internals/)
set(EVOENGINE_SDK_INTERNALS_DEF 
	VULKAN_HPP_TYPESAFE_CONVERSION
	EVOENGINE_INTERNALS_FOLDER=\"${EVOENGINE_SDK_INTERNALS_DIR}\")
	
set(EVOENGINE_SDK_INTERNALS_DIR ${EVOENGINE_SDK_INTERNALS_DIR} PARENT_SCOPE)
set(EVOENGINE_SDK_INTERNALS_DEF ${EVOENGINE_SDK_INTERNALS_DEF} PARENT_SCOPE)

target_compile_definitions(EvoEngine_SDK
	PRIVATE
	${EVOENGINE_SDK_INTERNALS_DEF}
	${DEBUG_STATUS}
	)

if (WIN32)
	# Compiler settings for Windows platform
	set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
	target_compile_definitions(EvoEngine_SDK
		PRIVATE
		NOMINMAX
		USE_ASSIMP
		)
else()
	target_link_libraries(EvoEngine_SDK
		m
		${Vulkan_LIBRARIES}
		stdc++
		dl
		Xau
		X11
		pthread
		Xrandr
		Xi
		)
endif ()

# ------------------------------------------------------------------
# EvoEngine SDK - compiler
# ------------------------------------------------------------------
if (CMAKE_CXX_COMPILER_ID STREQUAL GNU)
	message(STATUS "Setting G++ flags")
	# G++
	target_compile_options(EvoEngine_SDK PRIVATE -Wall -Wextra)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL MSVC)
	message(STATUS "Setting MSVC flags")
	# MSVC
	target_compile_options(EvoEngine_SDK PRIVATE /EHsc /W2 /c)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL Clang)
	message(STATUS "Setting Clang flags")
	# Activate all clang warnings except those that don't really affect much
	target_compile_options(EvoEngine_SDK PRIVATE -fPIC -Weverything -Wno-c++98-compat -Wno-c++98-compat-pedantic -Wno-newline-eof -Wno-padded -Wno-exit-time-destructors -Wno-global-constructors -Wno-constant-conversion)
endif ()

set(EVOENGINE_SDK_DEFS NOMINMAX PARENT_SCOPE)

# ------------------------------------------------------------------
# EvoEngine SDK - Copy resources
# ------------------------------------------------------------------
function(evoengine_sdk_copy_resources dst_target dst_dir)
	add_custom_command(TARGET ${dst_target} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory
		${EVOENGINE_SDK_DIR}/Internals ${dst_dir}
		VERBATIM)
endfunction()