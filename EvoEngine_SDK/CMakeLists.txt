# ------------------------------------------------------------------
# EvoEngine SDK - code
# ------------------------------------------------------------------

set(EVOENGINE_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(EVOENGINE_SDK_DIR ${EVOENGINE_SDK_DIR} PARENT_SCOPE)

file(GLOB_RECURSE EVOENGINE_SDK_HEADERS ${EVOENGINE_SDK_DIR}/include/*.hpp)
file(GLOB_RECURSE EVOENGINE_SDK_SOURCES ${EVOENGINE_SDK_DIR}/src/*.cpp)
set(EVOENGINE_MIKKTSPACE_SOURCES
	${EVOENGINE_SDK_DIR}/src/ThirdParty/MikkTSpace/mikktspace.c
	${EVOENGINE_SDK_DIR}/src/ThirdParty/MikkTSpace/mikktspace.h)
set_source_files_properties(${EVOENGINE_MIKKTSPACE_SOURCES} PROPERTIES SKIP_PRECOMPILE_HEADERS ON)

set(EVOENGINE_SDK_LIBRARY_TYPE STATIC)
if (EVOENGINE_ENABLE_RUNTIME_PACKAGES)
	set(EVOENGINE_SDK_LIBRARY_TYPE SHARED)
endif()

add_library(EvoEngine_SDK
	${EVOENGINE_SDK_LIBRARY_TYPE}
	${IMGUI_SOURCES}
	${EVOENGINE_SDK_HEADERS}
	${EVOENGINE_SDK_SOURCES}
	${EVOENGINE_MIKKTSPACE_SOURCES}
	)

if (WIN32 AND EVOENGINE_ENABLE_RUNTIME_PACKAGES)
	set(EVOENGINE_SDK_WINDOWS_EXPORTS_DEF
		"${CMAKE_CURRENT_BINARY_DIR}/EvoEngine_SDK.dir/$<CONFIG>/EvoEngine_SDK.filtered.def")
	set(EVOENGINE_SDK_WINDOWS_EXPORTS_OBJECTS
		"${CMAKE_CURRENT_BINARY_DIR}/EvoEngine_SDK.dir/$<CONFIG>/EvoEngine_SDK.objects.txt")
	file(GENERATE
		OUTPUT "${EVOENGINE_SDK_WINDOWS_EXPORTS_OBJECTS}"
		CONTENT "$<JOIN:$<TARGET_OBJECTS:EvoEngine_SDK>,\n>\n")
	add_custom_command(TARGET EvoEngine_SDK PRE_LINK
		COMMAND ${CMAKE_COMMAND}
			-E __create_def
			"${EVOENGINE_SDK_WINDOWS_EXPORTS_DEF}"
			"${EVOENGINE_SDK_WINDOWS_EXPORTS_OBJECTS}"
		COMMAND ${CMAKE_COMMAND}
			"-DINPUT=${EVOENGINE_SDK_WINDOWS_EXPORTS_DEF}"
			-P "${CMAKE_SOURCE_DIR}/cmake/EvoEngineFilterWindowsExports.cmake"
		VERBATIM)
	target_link_options(EvoEngine_SDK PRIVATE "/DEF:${EVOENGINE_SDK_WINDOWS_EXPORTS_DEF}")
endif()

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
	${EVOENGINE_SDK_DIR}/include/Utilities/ProceduralNoise
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
	${3RDPARTY_DEFS}
	VULKAN_HPP_TYPESAFE_CONVERSION
	EVOENGINE_INTERNALS_FOLDER=\"${EVOENGINE_SDK_INTERNALS_DIR}\")
	
set(EVOENGINE_SDK_INTERNALS_DIR ${EVOENGINE_SDK_INTERNALS_DIR} PARENT_SCOPE)
set(EVOENGINE_SDK_INTERNALS_DEF ${EVOENGINE_SDK_INTERNALS_DEF} PARENT_SCOPE)

target_compile_definitions(EvoEngine_SDK
	PRIVATE
	${EVOENGINE_SDK_INTERNALS_DEF}
	${DEBUG_STATUS}
	)

if (EVOENGINE_ENABLE_RUNTIME_PACKAGES)
	target_compile_definitions(EvoEngine_SDK
		PRIVATE
		EVOENGINE_SDK_EXPORTS
		EVOENGINE_RUNTIME_PACKAGES
		)
	if (WIN32)
		target_compile_definitions(EvoEngine_SDK PRIVATE
			"IMGUI_API=__declspec(dllexport)"
			"TINYPLY_API=__declspec(dllexport)"
			)
	endif()
endif()

if (WIN32)
	# Compiler settings for Windows platform
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
		pthread
		)
endif ()

# ------------------------------------------------------------------
# EvoEngine SDK - compiler
# ------------------------------------------------------------------
if (CMAKE_CXX_COMPILER_ID STREQUAL GNU)
	message(STATUS "Setting G++ flags")
	# G++
	target_compile_options(EvoEngine_SDK PRIVATE -Werror)
elseif (MSVC)
	message(STATUS "Setting MSVC flags")
	# MSVC
	target_compile_options(EvoEngine_SDK PRIVATE /EHsc /W2 /FS /c)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL Clang)
	message(STATUS "Setting Clang flags")
	# Activate all clang warnings except those that don't really affect much
	target_compile_options(EvoEngine_SDK PRIVATE -fPIC --no-warnings -Wno-c++98-compat -Wno-c++98-compat-pedantic -Wno-newline-eof -Wno-padded -Wno-exit-time-destructors -Wno-global-constructors -Wno-constant-conversion)
endif ()

set(EVOENGINE_SDK_DEFS NOMINMAX)
if (EVOENGINE_ENABLE_RUNTIME_PACKAGES)
	list(APPEND EVOENGINE_SDK_DEFS EVOENGINE_RUNTIME_PACKAGES)
endif()
set(EVOENGINE_SDK_DEFS ${EVOENGINE_SDK_DEFS} PARENT_SCOPE)

set(EVOENGINE_SDK_RUNTIME_LIBRARIES "")
set(EVOENGINE_SDK_PDBS "")
if (EVOENGINE_ENABLE_RUNTIME_PACKAGES)
	set(EVOENGINE_SDK_RUNTIME_LIBRARIES "$<TARGET_FILE:EvoEngine_SDK>")
	if (MSVC)
		set(EVOENGINE_SDK_PDBS "$<TARGET_PDB_FILE:EvoEngine_SDK>")
	endif()
endif()
set(EVOENGINE_SDK_RUNTIME_LIBRARIES ${EVOENGINE_SDK_RUNTIME_LIBRARIES} PARENT_SCOPE)
set(EVOENGINE_SDK_PDBS ${EVOENGINE_SDK_PDBS} PARENT_SCOPE)

# ------------------------------------------------------------------
# EvoEngine SDK - Copy resources
# ------------------------------------------------------------------
function(evoengine_sdk_copy_resources dst_target dst_dir)
	add_custom_command(TARGET ${dst_target} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory
		${EVOENGINE_SDK_DIR}/Internals ${dst_dir}
		VERBATIM)
endfunction()
