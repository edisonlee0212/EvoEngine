set(LSYSTEM_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set(LSYSTEM_INCLUDES
	${LSYSTEM_DIRECTORY}
	${LSYSTEM_DIRECTORY}/include
	)
set(LSYSTEM_INCLUDES ${LSYSTEM_INCLUDES} PARENT_SCOPE)
file(GLOB_RECURSE LSYSTEM_HEADERS CONFIGURE_DEPENDS ${LSYSTEM_DIRECTORY}/include/*.hpp)
file(GLOB_RECURSE LSYSTEM_SOURCES CONFIGURE_DEPENDS ${LSYSTEM_DIRECTORY}/src/*.cpp)
list(APPEND LSYSTEM_SOURCES ${LSYSTEM_DIRECTORY}/src/ParamSpaceExplorer.cpp)
list(REMOVE_DUPLICATES LSYSTEM_SOURCES)

# ---------------------------------------------------------------------------
# GPU pipeline migration flag (Phase 0 of real-time L-system plan).
# Off by default: the CPU pipeline remains the only build path until the
# GPU implementation lands. Turning this on does NOT yet change runtime
# behavior; it only compiles in the gpu/ subtree and exposes the
# LSYSTEM_GPU_PIPELINE preprocessor symbol so call-sites can branch as
# subsequent phases are implemented.
# ---------------------------------------------------------------------------
option(LSYSTEM_GPU_PIPELINE "Enable GPU-resident L-system derivation, growth, and mesh-shader emission." OFF)

add_library(LSystemPlugin
	STATIC
	${LSYSTEM_HEADERS}
	${LSYSTEM_SOURCES}
	)
target_compile_definitions(LSystemPlugin
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	${EVOENGINE_PLUGINS_DEFS}
	LSYSTEM_PLUGIN
	)
if(LSYSTEM_GPU_PIPELINE)
	target_compile_definitions(LSystemPlugin PUBLIC LSYSTEM_GPU_PIPELINE)
endif()
target_include_directories(LSystemPlugin
	PRIVATE
	${EVOENGINE_SDK_INCLUDES}
	${EVOENGINE_PLUGINS_INCLUDES}
	${LSYSTEM_INCLUDES}
	)
target_link_libraries(LSystemPlugin
	EvoEngine_SDK
	${EvoEngine_Plugins}
	)
set(LSYSTEM_PCH ${LSYSTEM_DIRECTORY}/include/LSystem_PCH.hpp)
set(LSYSTEM_PCH ${LSYSTEM_PCH} PARENT_SCOPE)

target_precompile_headers(LSystemPlugin
	PRIVATE
	${EVOENGINE_SDK_PCH}
	${EVOENGINE_PLUGINS_PCHS}
	${LSYSTEM_PCH}
	)

set(EVOENGINE_PLUGINS_INCLUDES ${EVOENGINE_PLUGINS_INCLUDES} ${LSYSTEM_INCLUDES} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_PCHS ${EVOENGINE_PLUGINS_PCHS} ${LSYSTEM_PCH} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_DEFS ${EVOENGINE_PLUGINS_DEFS} LSYSTEM_PLUGIN PARENT_SCOPE)
set(EVOENGINE_PLUGINS_RESOURCES ${EVOENGINE_PLUGINS_RESOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/Internals PARENT_SCOPE)
