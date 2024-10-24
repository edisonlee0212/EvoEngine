set(UNIVERSE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

set(UNIVERSE_INCLUDES
	${UNIVERSE_DIRECTORY}/include
	)

file(GLOB_RECURSE UNIVERSE_HEADERS ${UNIVERSE_DIRECTORY}/include/*.hpp)
file(GLOB_RECURSE UNIVERSE_SOURCES ${UNIVERSE_DIRECTORY}/src/*.cpp)

add_library(UniversePlugin
	STATIC
	${UNIVERSE_HEADERS}
	${UNIVERSE_SOURCES}
	)
target_compile_definitions(UniversePlugin
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	${EVOENGINE_PLUGINS_DEFS}
	)	
target_include_directories(UniversePlugin
	PRIVATE
	${EVOENGINE_SDK_INCLUDES}
	${EVOENGINE_PLUGINS_INCLUDES}
	${UNIVERSE_INCLUDES}
	)

set(UNIVERSE_PCH ${UNIVERSE_DIRECTORY}/include/Universe_PCH.hpp)
set(UNIVERSE_PCH ${UNIVERSE_PCH} PARENT_SCOPE)

target_precompile_headers(UniversePlugin
	PRIVATE
	${EVOENGINE_SDK_PCH}
	${EVOENGINE_PLUGINS_PCHS}
	${UNIVERSE_PCH}
	)

set(EVOENGINE_PLUGINS_INCLUDES ${EVOENGINE_PLUGINS_INCLUDES} ${UNIVERSE_INCLUDES} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_PCHS ${EVOENGINE_PLUGINS_PCHS} ${UNIVERSE_PCH} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_DEFS ${EVOENGINE_PLUGINS_DEFS} UNIVERSE_PLUGIN PARENT_SCOPE)
