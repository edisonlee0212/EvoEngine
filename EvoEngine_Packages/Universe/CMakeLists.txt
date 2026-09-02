set(UNIVERSE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

set(UNIVERSE_INCLUDES
	${UNIVERSE_DIRECTORY}/include
	)

file(GLOB_RECURSE UNIVERSE_HEADERS ${UNIVERSE_DIRECTORY}/include/*.hpp)
file(GLOB_RECURSE UNIVERSE_SOURCES ${UNIVERSE_DIRECTORY}/src/*.cpp)

add_library(UniversePackage
	SHARED
	${UNIVERSE_HEADERS}
	${UNIVERSE_SOURCES}
	)
target_compile_definitions(UniversePackage
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	)	
target_include_directories(UniversePackage
	PRIVATE
	${EVOENGINE_INCLUDES}
	${UNIVERSE_INCLUDES}
	)

target_link_libraries(UniversePackage
	PRIVATE
	EvoEngine_SDK
	)
evoengine_configure_runtime_package(UniversePackage
	RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/Internals
	)
