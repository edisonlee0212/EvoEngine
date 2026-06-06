set(LSYSTEM_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

set(LSYSTEM_INCLUDES
	${LSYSTEM_DIRECTORY}/include
	)

file(GLOB_RECURSE LSYSTEM_HEADERS ${LSYSTEM_DIRECTORY}/include/*.hpp)
file(GLOB_RECURSE LSYSTEM_SOURCES ${LSYSTEM_DIRECTORY}/src/*.cpp)

add_library(LSystemPackage
	SHARED
	${LSYSTEM_HEADERS}
	${LSYSTEM_SOURCES}
	)
target_compile_definitions(LSystemPackage
	PUBLIC
	LSYSTEM_PACKAGE
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	)
target_include_directories(LSystemPackage
	PUBLIC
	${EVOENGINE_INCLUDES}
	${LSYSTEM_INCLUDES}
	)
target_link_libraries(LSystemPackage
	PRIVATE
	EvoEngine_SDK
	)

set(LSYSTEM_PCH ${LSYSTEM_DIRECTORY}/include/LSystem_PCH.hpp)
evoengine_configure_runtime_package(LSystemPackage
	EXTRA_PCH ${LSYSTEM_PCH}
	)
