set(LSYSTEM_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

set(LSYSTEM_INCLUDES
	${LSYSTEM_DIRECTORY}/include
	${CMAKE_CURRENT_SOURCE_DIR}/../include
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
	${EVOENGINE_SERVICES_DEFS}
	)

target_include_directories(LSystemPackage
	PUBLIC
	${EVOENGINE_SDK_INCLUDES}
	${LSYSTEM_INCLUDES}
	PRIVATE
	${EVOENGINE_SERVICES_INCLUDES}
	)

target_link_libraries(LSystemPackage
	PRIVATE
	EvoEngine_SDK
	${EvoEngine_Services}
	)

set(LSYSTEM_PCH ${LSYSTEM_DIRECTORY}/include/LSystem_PCH.hpp)
evoengine_configure_runtime_package(LSystemPackage
	EXTRA_PCH ${LSYSTEM_PCH}
	RESOURCES ${LSYSTEM_DIRECTORY}/Internals
	)
