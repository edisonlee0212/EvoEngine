set(MESH_REPAIR_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

set(MESH_REPAIR_INCLUDES
	${MESH_REPAIR_DIRECTORY}/include
	${MESH_REPAIR_DIRECTORY}/include/VisibilityTest
	)

file(GLOB_RECURSE MESH_REPAIR_HEADERS ${MESH_REPAIR_DIRECTORY}/include/*.hpp)
file(GLOB_RECURSE MESH_REPAIR_SOURCES ${MESH_REPAIR_DIRECTORY}/src/*.cpp)

add_library(MeshRepairPackage
	SHARED
	${MESH_REPAIR_HEADERS}
	${MESH_REPAIR_SOURCES}
	)
target_compile_definitions(MeshRepairPackage
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	)	
target_include_directories(MeshRepairPackage
	PRIVATE
	${EVOENGINE_INCLUDES}
	${MESH_REPAIR_INCLUDES}
	)

target_link_libraries(MeshRepairPackage
	PRIVATE
	EvoEngine_SDK
	)
evoengine_configure_runtime_package(MeshRepairPackage)
