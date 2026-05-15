set(ECOSYSLAB_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set(ECOSYSLAB_INCLUDES
	${3RDPARTY_DIR}/pinchot/pinchot/include
	${ECOSYSLAB_DIRECTORY}
	${ECOSYSLAB_DIRECTORY}/include
	${ECOSYSLAB_DIRECTORY}/include/MeshFormation
	${ECOSYSLAB_DIRECTORY}/include/Physics/DynamicSkeleton
	${ECOSYSLAB_DIRECTORY}/include/Physics/DynamicStrands
	${ECOSYSLAB_DIRECTORY}/include/TreeModel/Descriptors
	${ECOSYSLAB_DIRECTORY}/include/Utilities/SpatialCompetition
	${ECOSYSLAB_DIRECTORY}/include/Physics
	${ECOSYSLAB_DIRECTORY}/include/Utilities/TreeIO
	${ECOSYSLAB_DIRECTORY}/include/Volumes
	${ECOSYSLAB_DIRECTORY}/include/ClimateModel
	${ECOSYSLAB_DIRECTORY}/include/SoilModel
	${ECOSYSLAB_DIRECTORY}/include/TreeModel/StrandModel
	${ECOSYSLAB_DIRECTORY}/include/TreeModel/ShootModel
	${ECOSYSLAB_DIRECTORY}/include/TreeModel/RootModel
	${ECOSYSLAB_DIRECTORY}/include/TreeModel/DynamicTreeStrandGraph
	${ECOSYSLAB_DIRECTORY}/include/TreeModel
	${ECOSYSLAB_DIRECTORY}/include/Utilities
	${ECOSYSLAB_DIRECTORY}/include/Structures
	${ECOSYSLAB_DIRECTORY}/include/Reconstruction
	${ECOSYSLAB_DIRECTORY}/include/VoronoiMeshes
	)
file(GLOB_RECURSE ECOSYSLAB_HEADERS ${ECOSYSLAB_DIRECTORY}/include/*.hpp)
file(GLOB_RECURSE ECOSYSLAB_SOURCES ${ECOSYSLAB_DIRECTORY}/src/*.cpp)

add_library(EcoSysLabPackage
	SHARED
	${ECOSYSLAB_HEADERS}
	${ECOSYSLAB_SOURCES}
	)
target_compile_definitions(EcoSysLabPackage
	PUBLIC
	ECOSYSLAB_PACKAGE
	BILLBOARD_CLOUDS_PACKAGE
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	${EVOENGINE_SERVICES_DEFS}
	)
target_include_directories(EcoSysLabPackage
	PUBLIC
	${EVOENGINE_SDK_INCLUDES}
	${ECOSYSLAB_INCLUDES}
	PRIVATE
	${EVOENGINE_SERVICES_INCLUDES}
	)
target_link_libraries(EcoSysLabPackage
	PUBLIC
	BillboardCloudsPackage
	PRIVATE
	EvoEngine_SDK
	${EvoEngine_Services}
	)

set(ECOSYSLAB_PCH ${ECOSYSLAB_DIRECTORY}/include/EcoSysLab_PCH.hpp)
evoengine_configure_runtime_package(EcoSysLabPackage
	EXTRA_PCH ${ECOSYSLAB_PCH}
	RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/Internals
	)

find_package(CGAL)
if(CGAL_FOUND)
	target_link_libraries(EcoSysLabPackage PRIVATE CGAL::CGAL)
	target_compile_definitions(EcoSysLabPackage PRIVATE USE_CGAL)
	message(STATUS "EcoSysLab Package: CGAL found and linked.")
else()
	message(WARNING "EcoSysLab Package: CGAL not found. Some features may be disabled.")
endif()
