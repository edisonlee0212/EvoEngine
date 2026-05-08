set(BILLBOARD_CLOUDS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

set(BILLBOARD_CLOUDS_INCLUDES
	${BILLBOARD_CLOUDS_DIRECTORY}/include
	)
file(GLOB_RECURSE BILLBOARD_CLOUDS_HEADERS ${BILLBOARD_CLOUDS_DIRECTORY}/include/*.hpp)
file(GLOB_RECURSE BILLBOARD_CLOUDS_SOURCES ${BILLBOARD_CLOUDS_DIRECTORY}/src/*.cpp)

add_library(BillboardCloudsPackage
	SHARED
	${BILLBOARD_CLOUDS_HEADERS}
	${BILLBOARD_CLOUDS_SOURCES}
	)
target_compile_definitions(BillboardCloudsPackage
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	)	
target_include_directories(BillboardCloudsPackage
	PRIVATE
	${EVOENGINE_INCLUDES}
	${BILLBOARD_CLOUDS_INCLUDES}
	)

target_link_libraries(BillboardCloudsPackage
	PRIVATE
	EvoEngine_SDK
	)

evoengine_configure_runtime_package(BillboardCloudsPackage)
