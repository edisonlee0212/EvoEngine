set(TEXTURE_BAKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

set(TEXTURE_BAKING_INCLUDES
	${TEXTURE_BAKING_DIRECTORY}/include
	)

file(GLOB_RECURSE TEXTURE_BAKING_HEADERS ${TEXTURE_BAKING_DIRECTORY}/include/*.hpp)
file(GLOB_RECURSE TEXTURE_BAKING_SOURCES ${TEXTURE_BAKING_DIRECTORY}/src/*.cpp)

add_library(TextureBakingPackage
	SHARED
	${TEXTURE_BAKING_HEADERS}
	${TEXTURE_BAKING_SOURCES}
	)
target_compile_definitions(TextureBakingPackage
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	)	
target_include_directories(TextureBakingPackage
	PRIVATE
	${EVOENGINE_INCLUDES}
	${TEXTURE_BAKING_INCLUDES}
	)

target_link_libraries(TextureBakingPackage
	PRIVATE
	EvoEngine_SDK
	)
evoengine_configure_runtime_package(TextureBakingPackage)
