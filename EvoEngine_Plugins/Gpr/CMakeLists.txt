set(GPR_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

set(GPR_INCLUDES
	${GPR_DIRECTORY}/include
	${3RDPARTY_DIR}/gpr/gpr/source/lib
	${3RDPARTY_DIR}/gpr/gpr/source/lib/common/public
	${3RDPARTY_DIR}/gpr/gpr/source/lib/vc5_common
	${3RDPARTY_DIR}/gpr/gpr/source/lib/vc5_decoder
	${3RDPARTY_DIR}/gpr/gpr/source/lib/vc5_encoder
	${3RDPARTY_DIR}/gpr/gpr/source/lib/dng_sdk
	${3RDPARTY_DIR}/gpr/gpr/source/lib/gpr_sdk/public
)
file(GLOB_RECURSE GPR_HEADERS ${GPR_DIRECTORY}/include/*.hpp)
file(GLOB_RECURSE GPR_SOURCES ${GPR_DIRECTORY}/src/*.cpp)

add_library(GprPlugin
	STATIC
	${GPR_HEADERS}
	${GPR_SOURCES}
	)
target_compile_definitions(GprPlugin
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	${EVOENGINE_PLUGINS_DEFS}
	"-DGPR_READING=1"
	"-DGPR_WRITING=1"
	)
target_include_directories(GprPlugin
	PRIVATE
	${EVOENGINE_SDK_INCLUDES}
	${EVOENGINE_PLUGINS_INCLUDES}
	${GPR_INCLUDES}
	)

target_link_libraries(GprPlugin
	common
	vc5_common
	vc5_decoder
	vc5_encoder
	dng_sdk
	gpr_sdk
	xmp_core
	expat_lib
	md5_lib
)


set(GPR_PCH ${GPR_DIRECTORY}/include/GPR_PCH.hpp)
set(GPR_PCH ${GPR_PCH} PARENT_SCOPE)

target_precompile_headers(GprPlugin
	PRIVATE
	${EVOENGINE_SDK_PCH}
	${EVOENGINE_PLUGINS_PCHS}
	${GPR_PCH}
	)

set(EVOENGINE_PLUGINS_INCLUDES ${EVOENGINE_PLUGINS_INCLUDES} ${GPR_INCLUDES} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_PCHS ${EVOENGINE_PLUGINS_PCHS} ${GPR_PCH} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_DEFS 
	${EVOENGINE_PLUGINS_DEFS} 
	GPR_PLUGIN
	"-DGPR_READING=1"
	"-DGPR_WRITING=1"
	PARENT_SCOPE)
