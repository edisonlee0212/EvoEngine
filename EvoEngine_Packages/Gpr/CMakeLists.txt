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

add_library(GprPackage
	SHARED
	${GPR_HEADERS}
	${GPR_SOURCES}
	)
target_compile_definitions(GprPackage
	PUBLIC
	GPR_PACKAGE
	"-DGPR_READING=1"
	"-DGPR_WRITING=1"
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	)
target_include_directories(GprPackage
	PUBLIC
	${GPR_INCLUDES}
	PRIVATE
	${EVOENGINE_INCLUDES}
	)

target_link_libraries(GprPackage
	PRIVATE
	EvoEngine_SDK
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

evoengine_configure_runtime_package(GprPackage)
