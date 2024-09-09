set(GPR_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

set(GPR_INCLUDES
	${GPR_DIRECTORY}/include
	${3RDPARTY_DIR}/gpr/gpr/source/lib
	${3RDPARTY_DIR}/gpr/gpr/source/lib/common/public
	${3RDPARTY_DIR}/gpr/gpr/source/lib/gpr_sdk/public
)

file(GLOB GPR_SOURCES ${GPR_DIRECTORY}/src/*.cpp)

add_library(GPRPlugin
	STATIC
	${GPR_SOURCES}
	)
target_compile_definitions(GPRPlugin
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	${EVOENGINE_PLUGINS_DEFS}
	)
target_include_directories(GPRPlugin
	PRIVATE
	${EVOENGINE_SDK_INCLUDES}
	${EVOENGINE_PLUGINS_INCLUDES}
	${GPR_INCLUDES}
	)

target_link_libraries(GPRPlugin
	gpr_sdk
)

set(GPR_PCH ${GPR_DIRECTORY}/include/GPR_PCH.hpp)
set(GPR_PCH ${GPR_PCH} PARENT_SCOPE)

target_precompile_headers(GPRPlugin
	PRIVATE
	${EVOENGINE_SDK_PCH}
	${EVOENGINE_PLUGINS_PCHS}
	${GPR_PCH}
	)

set(EVOENGINE_PLUGINS_INCLUDES ${EVOENGINE_PLUGINS_INCLUDES} ${GPR_INCLUDES} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_PCHS ${EVOENGINE_PLUGINS_PCHS} ${GPR_PCH} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_DEFS ${EVOENGINE_PLUGINS_DEFS} GPR_PLUGIN PARENT_SCOPE)

set(EvoEngine_Plugins ${EvoEngine_Plugins} GPRPlugin PARENT_SCOPE)

set_target_properties(GPRPlugin PROPERTIES FOLDER "Plugins")