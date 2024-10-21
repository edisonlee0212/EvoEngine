set (TARGET_NAME glslang)

set(GLSLANG_TESTS_DEFAULT OFF CACHE BOOL "" FORCE)
set(GLSLANG_ENABLE_INSTALL_DEFAULT OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_EXTERNAL OFF CACHE BOOL "" FORCE)
set(ENABLE_OPT OFF CACHE BOOL "" FORCE)
add_subdirectory(${TARGET_NAME} EXCLUDE_FROM_ALL)

set(3RDPARTY_INCLUDES 
	${3RDPARTY_INCLUDES}
	${CMAKE_CURRENT_SOURCE_DIR}/glslang/SPIRV
	${CMAKE_CURRENT_SOURCE_DIR}/glslang/glslang/Public
	PARENT_SCOPE)

set(3RDPARTY_LIBS ${3RDPARTY_LIBS} 
    ${TARGET_NAME}::${TARGET_NAME}
	${TARGET_NAME}::SPIRV
	${TARGET_NAME}::glslang-default-resource-limits
    PARENT_SCOPE)

set_property(TARGET ${TARGET_NAME} PROPERTY FOLDER 3rdParty/glslang)
set_property(TARGET GenericCodeGen PROPERTY FOLDER 3rdParty/glslang)
set_property(TARGET glslang-default-resource-limits PROPERTY FOLDER 3rdParty/glslang)
set_property(TARGET MachineIndependent PROPERTY FOLDER 3rdParty/glslang)
set_property(TARGET OSDependent PROPERTY FOLDER 3rdParty/glslang)
set_property(TARGET SPIRV PROPERTY FOLDER 3rdParty/glslang)