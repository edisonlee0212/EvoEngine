message("====Checking Python Binding====")

add_custom_target(PyResourceCopy)
add_dependencies(PyResourceCopy EvoEngine_SDK)

set(EVOENGINE_PYTHON_BUILD_RUNTIME_DIR ${CMAKE_CURRENT_BINARY_DIR})
if(CMAKE_CONFIGURATION_TYPES)
	set(EVOENGINE_PYTHON_BUILD_RUNTIME_DIR ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>)
endif()

evoengine_copy_resources(PyResourceCopy ${EVOENGINE_PYTHON_BUILD_RUNTIME_DIR})
evoengine_clear_shader_binaries(PyResourceCopy ${EVOENGINE_PYTHON_BUILD_RUNTIME_DIR})
add_custom_command(TARGET PyResourceCopy POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy 
			${CMAKE_CURRENT_SOURCE_DIR}/imgui.ini ${EVOENGINE_PYTHON_BUILD_RUNTIME_DIR} VERBATIM)

file(GLOB EVOENGINE_PYTHON_BUILD_SCRIPTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.py)
foreach(script_i IN LISTS EVOENGINE_PYTHON_BUILD_SCRIPTS)
	add_custom_command(TARGET PyResourceCopy POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy 
			${script_i} ${EVOENGINE_PYTHON_BUILD_RUNTIME_DIR} VERBATIM)
endforeach()

set_property(TARGET PyResourceCopy PROPERTY FOLDER PostBuildActions)

function(configure_python_binding_target target_name)
	target_compile_definitions(${target_name}
		PRIVATE
		${DEFAULT_PROJECT_DEF}
		${EVOENGINE_DEFS}
		)
	target_include_directories(${target_name}
		PRIVATE
		${EVOENGINE_INCLUDES}
		${CMAKE_CURRENT_SOURCE_DIR}/include
		${EVOENGINE_APP_DIR}/include
		)
	target_precompile_headers(${target_name}
		PRIVATE
		${EVOENGINE_PCHS}
		)
	target_link_libraries(${target_name}
		PRIVATE
		EvoEngine_SDK
		${EvoEngine_Services}
		EvoEngine_DemoScene
		)

	if (CMAKE_CXX_COMPILER_ID STREQUAL GNU)
		target_compile_options(${target_name} PRIVATE -Werror)
	elseif (MSVC)
		target_compile_options(${target_name} PRIVATE /EHsc /W2 /c)
	elseif (CMAKE_CXX_COMPILER_ID STREQUAL Clang)
		target_compile_options(${target_name} PRIVATE -fPIC --no-warnings -Wno-c++98-compat -Wno-c++98-compat-pedantic -Wno-newline-eof -Wno-padded -Wno-exit-time-destructors -Wno-global-constructors -Wno-constant-conversion)
	endif ()

	add_dependencies(${target_name} PyResourceCopy)
	install(TARGETS ${target_name}
		RUNTIME DESTINATION "${EVOENGINE_INSTALL_PYTHON_RUNTIME_DIR}"
		LIBRARY DESTINATION "${EVOENGINE_INSTALL_PYTHON_RUNTIME_DIR}")
	evoengine_install_target_pdb(${target_name} "${EVOENGINE_INSTALL_PYTHON_RUNTIME_DIR}")
	set_property(TARGET ${target_name} PROPERTY FOLDER PythonBindings)
endfunction()

function(register_core_python_binding)
	if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
		set(option_name PythonBinding-PyEvoEngine)
		option(${option_name} "Add core Python binding for EvoEngine" ON)
		if(${option_name})
			pybind11_add_module(PyEvoEngine
				${CMAKE_CURRENT_SOURCE_DIR}/src/PyEvoEngine.cpp
				${CMAKE_CURRENT_SOURCE_DIR}/src/PyEvoEngineModule.cpp)
			configure_python_binding_target(PyEvoEngine)
			message("Enabled [PyEvoEngine] PythonBinding.")
		endif()
	endif ()
endfunction()

function(register_python_binding target_name default_enable)
	if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
		set(option_name PythonBinding-${target_name})
		option(${option_name} "Add Python binding for ${target_name}" ${default_enable})
		if(${option_name})
			pybind11_add_module(${target_name} 
				${CMAKE_CURRENT_SOURCE_DIR}/src/PyEvoEngine.cpp
				${CMAKE_CURRENT_SOURCE_DIR}/src/${target_name}.cpp
				${CMAKE_CURRENT_SOURCE_DIR}/src/${target_name}Module.cpp)
			configure_python_binding_target(${target_name})
			message("Enabled [${target_name}] PythonBinding.")
		endif()
	endif ()
endfunction()

register_core_python_binding()

set("PythonBinding-PyEcoSysLab" OFF CACHE BOOL "Package-specific Python bindings are disabled during runtime package migration." FORCE)
set("PythonBinding-PyDigitalAgriculture" OFF CACHE BOOL
	"Package-specific Python bindings are disabled during runtime package migration." FORCE)
register_python_binding(PyEcoSysLab OFF)
register_python_binding(PyDigitalAgriculture OFF)

if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
	evoengine_install_runtime_payload("${EVOENGINE_INSTALL_PYTHON_RUNTIME_DIR}")
	install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/imgui.ini
		DESTINATION "${EVOENGINE_INSTALL_PYTHON_RUNTIME_DIR}")
	file(GLOB EVOENGINE_PYTHON_SCRIPTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.py)
	if (EVOENGINE_PYTHON_SCRIPTS)
		install(FILES ${EVOENGINE_PYTHON_SCRIPTS}
			DESTINATION "${EVOENGINE_INSTALL_PYTHON_RUNTIME_DIR}")
	endif()
endif ()
