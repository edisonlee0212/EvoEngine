message("====Checking Python Binding====")

add_custom_target(PyResourceCopy)
add_dependencies(PyResourceCopy EvoEngine_SDK)

evoengine_copy_resources(PyResourceCopy ${CMAKE_CURRENT_BINARY_DIR})
evoengine_clear_shader_binaries(PyResourceCopy)
add_custom_command(TARGET PyResourceCopy POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy 
			${CMAKE_CURRENT_SOURCE_DIR}/imgui.ini ${CMAKE_CURRENT_BINARY_DIR} VERBATIM)

add_custom_command(TARGET PyResourceCopy POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy 
			${CMAKE_CURRENT_SOURCE_DIR}/generate_sorghum_data.py ${CMAKE_CURRENT_BINARY_DIR} VERBATIM)

set_property(TARGET PyResourceCopy PROPERTY FOLDER PostBuildActions)


function(register_python_binding target_name default_enable)
	if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
		set(option_name PythonBinding-${target_name})
		option(option_name "Add Python binding for ${target_name}" default_enable)
		if(option_name)
			pybind11_add_module(${target_name} ${CMAKE_CURRENT_SOURCE_DIR}/src/${target_name}.cpp)
			target_compile_definitions(${target_name}
				PRIVATE
				${DEFAULT_PROJECT_DEF}
				${EVOENGINE_DEFS}
				)
			target_include_directories(${target_name}
				PRIVATE
				${EVOENGINE_INCLUDES}
				${CMAKE_CURRENT_SOURCE_DIR}/include
				)
			target_precompile_headers(${target_name}
				PRIVATE
				${EVOENGINE_PCHS}
				)
			target_link_libraries(${target_name}
				PRIVATE
				EvoEngine_SDK
				${EvoEngine_Plugins}
				)

			if (CMAKE_CXX_COMPILER_ID STREQUAL GNU)
				target_compile_options(${target_name} PRIVATE -Werror)
			elseif (CMAKE_CXX_COMPILER_ID STREQUAL MSVC)
				target_compile_options(${target_name} PRIVATE /EHsc /W2 /c)
			elseif (CMAKE_CXX_COMPILER_ID STREQUAL Clang)
				target_compile_options(${target_name} PRIVATE -fPIC --no-warnings -Wno-c++98-compat -Wno-c++98-compat-pedantic -Wno-newline-eof -Wno-padded -Wno-exit-time-destructors -Wno-global-constructors -Wno-constant-conversion)
			endif ()

			add_dependencies(${target_name} PyResourceCopy)
			set_property(TARGET ${target_name} PROPERTY FOLDER PythonBindings)
			message("Enabled [${target_name}] PythonBinding.")
		endif()
	endif ()
endfunction()

register_python_binding(PyEvoEngine ON)
register_python_binding(PyEcoSysLab ON)
register_python_binding(PyDigitalAgriculture ON)