message("====Checking Python Binding====")

add_custom_target(PyResourceCopy)
evoengine_copy_resources(PyResourceCopy ${CMAKE_CURRENT_BINARY_DIR})
evoengine_clear_shader_binaries(PyResourceCopy)
add_custom_command(TARGET PyResourceCopy POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy 
			${CMAKE_CURRENT_SOURCE_DIR}/imgui.ini ${CMAKE_CURRENT_BINARY_DIR} VERBATIM)

set_property(TARGET PyResourceCopy PROPERTY FOLDER PostBuildActions)


function(register_python_binding target_name default_enable)
	if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
		set(option_name PythonBinding-${target_name})
		option(option_name "Add Python binding for ${target_name}" default_enable)
		if(option_name)
			pybind11_add_module(${target_name} ${target_name}.cpp)
			target_compile_definitions(${target_name}
				PRIVATE
				${DEFAULT_PROJECT_DEF}
				${EVOENGINE_DEFS}
				)
			target_include_directories(${target_name}
				PRIVATE
				${EVOENGINE_INCLUDES}
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
			add_dependencies(${target_name} PyResourceCopy)
			set_property(TARGET ${target_name} PROPERTY FOLDER PythonBindings)
			message("Enabled [${target_name}] PythonBinding.")
		endif()
	endif ()
endfunction()

register_python_binding(PyEvoEngine ON)
register_python_binding(PyEcoSysLab ON)
register_python_binding(PyDigitalAgriculture ON)