message("====Checking Python Binding====")

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
			message("Enabled [${target_name}] PythonBinding.")
		endif()
	endif ()
endfunction()

register_python_binding(PyEvoEngine ON)
register_python_binding(PyEcoSysLab ON)
register_python_binding(PyDigitalAgriculture ON)