message("====Checking Runtime Packages====")

set(EvoEngine_RuntimePackages "")

function(register_evoengine_runtime_package package_name default_enable)
	set(option_name EVOENGINE_ENABLE_${package_name}_PACKAGE)
	option(${option_name} "Build ${package_name} runtime package." ${default_enable})
	if (${option_name} AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${package_name})
		add_subdirectory(${package_name})
		if (TARGET ${package_name}Package)
			set(target_name ${package_name}Package)
			set_target_properties(${target_name} PROPERTIES
				FOLDER "Runtime Packages"
				RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/EvoEngine_App/Packages"
				LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/EvoEngine_App/Packages"
				PDB_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/EvoEngine_App/Packages")
			foreach(config_name Debug Release RelWithDebInfo MinSizeRel)
				string(TOUPPER ${config_name} config_name_upper)
				set_target_properties(${target_name} PROPERTIES
					RUNTIME_OUTPUT_DIRECTORY_${config_name_upper} "${CMAKE_BINARY_DIR}/EvoEngine_App/${config_name}/Packages"
					LIBRARY_OUTPUT_DIRECTORY_${config_name_upper} "${CMAKE_BINARY_DIR}/EvoEngine_App/${config_name}/Packages"
					PDB_OUTPUT_DIRECTORY_${config_name_upper} "${CMAKE_BINARY_DIR}/EvoEngine_App/${config_name}/Packages")
			endforeach()
			install(TARGETS ${target_name}
				RUNTIME DESTINATION "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}/Packages"
				LIBRARY DESTINATION "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}/Packages")
			evoengine_install_target_pdb(${target_name} "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}/Packages")
			set(EvoEngine_RuntimePackages ${EvoEngine_RuntimePackages} ${target_name})
			message("Enabled [${package_name}] runtime package.")
		else()
			message("[${package_name}] runtime package not loaded.")
		endif()
	endif()
	set(EvoEngine_RuntimePackages ${EvoEngine_RuntimePackages} PARENT_SCOPE)
endfunction()

set(EvoEngine_RuntimePackages ${EvoEngine_RuntimePackages} PARENT_SCOPE)
