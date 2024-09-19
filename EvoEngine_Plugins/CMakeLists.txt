message("======Checking Plugins=======")
set(EvoEngine_Plugins "")

set(EVOENGINE_PLUGINS_INCLUDES "")
set(EVOENGINE_PLUGINS_PCHS "")
set(EVOENGINE_PLUGINS_DEFS "")
set(EVOENGINE_PLUGINS_DLLS "")
set(EVOENGINE_PLUGINS_RESOURCES "")


function(register_evoengine_plugin plugin_name default_enable)
  set(option_name EvoEngine_Plugin-${plugin_name})
  option(option_name "Enable ${plugin_name} plugin." ${default_enable}) 
  if(option_name AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${plugin_name})
	add_subdirectory(${plugin_name})
	set(EvoEngine_Plugins ${EvoEngine_Plugins} ${plugin_name}Plugin)
	if(TARGET ${plugin_name}Plugin)
		message("Loaded [${plugin_name}] plugin.")
		set_target_properties(${plugin_name}Plugin PROPERTIES FOLDER "Plugins")
	else()
		message("[${plugin_name}] plugin not loaded.")
	endif()
  endif()

  set(EvoEngine_Plugins ${EvoEngine_Plugins} PARENT_SCOPE)
  set(EVOENGINE_PLUGINS_INCLUDES ${EVOENGINE_PLUGINS_INCLUDES} PARENT_SCOPE)
  set(EVOENGINE_PLUGINS_PCHS ${EVOENGINE_PLUGINS_PCHS} PARENT_SCOPE)
  set(EVOENGINE_PLUGINS_DEFS ${EVOENGINE_PLUGINS_DEFS} PARENT_SCOPE)
  set(EVOENGINE_PLUGINS_DLLS ${EVOENGINE_PLUGINS_DLLS} PARENT_SCOPE)
  set(EVOENGINE_PLUGINS_RESOURCES ${EVOENGINE_PLUGINS_RESOURCES} PARENT_SCOPE)
endfunction()



register_evoengine_plugin(MeshRepair ON)
register_evoengine_plugin(TextureBaking ON)
register_evoengine_plugin(OptiXRayTracer ON)
register_evoengine_plugin(BillboardClouds ON)
register_evoengine_plugin(EcoSysLab ON)
register_evoengine_plugin(DigitalAgriculture ON)
register_evoengine_plugin(DatasetGeneration ON)
register_evoengine_plugin(LogGrading ON)
register_evoengine_plugin(LogScanning ON)
register_evoengine_plugin(Universe ON)
register_evoengine_plugin(Gpr ON)

#file(GLOB plugin_subdirectory_candidates RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
#    ${CMAKE_CURRENT_SOURCE_DIR}/*
#)

#foreach(plugin_subdirectory ${plugin_subdirectory_candidates})
#    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${plugin_subdirectory} AND NOT DEFINED ${plugin_subdirectory})
#       register_evoengine_plugin(${plugin_subdirectory} ON)
#   endif()
#endforeach()

set(EvoEngine_Plugins ${EvoEngine_Plugins} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_INCLUDES ${EVOENGINE_PLUGINS_INCLUDES} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_PCHS ${EVOENGINE_PLUGINS_PCHS} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_DEFS ${EVOENGINE_PLUGINS_DEFS} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_DLLS ${EVOENGINE_PLUGINS_DLLS} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_RESOURCES ${EVOENGINE_PLUGINS_RESOURCES} PARENT_SCOPE)

