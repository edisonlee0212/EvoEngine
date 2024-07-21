set(EvoEngine_Plugins "")

set(EVOENGINE_PLUGINS_INCLUDES "")
set(EVOENGINE_PLUGINS_PCHS "")
set(EVOENGINE_PLUGINS_DEFS "")
set(EVOENGINE_PLUGINS_DLLS "")

if (WIN32)
	option(EVOENGINE_PLUGIN_PHYSICS "Build the physics plugin" ON)
	if(EVOENGINE_PLUGIN_PHYSICS)
	  add_subdirectory(Physics)
	endif()
endif()

option(EVOENGINE_PLUGIN_CPU_RAY_TRACER "Build the cpu ray tracer plugin" ON)
if(EVOENGINE_PLUGIN_CPU_RAY_TRACER)
  add_subdirectory(CpuRayTracer)
endif()

option(EVOENGINE_PLUGIN_MESH_REPAIR "Build the mesh repair plugin" ON)
if(EVOENGINE_PLUGIN_CPU_RAY_TRACER AND EVOENGINE_PLUGIN_MESH_REPAIR)
  add_subdirectory(MeshRepair)
endif()

option(EVOENGINE_PLUGIN_TEXTURE_BAKING "Build the texture baking plugin" ON)
if(EVOENGINE_PLUGIN_CPU_RAY_TRACER AND EVOENGINE_PLUGIN_TEXTURE_BAKING)
  add_subdirectory(TextureBaking)
endif()

option(EVOENGINE_PLUGIN_OPTIX_RAY_TRACER "Build the OptiX ray tracer plugin" ON)
if(EVOENGINE_PLUGIN_OPTIX_RAY_TRACER)
  add_subdirectory(OptixRayTracer)
endif()

option(EVOENGINE_PLUGIN_BILLBOARD_CLOUDS "Build the billboard clouds plugin" ON)
if(EVOENGINE_PLUGIN_BILLBOARD_CLOUDS)
  add_subdirectory(BillboardClouds)
endif()

option(EVOENGINE_PLUGIN_ECOSYSLAB "Build the ecosyslab plugin" ON)
if(EVOENGINE_PLUGIN_ECOSYSLAB AND EVOENGINE_PLUGIN_BILLBOARD_CLOUDS)
  add_subdirectory(EcoSysLab)
endif()

set(EvoEngine_Plugins ${EvoEngine_Plugins} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_INCLUDES ${EVOENGINE_PLUGINS_INCLUDES} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_PCHS ${EVOENGINE_PLUGINS_PCHS} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_DEFS ${EVOENGINE_PLUGINS_DEFS} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_DLLS ${EVOENGINE_PLUGINS_DLLS} PARENT_SCOPE)
