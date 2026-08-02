if(NOT DEFINED ROOT)
  message(FATAL_ERROR "ROOT is required")
endif()

set(_evoengine_relocated_ecosyslab_glsl
  Cameras.glsl
  DDGI.glsl
  DDGIGather.glsl
  Environment.glsl
  GltfMaterial.glsl
  GltfRasterMaterial.glsl
  Instances.glsl
  Kernel.glsl
  Lighting.glsl
  Lights.glsl
  Math.glsl
  Noise.glsl
  PerFrame.glsl
  RenderInfo.glsl
  Textures.glsl
  VogelDisk.glsl)

foreach(_header IN LISTS _evoengine_relocated_ecosyslab_glsl)
  file(REMOVE "${ROOT}/DefaultResources/Shaders/Includes/${_header}")
endforeach()
