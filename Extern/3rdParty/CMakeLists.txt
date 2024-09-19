message("===Checking 3rdParty Libs====")
function(GetAllTargets var)
  set(targets)
  GetAllTargetsRecursive(targets ${CMAKE_CURRENT_SOURCE_DIR})
  set(${var} ${targets} PARENT_SCOPE)
endfunction()

macro(GetAllTargetsRecursive targets dir)
  get_property(subdirectories DIRECTORY ${dir} PROPERTY SUBDIRECTORIES)
  foreach(subdir ${subdirectories})
    GetAllTargetsRecursive(${targets} ${subdir})
  endforeach()

  get_property(current_targets DIRECTORY ${dir} PROPERTY BUILDSYSTEM_TARGETS)
  list(APPEND ${targets} ${current_targets})
endmacro()

# Turn off warnings for 3rd party code
if(MSVC)
  foreach(flag_var
	  CMAKE_C_FLAGS CMAKE_CXX_FLAGS)
	string(REPLACE "/W4" "/W0" ${flag_var} "${${flag_var}}")
  endforeach()
else()
  foreach(flag_var
	  CMAKE_C_FLAGS CMAKE_CXX_FLAGS)
	string(REPLACE "-Wall" "-w" ${flag_var} "${${flag_var}}")
  endforeach()
endif()

message("Loading 3rdParty libraries...")

function(message)
    if (NOT 3RDPARTY_SUPPRESS_ALL_MESSAGES)
        _message(${ARGN})
    endif()
endfunction()

set(3RDPARTY_SUPPRESS_ALL_MESSAGES OFF)

add_subdirectory(zlib EXCLUDE_FROM_ALL)

add_subdirectory(assimp EXCLUDE_FROM_ALL)
add_subdirectory(base64 EXCLUDE_FROM_ALL)
add_subdirectory(csvpp EXCLUDE_FROM_ALL)
add_subdirectory(eigen EXCLUDE_FROM_ALL)
add_subdirectory(glfw EXCLUDE_FROM_ALL)
add_subdirectory(glm EXCLUDE_FROM_ALL)
add_subdirectory(imgui EXCLUDE_FROM_ALL)
set(IMGUI_SOURCES ${IMGUI_SOURCES} PARENT_SCOPE)
add_subdirectory(json EXCLUDE_FROM_ALL)
add_subdirectory(glslang EXCLUDE_FROM_ALL)
add_subdirectory(meshoptimizer EXCLUDE_FROM_ALL)
add_subdirectory(pybind11 EXCLUDE_FROM_ALL)
add_subdirectory(stb EXCLUDE_FROM_ALL)
add_subdirectory(volk EXCLUDE_FROM_ALL)
add_subdirectory(VulkanMemoryAllocator EXCLUDE_FROM_ALL)
add_subdirectory(xatlas EXCLUDE_FROM_ALL)
add_subdirectory(yaml-cpp EXCLUDE_FROM_ALL)
add_subdirectory(gpr EXCLUDE_FROM_ALL)


unset(3RDPARTY_SUPPRESS_ALL_MESSAGES)
message("Loaded all 3rdParty libraries.")

# Turn off warnings for 3rd party code, part 1. Some projects set their own warnings by compile options.
# Need to turn them off as well.
GetAllTargets(all_targets)
foreach(target ${all_targets})
  if(MSVC)
    set(remove_flags /Wall /W4 /W3 /W2 /W1)
    set(add_flags /W0)
  else()
      set(remove_flags -Wall -Wextra)
      set(add_flags -w)
  endif()

  foreach(flag_var COMPILE_FLAGS COMPILE_OPTIONS)
    get_target_property(compile_opts ${target} ${flag_var})
    if(compile_opts)
      foreach(warning_flag ${remove_flags})
        list(REMOVE_ITEM compile_opts ${warning_flag})
      endforeach()

      set_target_properties(${target} PROPERTIES ${flag_var} "${compile_opts}")
    endif()
  endforeach()

  get_target_property(target_type ${target} TYPE)
  if ((NOT target_type STREQUAL "INTERFACE_LIBRARY") AND (NOT target_type STREQUAL "UTILITY"))
    target_compile_options(${target} PRIVATE ${add_flags})
  endif()
endforeach()

set(3RDPARTY_INCLUDES 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${3RDPARTY_INCLUDES}
    PARENT_SCOPE)

set(3RDPARTY_DLLS 
    ${3RDPARTY_DLLS}
    PARENT_SCOPE)

set(3RDPARTY_PDBS 
    ${3RDPARTY_PDBS}
    PARENT_SCOPE)