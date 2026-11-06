cmake_minimum_required(VERSION 3.15)
project(helios_soybean_exporter)

set(HELIOS_ROOT "" CACHE PATH "Path to the Helios source tree")
if(NOT HELIOS_ROOT OR NOT EXISTS "${HELIOS_ROOT}/core/CMake_project.cmake")
    message(FATAL_ERROR "Set HELIOS_ROOT to a Helios source tree")
endif()

get_filename_component(BASE_DIRECTORY "${HELIOS_ROOT}" ABSOLUTE)
set(EXECUTABLE_NAME "helios_soybean_exporter")
set(SOURCE_FILES "main.cpp")
set(PLUGINS "plantarchitecture")
# Helios' OBJ writer needs OpenMP enabled to emit every generated face.
set(ENABLE_OPENMP ON CACHE BOOL "" FORCE)
if(MSVC)
    # Helios replaces the Release flags and otherwise drops exception unwinding on current MSVC.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif()

include("${BASE_DIRECTORY}/core/CMake_project.cmake")

install(TARGETS ${EXECUTABLE_NAME} RUNTIME DESTINATION bin COMPONENT HeliosSoybeanExporter)
install(
    FILES
        "${BASE_DIRECTORY}/plugins/plantarchitecture/assets/obj/PetiolulePrototype.obj"
        "${BASE_DIRECTORY}/plugins/plantarchitecture/assets/obj/PetiolulePrototype.mtl"
        "${BASE_DIRECTORY}/plugins/plantarchitecture/assets/obj/SoybeanFlower_open_white.obj"
        "${BASE_DIRECTORY}/plugins/plantarchitecture/assets/obj/SoybeanFlower_open_white.mtl"
        "${BASE_DIRECTORY}/plugins/plantarchitecture/assets/obj/BeanFlower_closed_white.obj"
        "${BASE_DIRECTORY}/plugins/plantarchitecture/assets/obj/BeanFlower_closed_white.mtl"
        "${BASE_DIRECTORY}/plugins/plantarchitecture/assets/obj/SoybeanPod.obj"
        "${BASE_DIRECTORY}/plugins/plantarchitecture/assets/obj/SoybeanPod.mtl"
    DESTINATION "bin/plugins/plantarchitecture/assets/obj"
    COMPONENT HeliosSoybeanExporter
)
install(
    FILES "${BASE_DIRECTORY}/plugins/plantarchitecture/assets/textures/SoybeanLeaf.png"
    DESTINATION "bin/plugins/plantarchitecture/assets/textures"
    COMPONENT HeliosSoybeanExporter
)
