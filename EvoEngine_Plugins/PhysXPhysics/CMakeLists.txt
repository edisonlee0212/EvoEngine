return()

set(PHYSX_PHYSICS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

set(PHYSX_PHYSICS_INCLUDES
	${3RD_PARTY_DIR}/physx/include
	${PHYSX_PHYSICS_DIRECTORY}/include
	)
set(PHYSX_PHYSICS_INCLUDES ${PHYSX_PHYSICS_INCLUDES} PARENT_SCOPE)
file(GLOB_RECURSE PHYSX_PHYSICS_HEADERS ${PHYSX_PHYSICS_DIRECTORY}/include/*.cpp)
file(GLOB_RECURSE PHYSX_PHYSICS_SOURCES ${PHYSX_PHYSICS_DIRECTORY}/src/*.cpp)

add_library(PhysXPhysicsPlugin
	STATIC
	${PHYSX_PHYSICS_HEADERS}
	${PHYSX_PHYSICS_SOURCES}
	)
target_compile_definitions(PhysXPhysicsPlugin
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	${EVOENGINE_PLUGINS_DEFS}
	)
target_include_directories(PhysXPhysicsPlugin
	PRIVATE
	${EVOENGINE_SDK_INCLUDES}
	${EVOENGINE_PLUGINS_INCLUDES}
	${PHYSX_PHYSICS_INCLUDES}
	)

set(PHYSX_PHYSICS_PCH ${PHYSX_PHYSICS_DIRECTORY}/include/PhysXPhysics_PCH.hpp)
set(PHYSX_PHYSICS_PCH ${PHYSX_PHYSICS_PCH} PARENT_SCOPE)

target_precompile_headers(PhysXPhysicsPlugin
	PRIVATE
	${EVOENGINE_SDK_PCH}
	${EVOENGINE_PLUGINS_PCHS}
	${PHYSX_PHYSICS_PCH}
	)
if (WIN32)
	if (CMAKE_BUILD_TYPE MATCHES "^[Rr]elease" OR CMAKE_BUILD_TYPE MATCHES "^[Rr]elWithDebInfo")
		file(GLOB PHYSX_LIBS ${3RD_PARTY_DIR}/physx/win.x86_64.vc143.mt/release/*.lib)
	else()
		file(GLOB PHYSX_LIBS ${3RD_PARTY_DIR}/physx/win.x86_64.vc143.mt/debug/*.lib)
	endif ()
	# Compiler settings for Windows platform
	set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
elseif (UNIX AND NOT APPLE)
	if (CMAKE_BUILD_TYPE MATCHES "^[Rr]elease")
		file(GLOB PHYSX_LIBS ${3RD_PARTY_DIR}/physx/linux.clang/release/*.a)
		set(CMAKE_CXX_FLAGS_RELEASE "-O3")
	elseif (CMAKE_BUILD_TYPE MATCHES "[Dd]ebug")
		file(GLOB PHYSX_LIBS ${3RD_PARTY_DIR}/physx/linux.clang/debug/*.a)
		set(CMAKE_CXX_FLAGS_DEBUG "-g")
	endif ()
elseif (APPLE)
endif ()
target_link_libraries(PhysXPhysicsPlugin
	${PHYSX_LIBS}
	)

set(EVOENGINE_PLUGINS_INCLUDES ${EVOENGINE_PLUGINS_INCLUDES} ${PHYSX_PHYSICS_INCLUDES} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_PCHS ${EVOENGINE_PLUGINS_PCHS} ${PHYSX_PHYSICS_PCH} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_DEFS ${EVOENGINE_PLUGINS_DEFS} PHYSX_PHYSICS_PLUGIN PARENT_SCOPE)

if (WIN32)
    if (CMAKE_BUILD_TYPE MATCHES "^[Rr]elease" OR CMAKE_BUILD_TYPE MATCHES "^[Rr]elWithDebInfo")
        file(GLOB PHYSX_DLLS ${3RD_PARTY_DIR}/physx/win.x86_64.vc143.mt/release/*.dll)
    elseif (CMAKE_BUILD_TYPE MATCHES "[Dd]ebug")
        file(GLOB PHYSX_DLLS ${3RD_PARTY_DIR}/physx/win.x86_64.vc143.mt/debug/*.dll)
    endif ()
elseif (UNIX AND NOT APPLE)
    if (CMAKE_BUILD_TYPE MATCHES "^[Rr]elease" OR CMAKE_BUILD_TYPE MATCHES "^[Rr]elWithDebInfo")
        file(GLOB PHYSX_DLLS ${3RD_PARTY_DIR}/physx/linux.clang/release/*.so)
    elseif (CMAKE_BUILD_TYPE MATCHES "[Dd]ebug")
        file(GLOB PHYSX_DLLS ${3RD_PARTY_DIR}/physx/linux.clang/debug/*.so)
    endif ()
endif ()
set(EVOENGINE_PLUGINS_DLLS ${EVOENGINE_PLUGINS_DLLS} ${PHYSX_DLLS} PARENT_SCOPE)
