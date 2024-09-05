set(PHYSICS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

set(PHYSICS_INCLUDES
	${3RD_PARTY_DIR}/physx/include
	${PHYSICS_DIRECTORY}/include
	)
set(PHYSICS_INCLUDES ${PHYSICS_INCLUDES} PARENT_SCOPE)

file(GLOB PHYSICS_SOURCES ${PHYSICS_DIRECTORY}/src/*.cpp)

add_library(PhysicsPlugin
	STATIC
	${PHYSICS_SOURCES}
	)
target_compile_definitions(PhysicsPlugin
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	${EVOENGINE_PLUGINS_DEFS}
	)
target_include_directories(PhysicsPlugin
	PRIVATE
	${EVOENGINE_SDK_INCLUDES}
	${EVOENGINE_PLUGINS_INCLUDES}
	${PHYSICS_INCLUDES}
	)

set(PHYSICS_PCH ${PHYSICS_DIRECTORY}/include/Physics_PCH.hpp)
set(PHYSICS_PCH ${PHYSICS_PCH} PARENT_SCOPE)

target_precompile_headers(PhysicsPlugin
	PRIVATE
	${EVOENGINE_SDK_PCH}
	${EVOENGINE_PLUGINS_PCHS}
	${PHYSICS_PCH}
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
target_link_libraries(PhysicsPlugin
	${PHYSX_LIBS}
	)

set(EVOENGINE_PLUGINS_INCLUDES ${EVOENGINE_PLUGINS_INCLUDES} ${PHYSICS_INCLUDES} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_PCHS ${EVOENGINE_PLUGINS_PCHS} ${PHYSICS_PCH} PARENT_SCOPE)
set(EVOENGINE_PLUGINS_DEFS ${EVOENGINE_PLUGINS_DEFS} PHYSICS_PLUGIN PARENT_SCOPE)

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

set(EvoEngine_Plugins ${EvoEngine_Plugins} PhysicsPlugin PARENT_SCOPE)

set_target_properties(PhysicsPlugin PROPERTIES FOLDER "Plugins")