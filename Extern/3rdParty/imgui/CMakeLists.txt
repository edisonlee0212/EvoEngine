set(LOCAL_IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/imgui)
set(LOCAL_IMGUIZMO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ImGuizmo)

if (WIN32)
	set(IMGUI_BACKENDS ${LOCAL_IMGUI_DIR}/backends/imgui_impl_win32.cpp ${LOCAL_IMGUI_DIR}/backends/imgui_impl_glfw.cpp ${LOCAL_IMGUI_DIR}/backends/imgui_impl_vulkan.cpp)
else()
	set(IMGUI_BACKENDS ${LOCAL_IMGUI_DIR}/backends/imgui_impl_glfw.cpp ${LOCAL_IMGUI_DIR}/backends/imgui_impl_vulkan.cpp)
endif ()
file(GLOB LOCAL_IMGUI_SOURCES ${LOCAL_IMGUI_DIR}/*.cpp)
file(GLOB LOCAL_IMGUIZMO_SOURCES ${LOCAL_IMGUIZMO_DIR}/*.cpp)

set(IMGUI_SOURCES
	${LOCAL_IMGUI_SOURCES}
	${IMGUI_BACKENDS}
	${LOCAL_IMGUIZMO_SOURCES}
	PARENT_SCOPE
	)

set(3RDPARTY_INCLUDES 
	${3RDPARTY_INCLUDES}
	${CMAKE_CURRENT_SOURCE_DIR}/imgui
	${CMAKE_CURRENT_SOURCE_DIR}/ImGuizmo
	PARENT_SCOPE)