set(YAML_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp/)
SET(YAML_CPP_INSTALL OFF CACHE BOOL "Enable generation of yaml-cpp install targets")
SET(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "Enable yaml-cpp contrib in library")
SET(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "Enable parse tools")
SET(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "Enable yaml-cpp tests")
add_subdirectory(${YAML_CPP_DIR} EXCLUDE_FROM_ALL)

set(3RDPARTY_INCLUDES ${3RDPARTY_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp/include PARENT_SCOPE)

set_property(TARGET yaml-cpp PROPERTY FOLDER "3rdparty")