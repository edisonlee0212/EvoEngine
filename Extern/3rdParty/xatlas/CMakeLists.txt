set(xatlas_directory ${CMAKE_CURRENT_SOURCE_DIR}/xatlas/source/xatlas)

add_library(xatlas STATIC
  ${xatlas_directory}/xatlas.h 
  ${xatlas_directory}/xatlas.cpp
)
target_include_directories(xatlas
  PUBLIC
    ${xatlas_directory}
)
set_property(TARGET xatlas PROPERTY FOLDER "3rdparty")

set(3RDPARTY_INCLUDES ${3RDPARTY_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR}/xatlas/source/xatlas PARENT_SCOPE)