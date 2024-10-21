set (TARGET_NAME xatlas)

set(xatlas_directory ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}/source/${TARGET_NAME})

add_library(${TARGET_NAME} STATIC
  ${xatlas_directory}/xatlas.h 
  ${xatlas_directory}/xatlas.cpp
)
target_include_directories(${TARGET_NAME}
  PUBLIC
    ${xatlas_directory}
)
set_property(TARGET ${TARGET_NAME} PROPERTY FOLDER 3rdParty)

set(3RDPARTY_LIBS ${3RDPARTY_LIBS} 
    ${TARGET_NAME}
    PARENT_SCOPE)

set(3RDPARTY_INCLUDES ${3RDPARTY_INCLUDES} ${xatlas_directory} PARENT_SCOPE)