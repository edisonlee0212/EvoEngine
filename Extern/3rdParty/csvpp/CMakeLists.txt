set (TARGET_NAME csvpp)
project (${TARGET_NAME})

set (PROJECT_HEADERS
    csvpp/include/csvpp.h
    csvpp/include/stringhelper.h
)

set (PROJECT_SOURCES
    csvpp/include/csvpp.cpp
    csvpp/include/stringhelper.cpp
)

add_library (
    ${TARGET_NAME}
    STATIC
    ${PROJECT_HEADERS}
    ${PROJECT_SOURCES}
)

target_include_directories (
    ${TARGET_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library (
    ${COMMON_LIB}::${TARGET_NAME}
    ALIAS
    ${TARGET_NAME}
)
set(3RDPARTY_INCLUDES ${3RDPARTY_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR}/csvpp/include PARENT_SCOPE)

set_target_properties(${TARGET_NAME} PROPERTIES FOLDER "3rdparty")