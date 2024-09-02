set (PROJECT_NAME csvpp)
project (${PROJECT_NAME})

set (PROJECT_HEADERS
    csvpp/include/csvpp.h
    csvpp/include/stringhelper.h
)

set (PROJECT_SOURCES
    csvpp/include/csvpp.cpp
    csvpp/include/stringhelper.cpp
)

add_library (
    ${PROJECT_NAME}
    STATIC
    ${PROJECT_HEADERS}
    ${PROJECT_SOURCES}
)

target_include_directories (
    ${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library (
    ${COMMON_LIB}::${PROJECT_NAME}
    ALIAS
    ${PROJECT_NAME}
)
set(3RDPARTY_INCLUDES ${3RDPARTY_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR}/csvpp/include PARENT_SCOPE)