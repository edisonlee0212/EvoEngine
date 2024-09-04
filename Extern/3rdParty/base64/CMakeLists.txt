set (PROJECT_NAME base64)
project (${PROJECT_NAME})

set (PROJECT_HEADERS
    base64/include/base64.h
)

set (PROJECT_SOURCES
    base64/include/base64.cpp
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
set(3RDPARTY_INCLUDES ${3RDPARTY_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR}/base64/include PARENT_SCOPE)