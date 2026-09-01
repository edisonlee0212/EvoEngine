add_library(bc7enc STATIC
  bc7enc.c
  bc7enc.h
  bc7decomp.cpp
  bc7decomp.h)

target_include_directories(bc7enc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(bc7enc PROPERTIES FOLDER 3rdParty)

set(3RDPARTY_INCLUDES ${3RDPARTY_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
set(3RDPARTY_LIBS ${3RDPARTY_LIBS} bc7enc PARENT_SCOPE)
