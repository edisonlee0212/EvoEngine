add_definitions("-DGPR_WRITING=1")
add_definitions("-DGPR_READING=1")

include_directories(${3RDPARTY_DIR}/gpr/gpr/app/common)
include_directories(${3RDPARTY_DIR}/gpr/gpr/common/argument_parser)
include_directories(${3RDPARTY_DIR}/gpr/gpr/lib/common/private)
include_directories(${3RDPARTY_DIR}/gpr/gpr/lib/common/public)
include_directories(${3RDPARTY_DIR}/gpr/gpr/source/lib/common/public)
include_directories(${3RDPARTY_DIR}/gpr/gpr/source/lib/vc5_common)
include_directories(${3RDPARTY_DIR}/gpr/gpr/source/lib/vc5_decoder)
include_directories(${3RDPARTY_DIR}/gpr/gpr/source/lib/vc5_encoder)

include_directories(${3RDPARTY_DIR}/gpr/gpr/source/lib/dng_sdk)
include_directories(${3RDPARTY_DIR}/gpr/gpr/source/lib/gpr_sdk/public)
include_directories(${3RDPARTY_DIR}/gpr/gpr/source/lib/md5_lib)


add_subdirectory(gpr EXCLUDE_FROM_ALL)

set(3RDPARTY_INCLUDES ${3RDPARTY_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR}/gpr PARENT_SCOPE)

set_property(TARGET common PROPERTY FOLDER 3rdParty/gpr)
set_property(TARGET dng_sdk PROPERTY FOLDER 3rdParty/gpr)
set_property(TARGET expat_lib PROPERTY FOLDER 3rdParty/gpr)
set_property(TARGET gpr_sdk PROPERTY FOLDER 3rdParty/gpr)
set_property(TARGET md5_lib PROPERTY FOLDER 3rdParty/gpr)
set_property(TARGET vc5_common PROPERTY FOLDER 3rdParty/gpr)
set_property(TARGET vc5_decoder PROPERTY FOLDER 3rdParty/gpr)
set_property(TARGET vc5_encoder PROPERTY FOLDER 3rdParty/gpr)
set_property(TARGET xmp_core PROPERTY FOLDER 3rdParty/gpr)
