cmake_minimum_required(VERSION 3.16)
project(attiny3224_termo C ASM)

execute_process(COMMAND git rev-parse --short HEAD OUTPUT_VARIABLE COMMIT_ID OUTPUT_STRIP_TRAILING_WHITESPACE)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wstrict-prototypes")
set(CMAKE_C_FLAGS_RELEASE "-g -Os")

configure_file (
  "${CMAKE_CURRENT_SOURCE_DIR}/include/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/include/config.h"
  )

set(PROJECT_INCLUDES ${CMAKE_PREFIX_PATH}/include)
include_directories("${PROJECT_INCLUDES}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
include_directories("${CMAKE_CURRENT_BINARY_DIR}/include")

add_subdirectory(src)
