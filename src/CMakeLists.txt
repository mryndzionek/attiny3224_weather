include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

file(GLOB U8G2_SRCS
     "../external/u8g2/csrc/*.c"
)

include_directories("../external/u8g2/csrc/")

avr_add_library(
    u8g2
    ${U8G2_SRCS}
)

avr_add_executable(
    main
    main.c
    u8x8_tinyavr_2.c
)

avr_target_link_libraries(main u8g2)

