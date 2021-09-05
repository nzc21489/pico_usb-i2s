project(user_program)

add_library(user_program STATIC
    user_program.cpp
)

target_link_libraries(user_program pico_stdlib hardware_i2c)
