project(si5351)

add_library(si5351 STATIC
    si5351.cpp
)

target_link_libraries(si5351 pico_stdlib hardware_i2c)
