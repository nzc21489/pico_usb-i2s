project(pico_usb_i2s)

add_executable(pico_usb_i2s
        pico_usb_i2s.cpp
        )
        
target_link_libraries(pico_usb_i2s pico_stdlib pico_fxusb2 pico_audio_i2s si5351 user_program)

add_subdirectory(Si5351)

pico_enable_stdio_usb(pico_usb_i2s 1)

pico_add_extra_outputs(pico_usb_i2s)
