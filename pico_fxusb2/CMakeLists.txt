if (NOT TARGET pico_fxusb2)
    add_library(pico_fxusb2 INTERFACE)

    pico_generate_pio_header(pico_fxusb2 ${CMAKE_CURRENT_LIST_DIR}/pico_fxusb2.pio)

    target_link_libraries(pico_fxusb2 INTERFACE hardware_dma hardware_pio hardware_irq pico_fxusb2)
endif()
