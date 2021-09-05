cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(pico_usb_i2s)

pico_sdk_init()

set(PICO_GCC_TRIPLE "")

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O2 -Wl,-s -ffunction-sections -fdata-sections -ffat-lto-objects")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -Wl,-s -ffunction-sections -fdata-sections -fno-threadsafe-statics -ffat-lto-objects")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Wl,--gc-sections --use-linker-plugin")

set(INCLUDE_DIR
  ${PROJECT_SOURCE_DIR}/pico_usb_i2s
  ${PROJECT_SOURCE_DIR}/pico_usb_i2s/Si5351
  ${PROJECT_SOURCE_DIR}/pico_fxusb2
  ${PROJECT_SOURCE_DIR}/pico_i2s
  ${PROJECT_SOURCE_DIR}/user_program
  ${PROJECT_SOURCE_DIR}/config
  ${PICO_SDK_PATH}
)

include_directories("${INCLUDE_DIR}")

add_subdirectory(pico_usb_i2s)
add_subdirectory(pico_fxusb2)
add_subdirectory(pico_i2s)
add_subdirectory(user_program)
