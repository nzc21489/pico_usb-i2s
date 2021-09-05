#ifndef PICO_USB_I2S_CONFIG_H
#define PICO_USB_I2S_CONFIG_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define usb_buf_depth 8
#define usb_depth 1024 * 8 // 16kB
#define feedback_coefficient 200
#define feedback_interval 1000 // ms

#define pico_fxusb2_pin_base 2
#define pico_i2s_mclk_pin 20 // must be 20 or 22
#define pico_i2s_pin_base 21
#define pico_i2s_pin_data 22

#define i2c_port i2c1
#define sda_pin 26
#define scl_pin 27

extern volatile uint8_t audio_bit;
extern volatile uint32_t audio_frequency;

#endif // PICO_USB_I2S_CONFIG_H