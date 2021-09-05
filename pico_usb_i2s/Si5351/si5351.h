#ifndef SI5351_H
#define SI5351_H

#include "pico/stdlib.h"
#include "stdint.h"
#include "si5351_reg.h"
#include "hardware/i2c.h"

void setup_si5351_i2c();
void si5351_send(int reg_count, const si5351a_revb_register_t *si5351_regs);
void si5351_set_clock(uint8_t audio_bit, uint32_t audio_frequency);

#endif // SI5351_H