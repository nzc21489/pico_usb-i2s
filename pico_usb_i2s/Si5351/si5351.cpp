#include "pico_usb_i2s_config.h"
#include "si5351.h"

const uint8_t Si5351_ADDR = 0x60;

void setup_si5351_i2c()
{
    i2c_init(i2c_port, 100000);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
}

void si5351_send(int reg_count, const si5351a_revb_register_t *si5351_regs)
{
    // disable output
    uint8_t data[2];
    data[0] = 3;
    data[1] = 0xff;
    i2c_write_blocking(i2c_port, Si5351_ADDR, data, 2, false);

    // power down output drivers
    data[0] = 16;
    data[1] = 0x80;
    i2c_write_blocking(i2c_port, Si5351_ADDR, data, 2, false);
    data[0] = 17;
    data[1] = 0x80;
    i2c_write_blocking(i2c_port, Si5351_ADDR, data, 2, false);
    data[0] = 18;
    data[1] = 0x80;
    i2c_write_blocking(i2c_port, Si5351_ADDR, data, 2, false);

    for (volatile int i = 0; i < reg_count; i++)
    {
        uint8_t write_data[2];
        write_data[0] = (si5351_regs[i].address) & 0xff;
        write_data[1] = si5351_regs[i].value;
        if (si5351_regs[i].address == 17)
        {
            write_data[1] |= 0b10000; // clock1 invert
        }
        i2c_write_blocking(i2c_port, Si5351_ADDR, write_data, 2, false);
    }

    // soft reset
    data[0] = 177;
    data[1] = 0xac;
    i2c_write_blocking(i2c_port, Si5351_ADDR, data, 2, false);

    // enable output
    data[0] = 3;
    data[1] = 0x00;
    i2c_write_blocking(i2c_port, Si5351_ADDR, data, 2, false);
}

void si5351_set_clock(uint8_t bit, uint32_t freq)
{
    if (bit == 16)
    {
        switch (freq)
        {
        case 44100:
        {
            si5351_send(SI5351A_REVB_REG_CONFIG_NUM_REGS, &si5351a_16_44100[0]);
        }
        break;

        case 48000:
        {
            si5351_send(SI5351A_REVB_REG_CONFIG_NUM_REGS, &si5351a_16_48000[0]);
        }
        break;

        case 88200:
        {
            si5351_send(SI5351A_REVB_REG_CONFIG_NUM_REGS, &si5351a_16_88200[0]);
        }
        break;

        case 96000:
        {
            si5351_send(SI5351A_REVB_REG_CONFIG_NUM_REGS, &si5351a_16_96000[0]);
        }
        break;

        case 176400:
        {
            si5351_send(SI5351A_REVB_REG_CONFIG_NUM_REGS, &si5351a_16_176400[0]);
        }
        break;

        case 192000:
        {
            si5351_send(SI5351A_REVB_REG_CONFIG_NUM_REGS, &si5351a_16_192000[0]);
        }
        break;

        case 352800:
        {
            si5351_send(SI5351A_REVB_REG_CONFIG_NUM_REGS, &si5351a_16_352800[0]);
        }
        break;

        case 384000:
        {
            si5351_send(SI5351A_REVB_REG_CONFIG_NUM_REGS, &si5351a_16_384000[0]);
        }
        break;

        default:
            break;
        }
    }
    else
    {
        switch (freq)
        {
        case 44100:
        {
            si5351_send(SI5351A_REVB_REG_CONFIG_NUM_REGS, &si5351a_32_44100[0]);
        }
        break;

        case 48000:
        {
            si5351_send(SI5351A_REVB_REG_CONFIG_NUM_REGS, &si5351a_32_48000[0]);
        }
        break;

        case 88200:
        {
            si5351_send(SI5351A_REVB_REG_CONFIG_NUM_REGS, &si5351a_32_88200[0]);
        }
        break;

        case 96000:
        {
            si5351_send(SI5351A_REVB_REG_CONFIG_NUM_REGS, &si5351a_32_96000[0]);
        }
        break;

        case 176400:
        {
            si5351_send(SI5351A_REVB_REG_CONFIG_NUM_REGS, &si5351a_32_176400[0]);
        }
        break;

        case 192000:
        {
            si5351_send(SI5351A_REVB_REG_CONFIG_NUM_REGS, &si5351a_32_192000[0]);
        }
        break;

        case 352800:
        {
            si5351_send(SI5351A_REVB_REG_CONFIG_NUM_REGS, &si5351a_32_352800[0]);
        }
        break;

        case 384000:
        {
            si5351_send(SI5351A_REVB_REG_CONFIG_NUM_REGS, &si5351a_32_384000[0]);
        }
        break;

        default:
            break;
        }
    }
}