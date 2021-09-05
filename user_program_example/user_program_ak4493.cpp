#include "user_program.h"
#include "pico_usb_i2s_config.h"

#define ak4493address 0x11
uint8_t DFS[8] = {0b000, 0b010, 0b001, 0b100, 0b0100, 0b100, 0b100, 0b100};

uint8_t ak4493_register_value[10] = {
    0b10000111, // 0
    0b10100010, // 1
    0b00000010, // 2
    0b11111111, // 3
    0b11111111, // 4
    0b00000000, // 5
    0b00000000, // 6
    0b00000000, // 7
    0b00000000, // 8
    0b00000000  // 9
};

void ak4493_setup()
{
    uint8_t reg_data[2];
    int bw;
    for (uint8_t i = 0; i < 10; i++)
    {
        reg_data[0] = i;
        reg_data[1] = ak4493_register_value[i];
        bw = i2c_write_blocking(i2c_port, ak4493address, &reg_data[0], 2, false);
        sleep_ms(100);
    }

    // reset ak4493
    // 0
    ak4493_register_value[0] &= 0b11111110;
    reg_data[0] = 0;
    reg_data[1] = ak4493_register_value[0];
    bw = i2c_write_blocking(i2c_port, ak4493address, &reg_data[0], 2, false);

    // 0
    ak4493_register_value[0] |= 0b00000001;
    reg_data[0] = 0;
    reg_data[1] = ak4493_register_value[0];
    bw = i2c_write_blocking(i2c_port, ak4493address, &reg_data[0], 2, false);
}

void ak4493_mute()
{
    ak4493_register_value[1] |= 0b00000001;
    uint8_t reg_data[2];
    int bw;
    reg_data[0] = 1;
    reg_data[1] = ak4493_register_value[1];
    bw = i2c_write_blocking(i2c_port, ak4493address, &reg_data[0], 2, false);
}

void ak4493_unmute()
{
    ak4493_register_value[1] &= 0b11111110;
    uint8_t reg_data[2];
    int bw;
    reg_data[0] = 1;
    reg_data[1] = ak4493_register_value[1];
    bw = i2c_write_blocking(i2c_port, ak4493address, &reg_data[0], 2, false);
}

void ak4493_set_DFS()
{
    uint8_t audio_sample = 0;

    if (audio_frequency % 48000 == 0)
    {
        switch (audio_frequency / 48000)
        {
        case 1:
            audio_sample = 1;
            break;

        case 2:
            audio_sample = 3;
            break;

        case 4:
            audio_sample = 5;
            break;

        case 8:
            audio_sample = 7;
            break;

        default:
            break;
        }
    }
    else
    {
        if (audio_frequency % 44100 == 0)
        {
            switch (audio_frequency / 44100)
            {
            case 1:
                audio_sample = 0;
                break;

            case 2:
                audio_sample = 2;
                break;

            case 4:
                audio_sample = 4;
                break;

            case 8:
                audio_sample = 6;
                break;

            default:
                break;
            }
        }
    }

    uint8_t DFS0 = (DFS[audio_sample] & 0b1) << 3;
    if (DFS0 > 0)
    {
        ak4493_register_value[1] |= DFS0;
    }
    else
    {
        DFS0 |= 0b11111011;
        ak4493_register_value[1] &= DFS0;
    }

    uint8_t DFS1 = (DFS[audio_sample] & 0b10) << 3;
    if (DFS1 > 0)
    {
        ak4493_register_value[1] |= DFS1;
    }
    else
    {
        DFS1 |= 0b11111101;
        ak4493_register_value[1] &= DFS1;
    }

    uint8_t reg_data[2];
    int bw;
    reg_data[0] = 1;
    reg_data[1] = ak4493_register_value[1];
    bw = i2c_write_blocking(i2c_port, ak4493address, &reg_data[0], 2, false);

    uint8_t DFS2 = (DFS[audio_sample] & 0b100) >> 1;
    if (DFS2 > 0)
    {
        ak4493_register_value[5] |= DFS2;
    }
    else
    {
        DFS2 |= 0b11111101;
        ak4493_register_value[5] &= DFS2;
    }

    reg_data[0] = 5;
    reg_data[1] = ak4493_register_value[5];
    bw = i2c_write_blocking(i2c_port, ak4493address, &reg_data[0], 2, false);

    // reset ak4493
    // 0
    ak4493_register_value[0] &= 0b11111110;
    reg_data[0] = 0;
    reg_data[1] = ak4493_register_value[0];
    bw = i2c_write_blocking(i2c_port, ak4493address, &reg_data[0], 2, false);

    // 0
    ak4493_register_value[0] |= 0b00000001;
    reg_data[0] = 0;
    reg_data[1] = ak4493_register_value[0];
    bw = i2c_write_blocking(i2c_port, ak4493address, &reg_data[0], 2, false);
}

void ak4493_set_DIF()
{
    if ((audio_bit == 24) || (audio_bit == 16))
    {
        ak4493_register_value[0] &= 0b11110111;
        ak4493_register_value[0] |= 0b00000110;
    }
    else
    {
        ak4493_register_value[0] |= 0b00001110;
    }
    uint8_t reg_data[2];
    int bw;
    reg_data[0] = 0;
    reg_data[1] = ak4493_register_value[0];
    bw = i2c_write_blocking(i2c_port, ak4493address, &reg_data[0], 2, false);
}

void setup()
{
    ak4493_setup();
    ak4493_mute();
}

void i2s_start()
{
    ak4493_set_DIF();
    ak4493_set_DFS();
    sleep_ms(100);
    ak4493_unmute();
}

void loop()
{
}

void bit_freq_change()
{
    ak4493_mute();
}