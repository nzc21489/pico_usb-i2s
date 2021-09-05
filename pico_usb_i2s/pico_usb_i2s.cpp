#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

#include "pico_fxusb2.pio.h"
#include "pico_i2s.pio.h"

#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"

#include "hardware/resets.h"

#include "pico_usb_i2s_config.h"
#include "si5351.h"

#include "user_program.h"

PIO pio = pio0;
uint sm;
uint sm_fx2_communication;

PIO pio_i2s = pio1;
uint sm_i2s;

uint8_t dma_channel[2];
dma_channel_config dma_config[2];

uint8_t dma_channel_i2s[2];
dma_channel_config dma_config_i2s[2];

uint16_t usb_buf[usb_buf_depth][usb_depth]; // 2 * usb_depth * usb_buf_depth byte

volatile uint8_t buff_num = 0;
volatile uint8_t i2s_buff_num = 0;

volatile uint8_t audio_bit = 0;
volatile uint32_t audio_frequency = 0;

uint offset_i2s;

volatile int int_count_dma = 0;
volatile int int_count_i2s = 0;

volatile bool fx_usb2_receive_interrupt = false;

uint32_t feedback_time = 0;

uint pico_fxusb2_offset;
const pio_program_t *pico_fxusb2_pio_program;

const pio_program_t *i2s_pio_program;

uint offset_fx2_communication;

uint8_t lrck_timing_table[4][4] = {{31, 31, 31, 30},  // 16 bit : 48k, 96k, 192k, 384k
                                   {31, 31, 31, 30},  // 16 bit : 44.1k, 88.2k, 176.4k, 352.8k
                                   {63, 63, 62, 60},  // 32 bit : 48k, 96k, 192k, 384k
                                   {63, 63, 62, 60}}; // 32 bit : 44.1k, 88.2k, 176.4k, 352.8k

// USB DMA handler
void __isr __time_critical_func(dma_handler0)()
{
    if (dma_irqn_get_channel_status(0, dma_channel[0]))
    {
        dma_irqn_acknowledge_channel(0, dma_channel[0]);

        buff_num++;
        buff_num %= usb_buf_depth;

        uint8_t buff_sel = buff_num + 1;

        buff_sel %= usb_buf_depth;

        if (int_count_dma == 0)
        {
            dma_channel_configure(dma_channel[0], &dma_config[0],
                                  &usb_buf[0][0], // dst
                                  &pio->rxf[sm],  // src
                                  usb_depth,      // transfer count
                                  false           // start immediately
            );
        }

        dma_channel_set_write_addr(dma_channel[0], &usb_buf[buff_sel][0], false);
        int_count_dma++;
        irq_set_priority(DMA_IRQ_0, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_set_priority(DMA_IRQ_1, 0);
    }
}

void __isr __time_critical_func(dma_handler1)()
{
    if (dma_irqn_get_channel_status(1, dma_channel[1]))
    {
        dma_irqn_acknowledge_channel(1, dma_channel[1]);

        buff_num++;

        buff_num %= usb_buf_depth;

        uint8_t buff_sel = buff_num + 1;

        buff_sel %= usb_buf_depth;
        dma_channel_set_write_addr(dma_channel[1], &usb_buf[buff_sel][0], false);
        int_count_dma++;
        irq_set_priority(DMA_IRQ_1, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_set_priority(DMA_IRQ_0, 0);
    }
}

// I2S DMA handler
void __isr __time_critical_func(dma_handler_i2s0)()
{
    if (dma_irqn_get_channel_status(0, dma_channel_i2s[0]))
    {
        dma_irqn_acknowledge_channel(0, dma_channel_i2s[0]);

        i2s_buff_num++;

        i2s_buff_num %= usb_buf_depth;

        uint8_t i2s_buff_sel = i2s_buff_num + 1;

        i2s_buff_sel %= usb_buf_depth;

        dma_channel_set_read_addr(dma_channel_i2s[0], &usb_buf[i2s_buff_sel][0], false);
        int_count_i2s++;
    }
}

void __isr __time_critical_func(dma_handler_i2s1)()
{
    if (dma_irqn_get_channel_status(1, dma_channel_i2s[1]))
    {
        dma_irqn_acknowledge_channel(1, dma_channel_i2s[1]);

        i2s_buff_num++;

        i2s_buff_num %= usb_buf_depth;

        uint8_t i2s_buff_sel = i2s_buff_num + 1;

        i2s_buff_sel %= usb_buf_depth;

        dma_channel_set_read_addr(dma_channel_i2s[1], &usb_buf[i2s_buff_sel][0], false);
        int_count_i2s++;
    }
}

void __isr __time_critical_func(fx_usb2_receive_irq)()
{
    fx_usb2_receive_interrupt = true;
    pio->irq = 1u << sm_fx2_communication;
}

void pico_fxusb2_setup()
{
    sm = pio_claim_unused_sm(pio, true);

    uint offset;

    if (audio_bit == 24)
    {
        offset = pio_add_program(pio, &pico_fxusb2_24bit_program);
        pico_fxusb2_pio_program = &pico_fxusb2_24bit_program;
    }
    else
    {
        offset = pio_add_program(pio, &pico_fxusb2_program);
        pico_fxusb2_pio_program = &pico_fxusb2_program;
    }

    pico_fxusb2_offset = offset;

    dma_channel[0] = dma_claim_unused_channel(true);
    dma_channel[1] = dma_claim_unused_channel(true);

    // USB DMA channel 0
    {
        dma_config[0] = dma_channel_get_default_config(dma_channel[0]);
        channel_config_set_transfer_data_size(&dma_config[0], DMA_SIZE_8);
        channel_config_set_read_increment(&dma_config[0], false);
        channel_config_set_write_increment(&dma_config[0], true);

        channel_config_set_dreq(&dma_config[0], pio_get_dreq(pio, sm, false));
        channel_config_set_chain_to(&dma_config[0], dma_channel[1]);
        dma_channel_set_irq0_enabled(dma_channel[0], true);

        irq_add_shared_handler(DMA_IRQ_0, dma_handler0, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_set_enabled(DMA_IRQ_0, true);

        if (audio_bit == 24)
        {
            dma_channel_configure(dma_channel[0], &dma_config[0],
                                  &usb_buf[0][1], // dst
                                  &pio->rxf[sm],  // src
                                  usb_depth - 3,  // transfer count 24bit
                                  true            // start immediately
            );
        }
        else
        {
            dma_channel_configure(dma_channel[0], &dma_config[0],
                                  &usb_buf[0][1], // dst
                                  &pio->rxf[sm],  // src
                                  usb_depth - 2,  // transfer count 16/32bit
                                  true            // start immediately
            );
        }
    }

    // USB DMA channel 1
    {
        dma_config[1] = dma_channel_get_default_config(dma_channel[1]);
        channel_config_set_transfer_data_size(&dma_config[1], DMA_SIZE_8);
        channel_config_set_read_increment(&dma_config[1], false);
        channel_config_set_write_increment(&dma_config[1], true);

        channel_config_set_dreq(&dma_config[1], pio_get_dreq(pio, sm, false));
        channel_config_set_chain_to(&dma_config[1], dma_channel[0]);
        dma_channel_set_irq1_enabled(dma_channel[1], true);

        irq_add_shared_handler(DMA_IRQ_1, dma_handler1, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_set_enabled(DMA_IRQ_1, true);

        dma_channel_configure(dma_channel[1], &dma_config[1],
                              &usb_buf[1][0], // dst
                              &pio->rxf[sm],  // src
                              usb_depth,      // transfer count
                              false           // start immediately
        );
    }

    if (audio_bit == 24)
    {
        pico_fxusb2_24bit_program_init(pio, sm, offset, pico_fxusb2_pin_base);
    }
    else
    {
        pico_fxusb2_program_init(pio, sm, offset, pico_fxusb2_pin_base);
    }
}

void init_i2s()
{
    pio_gpio_init(pio_i2s, pico_i2s_pin_base);
    pio_gpio_init(pio_i2s, pico_i2s_pin_data);

    sm_i2s = pio_claim_unused_sm(pio_i2s, true);

    offset_i2s = pio_add_program(pio_i2s, &audio_i2s_program);
    i2s_pio_program = &audio_i2s_program;

    uint8_t lrck_setting;

    if (audio_bit == 16)
    {
            switch (audio_frequency)
            {
            case 48000:
                lrck_setting = lrck_timing_table[0][0];
                break;

            case 96000:
                lrck_setting = lrck_timing_table[0][1];
                break;

            case 192000:
                lrck_setting = lrck_timing_table[0][2];
                break;

            case 384000:
                lrck_setting = lrck_timing_table[0][3];
                break;

            case 44100:
                lrck_setting = lrck_timing_table[1][0];
                break;

            case 88200:
                lrck_setting = lrck_timing_table[1][1];
                break;

            case 176400:
                lrck_setting = lrck_timing_table[1][2];
                break;

            case 352800:
                lrck_setting = lrck_timing_table[1][3];
                break;

            default:
                break;
            }
    }
    else
    {
            switch (audio_frequency)
            {
            case 48000:
                lrck_setting = lrck_timing_table[2][0];
                break;

            case 96000:
                lrck_setting = lrck_timing_table[2][1];
                break;

            case 192000:
                lrck_setting = lrck_timing_table[2][2];
                break;

            case 384000:
                lrck_setting = lrck_timing_table[2][3];
                break;

            case 44100:
                lrck_setting = lrck_timing_table[3][0];
                break;

            case 88200:
                lrck_setting = lrck_timing_table[3][1];
                break;

            case 176400:
                lrck_setting = lrck_timing_table[3][2];
                break;

            case 352800:
                lrck_setting = lrck_timing_table[3][3];
                break;

            default:
                break;
            }
    }

    audio_i2s_program_init(pio_i2s, sm_i2s, offset_i2s, pico_i2s_pin_data, pico_i2s_pin_base, lrck_setting);

    dma_channel_i2s[0] = dma_claim_unused_channel(true);
    dma_channel_i2s[1] = dma_claim_unused_channel(true);

    // I2S DMA channel 0
    {
        dma_config_i2s[0] = dma_channel_get_default_config(dma_channel_i2s[0]);
        channel_config_set_transfer_data_size(&dma_config_i2s[0], DMA_SIZE_16);
        channel_config_set_read_increment(&dma_config_i2s[0], true);
        channel_config_set_write_increment(&dma_config_i2s[0], false);

        channel_config_set_dreq(&dma_config_i2s[0], pio_get_dreq(pio_i2s, sm_i2s, true));
        channel_config_set_chain_to(&dma_config_i2s[0], dma_channel_i2s[1]);
        dma_channel_set_irq0_enabled(dma_channel_i2s[0], true);
        irq_add_shared_handler(DMA_IRQ_0, dma_handler_i2s0, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_set_enabled(DMA_IRQ_0, true);

        dma_channel_configure(dma_channel_i2s[0], &dma_config_i2s[0],
                              &pio_i2s->txf[sm_i2s], // dst
                              &usb_buf[0][0],        // src
                              usb_depth / 2,         // transfer count
                              false                  // start immediately
        );
    }

    // I2S DMA channel 1
    {
        dma_config_i2s[1] = dma_channel_get_default_config(dma_channel_i2s[1]);
        channel_config_set_transfer_data_size(&dma_config_i2s[1], DMA_SIZE_16);
        channel_config_set_read_increment(&dma_config_i2s[1], true);
        channel_config_set_write_increment(&dma_config_i2s[1], false);

        channel_config_set_dreq(&dma_config_i2s[1], pio_get_dreq(pio_i2s, sm_i2s, true));
        channel_config_set_chain_to(&dma_config_i2s[1], dma_channel_i2s[0]);
        dma_channel_set_irq1_enabled(dma_channel_i2s[1], true);
        irq_add_shared_handler(DMA_IRQ_1, dma_handler_i2s1, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_set_enabled(DMA_IRQ_1, true);

        dma_channel_configure(dma_channel_i2s[1], &dma_config_i2s[1],
                              &pio_i2s->txf[sm_i2s], // dst
                              &usb_buf[1][0],        // src
                              usb_depth / 2,         // transfer count
                              false                  // start immediately
        );
    }

    int audio_bit_freq = ((int)(audio_bit / 24) + 1) * (int)(audio_frequency / 44100);
    pio_sm_set_clkdiv(pio_i2s, sm_i2s, 16 / audio_bit_freq);
}

void pico_fxusb2_deinit()
{
    dma_channel_abort(dma_channel[0]);
    dma_channel_abort(dma_channel[1]);
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);

    pio_remove_program(pio, pico_fxusb2_pio_program, pico_fxusb2_offset);
    pio_sm_unclaim(pio, sm);
    dma_channel_unclaim(dma_channel[0]);
    dma_channel_unclaim(dma_channel[1]);
    irq_remove_handler(DMA_IRQ_0, dma_handler0);
    irq_remove_handler(DMA_IRQ_1, dma_handler1);
    irq_set_enabled(DMA_IRQ_0, false);
    irq_set_enabled(DMA_IRQ_1, false);
}

void deinit_i2s()
{
    dma_channel_abort(dma_channel_i2s[0]);
    dma_channel_abort(dma_channel_i2s[1]);
    pio_sm_set_enabled(pio_i2s, sm_i2s, false);
    pio_sm_clear_fifos(pio_i2s, sm_i2s);
    pio_sm_restart(pio_i2s, sm_i2s);

    pio_remove_program(pio_i2s, i2s_pio_program, offset_i2s);
    pio_sm_unclaim(pio_i2s, sm_i2s);
    dma_channel_unclaim(dma_channel_i2s[0]);
    dma_channel_unclaim(dma_channel_i2s[1]);
    irq_remove_handler(DMA_IRQ_0, dma_handler_i2s0);
    irq_remove_handler(DMA_IRQ_1, dma_handler_i2s1);
    irq_set_enabled(DMA_IRQ_0, false);
    irq_set_enabled(DMA_IRQ_1, false);
}

void pico_fxusb2_communication_deinit()
{
    pio_sm_set_enabled(pio, sm_fx2_communication, false);
    pio_sm_clear_fifos(pio, sm_fx2_communication);
    pio_sm_restart(pio, sm_fx2_communication);

    pio_remove_program(pio, &pico_fxusb2_communication_program, offset_fx2_communication);
    pio_sm_unclaim(pio, sm_fx2_communication);
    irq_remove_handler(PIO0_IRQ_0, fx_usb2_receive_irq);
    irq_set_enabled(PIO0_IRQ_0, false);
}

uint32_t pico_fxusb2_send(uint32_t data, uint offset_fx2)
{
    pio_sm_set_enabled(pio, sm_fx2_communication, false);
    pio_sm_clear_fifos(pio, sm_fx2_communication);

    pio_sm_put(pio, sm_fx2_communication, data);
    pio_sm_exec(pio, sm_fx2_communication, pio_encode_jmp(offset_fx2 + pico_fxusb2_communication_offset_send_data));
    pio_sm_set_enabled(pio, sm_fx2_communication, true);

    while (!pio_sm_is_tx_fifo_empty(pio, sm_fx2_communication))
    {
        sleep_ms(1);
    }

    uint32_t rx_data;
    sleep_ms(20);
    rx_data = pio_sm_get(pio, sm_fx2_communication);

    return rx_data;
}

void convert_bit_freq(uint8_t pre_convert_data, volatile uint8_t *bit, volatile uint32_t *freq)
{
    switch ((pre_convert_data >> 5) & 0b11)
    {
    case 1:
        *bit = 16;
        break;

    case 2:
        *bit = 24;
        break;

    case 3:
        *bit = 32;
        break;

    default:
        break;
    }

    switch ((pre_convert_data >> 2) & 0b111)
    {
    case 0:
        *freq = 48000;
        break;
    case 1:
        *freq = 44100;
        break;
    case 2:
        *freq = 96000;
        break;
    case 3:
        *freq = 88200;
        break;
    case 4:
        *freq = 192000;
        break;
    case 5:
        *freq = 176400;
        break;
    case 6:
        *freq = 384000;
        break;
    case 7:
        *freq = 352800;
        break;

    default:
        break;
    }
}

int main()
{
    stdio_init_all();

    // regulator power save disable
    gpio_init(23);
    gpio_set_dir(23, GPIO_OUT);
    gpio_put(23, 1);

    for (int i = 0; i < usb_buf_depth; i++)
    {
        for (int j = 0; j < usb_depth; j++)
        {
            usb_buf[i][j] = 0;
        }
    }

    setup_si5351_i2c();

    // user_program
    setup();

    while (1)
    {
        sm_fx2_communication = pio_claim_unused_sm(pio, true);
        offset_fx2_communication = pio_add_program(pio, &pico_fxusb2_communication_program);
        irq_set_exclusive_handler(PIO0_IRQ_0, fx_usb2_receive_irq);
        irq_set_enabled(PIO0_IRQ_0, true);
        pio->inte0 = PIO_IRQ0_INTE_SM0_BITS;

        pio_sm_put(pio, sm_fx2_communication, 0);
        pico_fxusb2_communication_program_init(pio, sm_fx2_communication, offset_fx2_communication, 12);
        uint32_t fxusb2_received_data;

        do
        {
            fxusb2_received_data = pico_fxusb2_send(0x60000, offset_fx2_communication);
        } while (((fxusb2_received_data & 0b11) != 0b01) || (((fxusb2_received_data >> 5) & 0b11) == 0));

        convert_bit_freq(fxusb2_received_data, &audio_bit, &audio_frequency);

        printf("bit = %d, frequency = %d\n", audio_bit, audio_frequency);

        si5351_set_clock(audio_bit, audio_frequency);

        sleep_ms(100);

        // set system clock to MCLK
        if (audio_frequency % 48000 == 0)
        {
            clock_configure_gpin(clk_sys, pico_i2s_mclk_pin, 49152000, 49152000);
        }
        else
        {
            clock_configure_gpin(clk_sys, pico_i2s_mclk_pin, 45158400, 45158400);
        }

        sleep_ms(100);

        pico_fxusb2_setup();

        init_i2s();

        pio_sm_exec(pio_i2s, sm_i2s, pio_encode_jmp(offset_i2s + audio_i2s_offset_entry_point));

        pio_sm_set_enabled(pio, sm_fx2_communication, false);
        pio_sm_clear_fifos(pio, sm_fx2_communication);
        pio_sm_exec(pio, sm_fx2_communication, pio_encode_jmp(offset_fx2_communication + pico_fxusb2_communication_offset_receive_data));
        pio_sm_set_enabled(pio, sm_fx2_communication, true);

        fx_usb2_receive_interrupt = false;
        sleep_ms(10);
        pio_sm_clear_fifos(pio, sm_fx2_communication);

        while (1)
        {
            if (buff_num >= (usb_buf_depth / 2)) // wait until enough buffer
            {
                pio_sm_clear_fifos(pio_i2s, sm_i2s);
                dma_channel_start(dma_channel_i2s[0]);
                pio_sm_set_enabled(pio_i2s, sm_i2s, true); // start I2S
                break;
            }
        }

        // user_program
        i2s_start();

        uint32_t feedback;

        if (audio_frequency % 48000 == 0)
        {
            feedback = 0x60000;
        }
        else
        {
            feedback = 0x58330;
        }

        bool feedback_on = false;
        uint32_t delta_pre = 0;

        feedback_time = 0;

        volatile uint32_t received_data = pico_fxusb2_send(feedback, offset_fx2_communication);

        if (((received_data >> 24) & 0b11) == 0b01 || ((received_data >> 24) & 0b11) == 0b10)
        {
            uint8_t audio_bit_pre = audio_bit;
            uint32_t audio_frequency_pre = audio_frequency;
            convert_bit_freq((received_data >> 24), &audio_bit, &audio_frequency);
            if ((audio_bit != audio_bit_pre) || (audio_frequency != audio_frequency_pre))
            {
                printf("bit / frequency changed\nbit = %d, frequency = %d\n", audio_bit, audio_frequency);
                break;
            }
        }

        while (1)
        {
            // user_program
            loop();

            // check wheter there is enough data
            if ((int_count_i2s - int_count_dma) > (usb_buf_depth / 2 - 1))
            {
                break;
            }

            // interrupt handle
            if (fx_usb2_receive_interrupt)
            {
                fx_usb2_receive_interrupt = false;
                sleep_ms(10);
                uint32_t interrupted_data = pio_sm_get(pio, sm_fx2_communication);
                pio_sm_clear_fifos(pio, sm_fx2_communication);
                if (((interrupted_data >> 24) & 0b11) == 0b01 || ((interrupted_data >> 24) & 0b11) == 0b10)
                {
                    uint8_t audio_bit_pre = audio_bit;
                    uint32_t audio_frequency_pre = audio_frequency;
                    convert_bit_freq((received_data >> 24), &audio_bit, &audio_frequency);
                    if ((audio_bit != audio_bit_pre) || (audio_frequency != audio_frequency_pre))
                    {
                        break;
                    }
                }
            }

            // feedback
            if (to_ms_since_boot(get_absolute_time()) - feedback_time > feedback_interval)
            {
                int delta = ((int_count_dma - int_count_i2s) % usb_buf_depth) - usb_buf_depth / 2;
                while (delta < (-usb_buf_depth))
                {
                    delta += usb_buf_depth;
                }

                printf("dma = %d, i2s = %d, %d, %d\n", int_count_dma, int_count_i2s, (int_count_dma - int_count_i2s), delta);

                if (delta > 1 || delta < -1)
                {
                    feedback_on = true;
                    if (delta_pre != delta)
                    {
                        uint32_t feedback_data = feedback - delta * feedback_coefficient;

                        if (audio_frequency % 48000 == 0)
                        {
                            feedback_data *= (audio_frequency / 48000);
                        }
                        else
                        {
                            feedback_data *= (audio_frequency / 44100);
                        }

                        received_data = pico_fxusb2_send(feedback_data, offset_fx2_communication);

                        if (((received_data >> 24) & 0b11) == 0b01 || ((received_data >> 24) & 0b11) == 0b10)
                        {
                            uint8_t audio_bit_pre = audio_bit;
                            uint32_t audio_frequency_pre = audio_frequency;
                            convert_bit_freq((received_data >> 24), &audio_bit, &audio_frequency);
                            if ((audio_bit != audio_bit_pre) || (audio_frequency != audio_frequency_pre))
                            {
                                break;
                            }
                        }

                        printf("feedback_data = %x\n", feedback_data);
                    }
                }
                else
                {
                    if (feedback_on)
                    {
                        uint32_t feedback_data = feedback;

                        if (audio_frequency % 48000 == 0)
                        {
                            feedback_data *= (audio_frequency / 48000);
                        }
                        else
                        {
                            feedback_data *= (audio_frequency / 44100);
                        }

                        received_data = pico_fxusb2_send(feedback_data, offset_fx2_communication);

                        if (((received_data >> 24) & 0b11) == 0b01 || ((received_data >> 24) & 0b11) == 0b10)
                        {
                            uint8_t audio_bit_pre = audio_bit;
                            uint32_t audio_frequency_pre = audio_frequency;
                            convert_bit_freq((received_data >> 24), &audio_bit, &audio_frequency);
                            if ((audio_bit != audio_bit_pre) || (audio_frequency != audio_frequency_pre))
                            {
                                break;
                            }
                        }

                        printf("feedback_data = %x\n", feedback_data);

                        feedback_on = false;
                    }
                }
                delta_pre = delta;

                feedback_time = to_ms_since_boot(get_absolute_time());
            }
        }

        // user_program
        bit_freq_change();

        deinit_i2s();
        pico_fxusb2_deinit();
        pico_fxusb2_communication_deinit();

        // set system clock to 125MHz
        clock_configure(clk_sys,
                        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                        125 * MHZ,
                        125 * MHZ);

        buff_num = 0;
        i2s_buff_num = 0;
        int_count_dma = 0;
        int_count_i2s = 0;
        fx_usb2_receive_interrupt = false;
        if (dma_irqn_get_channel_status(0, dma_channel[0]))
        {
            dma_irqn_acknowledge_channel(0, dma_channel[0]);
        }
        if (dma_irqn_get_channel_status(1, dma_channel[1]))
        {
            dma_irqn_acknowledge_channel(1, dma_channel[1]);
        }
        if (dma_irqn_get_channel_status(0, dma_channel_i2s[0]))
        {
            dma_irqn_acknowledge_channel(0, dma_channel_i2s[0]);
        }
        if (dma_irqn_get_channel_status(1, dma_channel_i2s[1]))
        {
            dma_irqn_acknowledge_channel(1, dma_channel_i2s[1]);
        }

        for (int i = 0; i < usb_buf_depth; i++)
        {
            for (int j = 0; j < usb_depth; j++)
            {
                usb_buf[i][j] = 0;
            }
        }

        // reset DMA
        reset_block(RESETS_RESET_DMA_BITS);
        unreset_block_wait(RESETS_RESET_DMA_BITS);
    }
    return 0;
}