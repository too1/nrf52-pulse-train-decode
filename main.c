/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
* @defgroup temperature_example_main main.c
* @{
* @ingroup temperature_example
* @brief Temperature Example Application main file.
* @details
* This file contains the source code for a sample application using the temperature sensor.
* This contains workaround for PAN_028 rev2.0A anomalies 28, 29,30 and 31. PAN 43 is not covered.
*  - PAN_028 rev2.0A anomaly 28 - TEMP: Negative measured values are not represented correctly
*  - PAN_028 rev2.0A anomaly 29 - TEMP: Stop task clears the TEMP register.
*  - PAN_028 rev2.0A anomaly 30 - TEMP: Temp module analog front end does not power down when DATARDY event occurs.
*  - PAN_028 rev2.0A anomaly 31 - TEMP: Temperature offset value has to be manually loaded to the TEMP module
*  - PAN_028 rev2.0A anomaly 43 - TEMP: Using PPI between DATARDY event and START task is not functional.
*
*/

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "bsp.h"
#include "nrfx_ppi.h"
#include "nrfx_gpiote.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SAMPLE_PIN                      13
#define GPIOTE_CH_HI_TO_LO              4
#define GPIOTE_CH_LO_TO_HI              5
#define CAPTURE_TIMER                   NRF_TIMER3
#define CAPTURE_TIMER_IRQn              TIMER3_IRQn
#define CAPTURE_TIMER_IRQHandler        TIMER3_IRQHandler
#define CMD_TIMEOUT_US                  1000
#define CAPTURE_BUF_SIZE                256

#define CC_NUM                          2

static volatile uint32_t current_cc_index = 0;
static volatile uint32_t last_cc_value = 0;
static uint32_t data_buffer[CAPTURE_BUF_SIZE];
static uint32_t data_buffer_position = 0;
static uint32_t data_buffer_complete[CAPTURE_BUF_SIZE];
static uint32_t data_buffer_complete_size = 0;
static volatile bool capture_complete = false;
static volatile bool buffer_overrun = false;

static volatile uint32_t int_timer_delay_time_us = 50;
#define INT_TIMER_INT_TIME_US 90

static void interrupt_timer_run(uint32_t int_time_us, uint32_t int_interval_us)
{
    NRF_TIMER0->PRESCALER = 0;
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER0->CC[0] = int_interval_us * 16;
    NRF_TIMER0->CC[1] = (int_interval_us + int_time_us) * 16;
    NRF_TIMER0->CC[2] = (int_interval_us + int_time_us + CMD_TIMEOUT_US) * 16;
    NRF_TIMER0->SHORTS = TIMER_SHORTS_COMPARE2_STOP_Msk | TIMER_SHORTS_COMPARE2_CLEAR_Msk;
    NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
    NVIC_SetPriority(TIMER0_IRQn, 0);
    NVIC_EnableIRQ(TIMER0_IRQn);

    static nrf_ppi_channel_t ppi_ch_debug;
    nrfx_ppi_channel_alloc(&ppi_ch_debug);
    nrfx_ppi_channel_assign(ppi_ch_debug, 
                            (uint32_t)&NRF_GPIOTE->EVENTS_IN[GPIOTE_CH_HI_TO_LO], 
                            (uint32_t)&NRF_TIMER0->TASKS_START);
    nrfx_ppi_channel_enable(ppi_ch_debug);
}

void TIMER0_IRQHandler(void)
{
    if(NRF_TIMER0->EVENTS_COMPARE[0])
    {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        nrf_gpio_pin_set(LED_1);
        NRF_TIMER0->EVENTS_COMPARE[1] = 0;
        while(NRF_TIMER0->EVENTS_COMPARE[1] == 0);
        nrf_gpio_pin_clear(LED_1);
    }
}

static void gpiote_capture_reset(void)
{
    current_cc_index = 0;
    last_cc_value = 0;
    for(int i = 0; i < CC_NUM; i++)
    {
        CAPTURE_TIMER->CC[i] = 0;
    }
    CAPTURE_TIMER->CC[5] = 0xFFFFFFFF;
    data_buffer_position = 0;
    NRF_GPIOTE->EVENTS_IN[GPIOTE_CH_LO_TO_HI] = 0;
    NRF_GPIOTE->EVENTS_IN[GPIOTE_CH_HI_TO_LO] = 0;
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
}

void CAPTURE_TIMER_IRQHandler(void)
{
    static uint32_t current_cc_index = 0;
    if(CAPTURE_TIMER->EVENTS_COMPARE[5])
    {
        CAPTURE_TIMER->EVENTS_COMPARE[5] = 0;

        //memcpy(data_buffer_complete, data_buffer, data_buffer_position * sizeof(data_buffer[0]));
        for(int i = 0; i < data_buffer_position; i++)
        {
            // Using 50us as a base unit divide down the result
            data_buffer_complete[i] = (data_buffer[i] + 25) / 50;
        }
        data_buffer_complete_size = data_buffer_position;
        capture_complete = true;
        gpiote_capture_reset();
    }
}

static void process_gpiote_irq(uint32_t cc_index)
{
    uint32_t current_cc_value = CAPTURE_TIMER->CC[cc_index];
    if(current_cc_value > last_cc_value)
    {
        if(data_buffer_position < CAPTURE_BUF_SIZE)
        {
            data_buffer[data_buffer_position++] = current_cc_value - last_cc_value;
        }
        else buffer_overrun = true;

        CAPTURE_TIMER->CC[5] = current_cc_value + CMD_TIMEOUT_US;
        last_cc_value = current_cc_value;
    }
    current_cc_index = (current_cc_index + 1) % CC_NUM;
}

void GPIOTE_IRQHandler(void)
{
    const uint32_t event_index_list[] = {GPIOTE_CH_HI_TO_LO, GPIOTE_CH_LO_TO_HI};
    uint32_t current_cc_value;
    uint32_t cc_index = current_cc_index;
    nrf_gpio_pin_set(LED_2);
    for(int i = cc_index; ; i++)
    {
        if(NRF_GPIOTE->EVENTS_IN[event_index_list[i % CC_NUM]])
        {
            NRF_GPIOTE->EVENTS_IN[event_index_list[i % CC_NUM]] = 0;

            process_gpiote_irq(i % CC_NUM);
        }
        else if(i >= CC_NUM) break;
    }
    nrf_gpio_pin_clear(LED_2);
}

static void gpiote_capture_init(void)
{
    uint32_t err_code;

    nrf_gpio_cfg_input(SAMPLE_PIN, NRF_GPIO_PIN_NOPULL);

    // Timer init
    CAPTURE_TIMER->PRESCALER = 4;
    CAPTURE_TIMER->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
    CAPTURE_TIMER->SHORTS = TIMER_SHORTS_COMPARE5_STOP_Msk | TIMER_SHORTS_COMPARE5_CLEAR_Msk;
    CAPTURE_TIMER->INTENSET = TIMER_INTENSET_COMPARE5_Msk;
    NVIC_SetPriority(CAPTURE_TIMER_IRQn, 3);
    NVIC_EnableIRQ(CAPTURE_TIMER_IRQn);

    // GPIOTE init
    NRF_GPIOTE->CONFIG[GPIOTE_CH_HI_TO_LO] = GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos |
                                             GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos |
                                             SAMPLE_PIN << GPIOTE_CONFIG_PSEL_Pos;
    NRF_GPIOTE->CONFIG[GPIOTE_CH_LO_TO_HI] = GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos |
                                             GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos |
                                             SAMPLE_PIN << GPIOTE_CONFIG_PSEL_Pos;

    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Msk << GPIOTE_CH_HI_TO_LO |
                           GPIOTE_INTENSET_IN0_Msk << GPIOTE_CH_LO_TO_HI;
                            
    NVIC_SetPriority(GPIOTE_IRQn, 3);
    NVIC_EnableIRQ(GPIOTE_IRQn);

    // PPI init
    static nrf_ppi_channel_t ppi_ch_gpiote_lo_a;
    static nrf_ppi_channel_t ppi_ch_gpiote_hi_a;
    nrfx_ppi_channel_alloc(&ppi_ch_gpiote_lo_a);
    nrfx_ppi_channel_alloc(&ppi_ch_gpiote_hi_a);

    nrfx_ppi_channel_assign(ppi_ch_gpiote_lo_a, 
                            (uint32_t)&NRF_GPIOTE->EVENTS_IN[GPIOTE_CH_HI_TO_LO], 
                            (uint32_t)&CAPTURE_TIMER->TASKS_CAPTURE[0]);
    nrfx_ppi_channel_fork_assign(ppi_ch_gpiote_lo_a, (uint32_t)&CAPTURE_TIMER->TASKS_START);

    nrfx_ppi_channel_assign(ppi_ch_gpiote_hi_a, 
                            (uint32_t)&NRF_GPIOTE->EVENTS_IN[GPIOTE_CH_LO_TO_HI], 
                            (uint32_t)&CAPTURE_TIMER->TASKS_CAPTURE[1]);
    nrfx_ppi_channel_fork_assign(ppi_ch_gpiote_hi_a, (uint32_t)&CAPTURE_TIMER->TASKS_START);

    nrfx_ppi_channel_enable(ppi_ch_gpiote_lo_a);
    nrfx_ppi_channel_enable(ppi_ch_gpiote_hi_a);

    gpiote_capture_reset();

    CAPTURE_TIMER->TASKS_CLEAR = 1;
}


int main(void)
{

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("GPIOTE capture example started.");

    nrf_gpio_cfg_output(LED_1);
    nrf_gpio_cfg_output(LED_2);

    nrf_gpio_cfg_input(BUTTON_3, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(BUTTON_4, NRF_GPIO_PIN_PULLUP);

    gpiote_capture_init();

    interrupt_timer_run(INT_TIMER_INT_TIME_US, int_timer_delay_time_us);

    while (true)
    {
        if(capture_complete)
        {
            capture_complete = false;
            
            NRF_LOG_RAW_INFO("Capture: ");
            for(int i = 0; i < data_buffer_complete_size; i++)
            {
                NRF_LOG_RAW_INFO("%i, ", data_buffer_complete[i]);
            }
            NRF_LOG_RAW_INFO("\r\n");
        }
        NRF_LOG_FLUSH();
        if(nrf_gpio_pin_read(BUTTON_3) == 0)
        {
            int_timer_delay_time_us--;
            nrf_delay_ms(10);
            interrupt_timer_run(INT_TIMER_INT_TIME_US, int_timer_delay_time_us);
        }
        if(nrf_gpio_pin_read(BUTTON_4) == 0)
        {
            int_timer_delay_time_us++;
            nrf_delay_ms(10);
            interrupt_timer_run(INT_TIMER_INT_TIME_US, int_timer_delay_time_us);
        }
    }
}

