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
#define CMD_TIMEOUT_US                  1000000

#define CC_NUM                          2

static volatile uint32_t current_cc_index = 0;
static volatile uint32_t last_cc_value = 0;
static uint32_t data_buffer[64];
static uint32_t data_buffer_position = 0;

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

        gpiote_capture_reset();
    }
}

static void process_gpiote_irq(uint32_t cc_index)
{
    uint32_t current_cc_value = CAPTURE_TIMER->CC[cc_index];
    if(current_cc_value > last_cc_value)
    {
        data_buffer[data_buffer_position++] = current_cc_value - last_cc_value;
        CAPTURE_TIMER->CC[5] = current_cc_value + CMD_TIMEOUT_US;
        last_cc_value = current_cc_value;
        current_cc_index = (current_cc_index + 1) % CC_NUM;
    }
}

void GPIOTE_IRQHandler(void)
{
    const uint32_t event_index_list[] = {GPIOTE_CH_HI_TO_LO, GPIOTE_CH_LO_TO_HI};
    uint32_t current_cc_value;
    uint32_t cc_index = current_cc_index;
    for(int i = cc_index; i < (cc_index + CC_NUM); i++)
    {
        if(NRF_GPIOTE->EVENTS_IN[event_index_list[i % CC_NUM]])
        {
            NRF_GPIOTE->EVENTS_IN[event_index_list[i % CC_NUM]] = 0;

            process_gpiote_irq(i % CC_NUM);
        }
    }
}

static void gpiote_capture_init(void)
{
    uint32_t err_code;

    nrf_gpio_cfg_input(SAMPLE_PIN, NRF_GPIO_PIN_PULLUP);

    // Timer init
    CAPTURE_TIMER->PRESCALER = 4;
    CAPTURE_TIMER->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
    CAPTURE_TIMER->SHORTS = TIMER_SHORTS_COMPARE5_STOP_Msk | TIMER_SHORTS_COMPARE5_CLEAR_Msk;
    CAPTURE_TIMER->INTENSET = TIMER_INTENSET_COMPARE5_Msk;
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
    //CAPTURE_TIMER->TASKS_START = 1;
}


/** @brief Function for main application entry.
 */
int main(void)
{

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("GPIOTE capture example started.");

    gpiote_capture_init();

    while (true)
    {
        NRF_LOG_FLUSH();
    }
}


/** @} */
