/**
 * Copyright (c) 2023, Iguanavation, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Neither the name of Iguanavation, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Iguanavation, Inc. "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Iguanavation, Inc. OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdint.h>

#include "app_error.h"
#include "nrf_gpio.h"
#include "ble_dfu.h"

// ISS Files
#include "common.h"
#include "spi_lis2hh12.h"
#include "iss_timers.h"
#include "power.h"
#include "ble_service.h"
#include "ble_data_update.h"

//****************************************************************
int main(void)
{
    ret_code_t err_code;

#if (BLE_DFU_ENABLED == 1)
    // Initialize the async SVCI interface to bootloader before any interrupts are enabled
    err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);
#endif

    // Initialize LED
    nrf_gpio_cfg_output(LED_PIN);  
    nrf_gpio_pin_clear(LED_PIN);

    // Create a timer to wait for acceleremeter to wake up and a timer for seconds awake
    timers_init();
    power_management_init();
    spim_init();
    start_ble_system();

    // Start the timers
    // The accelereometer configuration will happen when the accelerometer wakeup timer expires
    application_timers_start();

    // Enter main loop.
    for (;;)
    {   
        if (new_accel_data_ready)
        {        
            // convert raw to mg. 1 g = 4096. *1000 for mg.
            // Do *1000 first to avoid losing data.
            // Divide by 4096 by using >> 12. 
            accel_x_mg = (int16_t)((((int32_t)accel_x_raw) * 1000) >> 12);        
            accel_y_mg = (int16_t)((((int32_t)accel_y_raw) * 1000) >> 12);
            accel_z_mg = (int16_t)((((int32_t)accel_z_raw) * 1000) >> 12);
          
            update_ble_data();

            new_accel_data_ready = false;
        }

        // Look for a seconds tick (updated in the timer interrupt)
        if (seconds_awake_updated)
        { 
            seconds_awake_updated = false;
            
            // Blink LED
            nrf_gpio_pin_toggle(LED_PIN);  

            // Notify power system of time change and do any time based events
            update_power_management(1); // pass in 1 as number of seconds since last update

            update_ble_data();
        }

        idle_state_handle();
        mainLoopCyclesPerSecondCount++;
    }
}

