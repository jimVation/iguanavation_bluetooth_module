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

#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_gpio.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define GO_TO_SLEEP_SECONDS 60
#if (GO_TO_SLEEP_SECONDS < 20)
#error Awake Time to short to allow for OTA updates
#endif

typedef enum
{
	POWER_STATE_ON,
	POWER_STATE_GOING_TO_SLEEP,
    POWER_STATE_WAIT_FOR_SLEEP,
} power_states_t;

uint32_t inactivity_timer_seconds = 0;
uint32_t inactivityTimeLimitSeconds = GO_TO_SLEEP_SECONDS;  // seconds of inactivity (no kicks) until we go to sleep


//******************************************************************
void power_management_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

//******************************************************************
// Function for handling the idle state (main loop).
// If there is no pending log operation, then sleep until the next event occurs.
void idle_state_handle(void)
{
    nrf_pwr_mgmt_run();
}

//******************************************************************
void reset_inactivity_timer(void)
{
	inactivity_timer_seconds = 0;
}

//******************************************************************
void update_power_management(uint8_t seconds_since_last_update)
{
	static power_states_t power_state = POWER_STATE_ON;
	
	inactivity_timer_seconds += seconds_since_last_update;
	
	switch(power_state)
	{
		case POWER_STATE_ON:
			if (inactivity_timer_seconds > inactivityTimeLimitSeconds)
			{
				power_state = POWER_STATE_GOING_TO_SLEEP;
			}
			break;
		
		case POWER_STATE_GOING_TO_SLEEP:
            // Turn off the LED
            nrf_gpio_pin_set(LED_PIN);
            nrf_gpio_cfg_default(LED_PIN);  // disconnect pin to minimize current 

            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);	// system off mode	
            power_state = POWER_STATE_WAIT_FOR_SLEEP;
            break;

        case POWER_STATE_WAIT_FOR_SLEEP:
            break;
	}
}

//******************************************************************
// During shutdown procedures, this function will be called at a 1 second interval
// untill the function returns true. When the function returns true, it means that the
// app is ready to reset to DFU mode.
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    bool m_ready_for_reset = true;

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            if (!m_ready_for_reset)
            {
                return false;
            }
            else
            {
                // Device ready to enter
                uint32_t err_code;
                err_code = nrf_sdh_disable_request();
                APP_ERROR_CHECK(err_code);
                err_code = app_timer_stop_all();
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            return true;
    }

    return true;
}

//Register application shutdown handler with priority 0.
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);


