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
#include "app_timer.h"

// ISS Files
#include "power.h"
#include "spi_lis2hh12.h"

#define ACCEL_STARTUP_TIMER_TICKS           APP_TIMER_TICKS(100)

APP_TIMER_DEF(timer_wait_for_accel_ready);

//****************************************************************
static void accel_ready_timeout_handler(void * p_context)
{
	configureAccelInterrputPin();
	configure_accel_streaming_data(ACCEL_400_HZ);
}

//****************************************************************
// Initializes the timer module. This creates and starts application timers.
void accel_startup_timer_init(void)
{
    // Create timers.
    ret_code_t err_code = app_timer_create(&timer_wait_for_accel_ready, APP_TIMER_MODE_SINGLE_SHOT, accel_ready_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

//***************************************************************
//Function for starting application timers
void accel_startup_timer_start(void)
{
    // Start application timers	
	ret_code_t err_code = app_timer_start(timer_wait_for_accel_ready, ACCEL_STARTUP_TIMER_TICKS, NULL);
    APP_ERROR_CHECK(err_code);	
}

