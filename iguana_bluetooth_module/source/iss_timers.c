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

#define SECONDS_TIMER_TICKS                 APP_TIMER_TICKS(1000)

APP_TIMER_DEF(timer_seconds_awake);

uint32_t seconds_awake = 0;
bool seconds_awake_updated = false;
uint32_t mainLoopCyclesPerSecondCount = 0;
uint32_t mainLoopCyclesPerSecondCountMin = 0xFFFFFFFF;
bool testingMainLoopCycles = false;

//****************************************************************
static void timer_seconds_awake_handler(void * p_context)
{
    seconds_awake++;
    seconds_awake_updated = true;  // Do non time critical things in main, outside of interrupt context

    // For testing free time in main loop
    if (testingMainLoopCycles & (mainLoopCyclesPerSecondCount < mainLoopCyclesPerSecondCountMin))
    {
        mainLoopCyclesPerSecondCountMin = mainLoopCyclesPerSecondCount;
    }

    mainLoopCyclesPerSecondCount = 0;
}

//****************************************************************
// Initializes the timer module. This creates and starts application timers.
void timers_init(void)
{
    // Initialize timer module.
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&timer_seconds_awake, APP_TIMER_MODE_REPEATED, timer_seconds_awake_handler);
    APP_ERROR_CHECK(err_code);
}

//***************************************************************
//Function for starting application timers
void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers	
	err_code = app_timer_start(timer_seconds_awake, SECONDS_TIMER_TICKS, NULL);
    APP_ERROR_CHECK(err_code);	
}