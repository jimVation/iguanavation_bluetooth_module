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

#ifndef SPI_LIS2HH12_H
#define SPI_LIS2HH12_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
	ACCEL_400_HZ,
	ACCEL_50_HZ,
	ACCEL_10_HZ,
	ACCEL_POWER_DOWN
} accel_sample_rates_t;

void spim_init(void);
void configure_accel_streaming_data(accel_sample_rates_t sample_rate);
void configure_accel_free_fall_detect(void);
void configure_accel_shake_detect(void);
void request_accelerometer_data(void);
void configureAccelInterrputPin(void);

extern int16_t accel_x_raw;
extern int16_t accel_y_raw;
extern int16_t accel_z_raw;

// Values converted to milli gravities
extern int16_t  accel_x_mg;
extern int16_t  accel_y_mg;
extern int16_t  accel_z_mg;

extern bool new_accel_data_ready;

#endif
