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

#include "spi_lis2hh12.h"
#include "nrfx_spim.h"
#include "nrf_drv_gpiote.h"
#include <stdint.h>
#include <string.h>		// for memset

#include "common.h"

#define NRFX_SPIM_MOSI_PIN   14
#define NRFX_SPIM_MISO_PIN   15
#define NRFX_SPIM_SS_PIN     16
#define NRFX_SPIM_SCK_PIN    18
#define ACCEL_INTERRUPT_PIN  21

#define SPI_INSTANCE  0

static const nrfx_spim_t spim_accel = NRFX_SPIM_INSTANCE(SPI_INSTANCE);

static uint8_t       m_tx_buf[8] = { 0 };    // TX buffer
static uint8_t       m_rx_buf[8] = { 0 };    // RX buffer

static uint8_t       accel_req_data_tx_buf[7] = { 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0x00 };    // TX buffer
static uint8_t       accel_req_data_rx_buf[7] = { 0 };    // RX buffer

static uint32_t received_count = 0;

// Tracks what the next accel interrupt should mean
typedef enum
{
	WAITING_FOR_ACCEL_CONFIGURED_RESPONSE,
	ACCEL_DATA_RESPONSE_EXPECTED,
	FREEFALL_CONFIG_1_DONE,
	WAITING_FOR_FREEFALL_CONFIGURED_RESPONSE,
	FREEFALL_INTERRUPT_EXPECTED,
	SHAKE_CONFIG_1_DONE,
	WAITING_FOR_SHAKE_CONFIGURED_RESPONSE,
	SHAKE_INTERRUPT_EXPECTED,
	POWER_DOWN_EXPECTED,
	NO_RESPONSE_EXPECTED
} accel_states_t;

accel_states_t accel_state = NO_RESPONSE_EXPECTED;  // accel not configured
	
void request_IG_SRC1(void);

//******************************************************************
void spim_event_handler(nrfx_spim_evt_t const * p_event, void * p_context)
{
//	uint8_t interrupt_generator_1_source;
	
	switch (accel_state)
	{
		case WAITING_FOR_ACCEL_CONFIGURED_RESPONSE:
			// Do an initial read, to reset accelerometer interrupt
			request_accelerometer_data();
			accel_state = ACCEL_DATA_RESPONSE_EXPECTED;
			break;
		
		case ACCEL_DATA_RESPONSE_EXPECTED:
			received_count++;
			
			accel_x_raw	= accel_req_data_rx_buf[2];
			accel_x_raw = accel_x_raw << 8;
			accel_x_raw |= accel_req_data_rx_buf[1];
			
			accel_y_raw	= accel_req_data_rx_buf[4];
			accel_y_raw = accel_y_raw << 8;
			accel_y_raw |= accel_req_data_rx_buf[3];
			
			accel_z_raw	= accel_req_data_rx_buf[6];
			accel_z_raw = accel_z_raw << 8;
			accel_z_raw |= accel_req_data_rx_buf[5];
    
      new_accel_data_ready = true;
			break;
		
		case FREEFALL_CONFIG_1_DONE:
			configure_accel_free_fall_detect();  // continue with second part of configuration
		    break;
		
		case WAITING_FOR_FREEFALL_CONFIGURED_RESPONSE:
			// clear the interrupt pin with a read			
			request_IG_SRC1();    // clear interrupt source register, to clear interrupt.
			accel_state = FREEFALL_INTERRUPT_EXPECTED;
		    break;
		
		case FREEFALL_INTERRUPT_EXPECTED:
			//interrupt_generator_1_source = m_rx_buf[1];	
			break;
		
		case SHAKE_CONFIG_1_DONE:
			configure_accel_shake_detect(); // continue with second part of configuration
			break;
		
		case WAITING_FOR_SHAKE_CONFIGURED_RESPONSE:
			// Make sure interrupt is cleared
			if (nrf_gpio_pin_read(ACCEL_INTERRUPT_PIN))
			{
				// clear the interrupt pin with a read			
				request_IG_SRC1();    // clear interrupt source register, to clear interrupt.
			}
			else
			{
				accel_state = SHAKE_INTERRUPT_EXPECTED;
			}
		    break;	

		case SHAKE_INTERRUPT_EXPECTED:	
			//interrupt_generator_1_source = m_rx_buf[1];	
			// Docs imply that bits will only be high if an event has occurred
			// But in testing it seems that either the H or L bit for each axis is always high
			// to reflect whether that axis is above or below the threshold at the time of the event.
			// To determine which event occurred, you would need to find the one (or ones) that meet the
			// interrupt criterion.
 			break;
		
		case POWER_DOWN_EXPECTED:
			break;
		
		default:
			break;
	}
}

//******************************************************************
void request_accelerometer_data(void)
{
	uint32_t flags = 0;
	
	nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(accel_req_data_tx_buf, sizeof(accel_req_data_tx_buf), accel_req_data_rx_buf, sizeof(accel_req_data_rx_buf));	
	APP_ERROR_CHECK(nrfx_spim_xfer(&spim_accel, &xfer_desc, flags));
}

//******************************************************************
void request_IG_SRC1(void)
{
	uint32_t flags = 0;
	m_tx_buf[0] = 0xB1;     // Read from IG_SRC1 (address 31h & 0x80h for read))
	m_tx_buf[1] = 0x00;     // empty buffer just for continuing transfer for reading back result
	
	m_rx_buf[0] = 0x00;
	m_rx_buf[1] = 0x00;
	
	nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buf, 2, m_rx_buf, 2);
	APP_ERROR_CHECK(nrfx_spim_xfer(&spim_accel, &xfer_desc, flags));
}

//******************************************************************
// Handle Accel data ready
static void handle_data_ready_event(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	if(nrf_drv_gpiote_in_is_set(pin))
	{	
		if (accel_state == ACCEL_DATA_RESPONSE_EXPECTED)
		{
			request_accelerometer_data();    // after data is returned, will be handled by spim_event_handler or SPIM0_SPIS0_IRQHandler
		}
		
		if (accel_state == FREEFALL_INTERRUPT_EXPECTED)
		{
			request_IG_SRC1();	// clear interrupt
		}	

		if (accel_state == SHAKE_INTERRUPT_EXPECTED)
		{
			request_IG_SRC1();	// clear interrupt
		}	
	}
}

//******************************************************************
void configureAccelInterrputPin(void)
{
	uint32_t err_code;
	
	if(!nrf_drv_gpiote_is_init())
	{
		err_code = nrf_drv_gpiote_init();
		APP_ERROR_CHECK(err_code);
	}	
	
	// Configure input to trigger event on low to high transition, no pull up/downs
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);

	// Initialize gpiote input
	err_code = nrf_drv_gpiote_in_init(ACCEL_INTERRUPT_PIN, &in_config, handle_data_ready_event);
	APP_ERROR_CHECK(err_code);

	// Turn on gpiote input event detection
	nrf_drv_gpiote_in_event_enable(ACCEL_INTERRUPT_PIN, true);	
}

//******************************************************************
void configure_accel_streaming_data(accel_sample_rates_t sample_rate)
{
	uint32_t flags = 0;
	uint8_t sample_rate_converted = 0x00;

	// for most cases, we will be expecting a data response
	accel_state = WAITING_FOR_ACCEL_CONFIGURED_RESPONSE;
	
	switch (sample_rate)
	{
		case ACCEL_400_HZ:
			sample_rate_converted = 0x57;	// 400Hz, BDU disabled
			break;
		
		case ACCEL_50_HZ:
			sample_rate_converted = 0x27;	// 50Hz, BDU disabled
			break;
		
		case ACCEL_10_HZ:
			sample_rate_converted = 0x17;	// 10Hz, BDU disabled
			break;	

		case ACCEL_POWER_DOWN:
			sample_rate_converted = 0x00;	// 0Hz, BDU disabled
			accel_state = POWER_DOWN_EXPECTED;
			break;

		default:
			break;		
				
	}
	// set TX buffer
	m_tx_buf[0] = 0x20;   // write to address for ctrl 1
	m_tx_buf[1] = sample_rate_converted;   // control register 1, X, Y, Z, ODR, BDU
	m_tx_buf[2] = 0x00;   // control register 2, high pass filter settings
	m_tx_buf[3] = 0x00;   // control register 3, interrupt 1
//	m_tx_buf[4] = 0x06;   // control register 4, +/-2g, auto increment address, I2C disabled
	m_tx_buf[4] = 0x36;   // control register 4, +/-8g, auto increment address, I2C disabled  
	m_tx_buf[5] = 0x00;   // control register 5
	m_tx_buf[6] = 0x01;   // control register 6, data ready signal on interrupt 2

	nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buf, 7, m_rx_buf, 7);

	APP_ERROR_CHECK(nrfx_spim_xfer(&spim_accel, &xfer_desc, flags));
}

//******************************************************************
void configure_accel_free_fall_detect(void)
{
	uint32_t flags = 0;
	
	if (accel_state != FREEFALL_CONFIG_1_DONE)
	{
		// set TX buffer
		m_tx_buf[0] = 0x20;   // write to address for ctrl 1
		m_tx_buf[1] = 0x1F;   // control register 1, X, Y, Z, enabled, ODR = 10 Hz, BDU enabled
		m_tx_buf[2] = 0x00;   // control register 2, high pass filter disabled
		m_tx_buf[3] = 0x00;   // control register 3, interrupt 1, unused
		m_tx_buf[4] = 0x06;   // control register 4, auto increment address, I2C disabled
		m_tx_buf[5] = 0x00;   // control register 5
		m_tx_buf[6] = 0x08;   // control register 6, interrupt generator 1 on interrupt 2
		m_tx_buf[7] = 0x00;   // control register 7, interrupt latched
		
		nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buf, 8, m_rx_buf, 8);
		APP_ERROR_CHECK(nrfx_spim_xfer(&spim_accel, &xfer_desc, flags));
		
		accel_state = FREEFALL_CONFIG_1_DONE;
	}
	else // accel_state = FREEFALL_CONFIG_1_DONE
	{
		m_tx_buf[0] = 0x30;   // write to address for IG_CFG1
		m_tx_buf[1] = 0x95;   // IG_CFG1, free-fall recognition (detect low event on X and Y and Z)
		m_tx_buf[2] = 0x00;   // IG_SRC1, Int gen 1 status register (read only)
		m_tx_buf[3] = 0x2D;   // IG_THS_X1, Threshold = 352 mg [ (2/256)*45 = 352 mg]
		m_tx_buf[4] = 0x2D;   // IG_THS_Y1, Threshold = 352 mg [ (2/256)*45 = 352 mg]
		m_tx_buf[5] = 0x2D;   // IG_THS_Z1, Threshold = 352 mg [ (2/256)*45 = 352 mg]
		m_tx_buf[6] = 0x03;   // IG_DUR1, no wait, duration = 3 samples event duration
		
		nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buf, 7, m_rx_buf, 7);
		APP_ERROR_CHECK(nrfx_spim_xfer(&spim_accel, &xfer_desc, flags));
		
		accel_state = WAITING_FOR_FREEFALL_CONFIGURED_RESPONSE;
	}
}

//******************************************************************
void configure_accel_shake_detect(void)
{
	uint32_t flags = 0;
	
	if (accel_state != SHAKE_CONFIG_1_DONE)
	{
		// set TX buffer
		m_tx_buf[0] = 0x20;   // write to address for ctrl 1
		m_tx_buf[1] = 0x1F;   // control register 1, X, Y, Z, enabled, ODR = 10 Hz, BDU enabled
		m_tx_buf[2] = 0x00;   // control register 2, high pass filter disabled
		m_tx_buf[3] = 0x00;   // control register 3, interrupt 1, unused
		m_tx_buf[4] = 0x06;   // control register 4, auto increment address, I2C disabled
		m_tx_buf[5] = 0x00;   // control register 5
		m_tx_buf[6] = 0x08;   // control register 6, interrupt generator 1 on interrupt 2
		m_tx_buf[7] = 0x04;   // control register 7, interrupt 1 latched
		
		nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buf, 8, m_rx_buf, 8);
		APP_ERROR_CHECK(nrfx_spim_xfer(&spim_accel, &xfer_desc, flags));
		
		accel_state = SHAKE_CONFIG_1_DONE;
	}
	else // accel_state = SHAKE_CONFIG_1_DONE
	{
		m_tx_buf[0] = 0x30;   // write to address for IG_CFG1
		m_tx_buf[1] = 0x2A;   // IG_CFG1, shake detect (detect high event on X or Y or Z)
		m_tx_buf[2] = 0x00;   // IG_SRC1, Int gen 1 status register (read only)
		m_tx_buf[3] = 0x90;   // IG_THS_X1, Threshold =  mg [ (2/256)* =  mg]
		m_tx_buf[4] = 0x90;   // IG_THS_Y1, Threshold =  mg [ (2/256)* =  mg]
		m_tx_buf[5] = 0x90;   // IG_THS_Z1, Threshold =  mg [ (2/256)* =  mg]
		m_tx_buf[6] = 0x01;   // IG_DUR1, no wait, duration = 1 sample event duration
		
		nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buf, 7, m_rx_buf, 7);
		APP_ERROR_CHECK(nrfx_spim_xfer(&spim_accel, &xfer_desc, flags));
		
		accel_state = WAITING_FOR_SHAKE_CONFIGURED_RESPONSE;
	}
}

//******************************************************************
// Note that startup time for LIS2HH12 is 20ms
void spim_init(void)
{
    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    spi_config.frequency      = NRF_SPIM_FREQ_1M;
    spi_config.ss_pin         = NRFX_SPIM_SS_PIN;
    spi_config.miso_pin       = NRFX_SPIM_MISO_PIN;
    spi_config.mosi_pin       = NRFX_SPIM_MOSI_PIN;
    spi_config.sck_pin        = NRFX_SPIM_SCK_PIN;
	spi_config.mode           = NRF_SPIM_MODE_3;
	spi_config.ss_active_high = false;	// LIS2HH12 is active when select is low
	spi_config.bit_order      = NRF_SPIM_BIT_ORDER_MSB_FIRST;
	spi_config.irq_priority   = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY;
	spi_config.orc            = 0x00;	// over run character
	
    APP_ERROR_CHECK(nrfx_spim_init(&spim_accel, &spi_config, spim_event_handler, NULL));	
}

