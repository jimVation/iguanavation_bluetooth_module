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

// Nordic SDK files
#include "app_error.h"
#include "ble_advdata.h"
#include "ble_conn_state.h"
#include "nrf_soc.h"

// Local App files
#include "ble_data_update.h"
#include "ble_service.h"
#include "spi_lis2hh12.h"
#include "ble_nus_main.h"

#define NUS_DATA_BUFFER_LENGTH     240
#define NUS_HEADER_LENGTH   2
#define NUS_FOOTER_LENGTH   1

// data configuration variables
bool transmitAccelDataEnabled = false;

// Two sets of buffers (ping pong) needed for dynamically changing advertising data
static uint8_t enc_adv_data_buffer1[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
static uint8_t enc_adv_data_buffer2[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
static uint8_t enc_scan_rsp_buffer1[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
static uint8_t enc_scan_rsp_buffer2[BLE_GAP_ADV_SET_DATA_SIZE_MAX];

//****************************************************************
static ble_gap_adv_data_t advertisingScanResponseBuffer[2] = 
{
	{ 
		.scan_rsp_data.p_data = enc_scan_rsp_buffer1,
		.scan_rsp_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX,
		.adv_data.p_data = enc_adv_data_buffer1,
		.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX
	},
	{ 
		.scan_rsp_data.p_data = enc_scan_rsp_buffer2,
		.scan_rsp_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX,
		.adv_data.p_data = enc_adv_data_buffer2,
		.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX
	},
};

//****************************************************************
void handleDisconnectForStreamingData(void)
{   // turn off streaming on disconnect
    transmitAccelDataEnabled = false;
}

//****************************************************************
void transmitAccelData(void)
{
	static uint8_t dataPointer = 0;
    uint8_t dataBytesAdded = 0;
    static uint8_t error_buff[4] = { 0x03, 0x02, 0x00, 0x05 };  // first byte 0x03 means error, 0x02 is data length (with checksum), 0x05 is checksum
    uint16_t error_buff_length = 4;

    uint8_t checksum = 0;
	
    static uint16_t temp_buff_length = NUS_DATA_BUFFER_LENGTH + NUS_HEADER_LENGTH + NUS_FOOTER_LENGTH;
	static uint8_t temp_buff[NUS_DATA_BUFFER_LENGTH + NUS_HEADER_LENGTH + NUS_FOOTER_LENGTH];

    if (dataPointer == 0)
    {   // Add Header
        temp_buff[0] = 0x02; // 0x02 means data
        dataPointer = NUS_HEADER_LENGTH;     // skip the length byte for now
    }
	
	// Get latest accel data
    temp_buff[dataPointer++] = accel_x_mg >> 8;
    temp_buff[dataPointer++] = accel_x_mg & 0x00FF;
    temp_buff[dataPointer++] = accel_y_mg >> 8;
    temp_buff[dataPointer++] = accel_y_mg & 0x00FF;
    temp_buff[dataPointer++] = accel_z_mg >> 8;
    temp_buff[dataPointer++] = accel_z_mg & 0x00FF;

	dataBytesAdded = dataPointer - NUS_HEADER_LENGTH;

    // Check for data count error
    if (dataBytesAdded > NUS_DATA_BUFFER_LENGTH)
    {
        ble_data_send_wrapper(error_buff, &error_buff_length);
    }
	else if (dataBytesAdded == NUS_DATA_BUFFER_LENGTH)
	{   // Buffer is full. Transmit it.
        temp_buff[1] = dataBytesAdded;

        // Calculate checksum
        checksum = 0;
        while (dataPointer !=0)
            checksum += temp_buff[--dataPointer];

        temp_buff[temp_buff_length - 1] = checksum;  // Checksum goes here, in last byte
        ble_data_send_wrapper(temp_buff, &temp_buff_length);
	}
}

//****************************************************************
void updateAdvertisingData(void)
{
	static uint8_t bufferIndex = 0;
    ret_code_t errCode;
    int32_t temperature_c_0_25_increments;

    // Get core temperature
    sd_temp_get(&temperature_c_0_25_increments);
	
	static ble_advdata_manuf_data_t    jaet2l_mfg_info;
	static uint8_t temp_buff[MFG_DATA_BYTES_SIZE];
	
	// Get latest data
	temp_buff[0] = accel_x_raw >> 8;
	temp_buff[1] = accel_x_raw & 0x00FF;
	temp_buff[2] = accel_y_raw >> 8;
	temp_buff[3] = accel_y_raw & 0x00FF;
	temp_buff[4] = accel_z_raw >> 8;
	temp_buff[5] = accel_z_raw & 0x00FF;
	temp_buff[6] = 0x00;
	temp_buff[7] = 0x00;
	temp_buff[8] = (uint8_t)temperature_c_0_25_increments;
	
	jaet2l_mfg_info.company_identifier = 0x0733;
	jaet2l_mfg_info.data.size = MFG_DATA_BYTES_SIZE;
	jaet2l_mfg_info.data.p_data = &temp_buff[0];

	// Load latest accel data into the advertising data buffer
	advertising_info.srdata.p_manuf_specific_data  = &jaet2l_mfg_info;	
	
	// Load advertising data buffer into the advertising info structure
	errCode = ble_advdata_encode( &advertising_info.srdata, advertisingScanResponseBuffer[bufferIndex].scan_rsp_data.p_data, (uint16_t *)&advertisingScanResponseBuffer[bufferIndex].scan_rsp_data.len );
	APP_ERROR_CHECK(errCode);
    errCode = ble_advdata_encode( &advertising_info.advdata, advertisingScanResponseBuffer[bufferIndex].adv_data.p_data, (uint16_t *)&advertisingScanResponseBuffer[bufferIndex].adv_data.len );
	APP_ERROR_CHECK(errCode);
	
	// Send full advertising info structure to radio
	errCode = sd_ble_gap_adv_set_configure(&advertising_module.adv_handle, &advertisingScanResponseBuffer[bufferIndex], NULL);
	APP_ERROR_CHECK( errCode );
	
	bufferIndex++;
	bufferIndex = bufferIndex % 2;
}

// ***************************************************************
void update_ble_data(void)
{
    if (ble_conn_state_status(m_conn_handle) == BLE_CONN_STATUS_CONNECTED)
    {   // Things to do while connected
        if (transmitAccelDataEnabled)
        {
            transmitAccelData();
        }
    }
    else
    {   // Things to do when not connected
        updateAdvertisingData();
    }
}

