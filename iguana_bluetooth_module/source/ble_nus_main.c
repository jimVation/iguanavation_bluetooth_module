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

#include "ble_nus_iss.h"
#include "ble_nus_main.h"
#include "ble_service.h"
#include "app_error.h"

#include "ble_data_update.h"
#include "power.h"

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);

char outMessage[51];
uint16_t outMessageLength;

void ble_text_send_wrapper(void);

//****************************************************************************
//This function will process the data received from the Nordic UART BLE Service and send it to the UART module.
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
	uint16_t data_length;
//  static uint8_t errors = 0;
	
	// Handle Incoming Data
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {      
      	data_length = p_evt->params.rx_data.length;
      
        // Check for at least one byte to process
        if (!data_length)
        {
          return;
        }
        
        uint8_t data[data_length];
  
        for (uint32_t i = 0; i < data_length; i++)
        {
            data[i] = p_evt->params.rx_data.p_data[i];
        }
        
        //  Super simple message parsing till we need something better
        switch (data[0])
        {
            case 'a':
                streamDataEnabled = (data[1] - 0x30);

                if (streamDataEnabled)
                {
                    outMessageLength = sprintf(outMessage, "Data streaming on"); 
                }
                else
                {
                    outMessageLength = sprintf(outMessage, "Data streaming off");
                }
                break;
            
            case 'c':
                set_advertsing_power(4);
                sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, 4);
                outMessageLength = sprintf(outMessage, "Tx_power set to 4");  
                break;   

            case 'd':
                set_advertsing_power(0);
                sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, 0);
                outMessageLength = sprintf(outMessage, "Tx_power set to 0");   
                break;            
            
            case 'e':
                set_advertsing_power(-40);
                sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, -40);
                outMessageLength = sprintf(outMessage, "Tx_power set to -40");
                break;
            
            case 'f':  // Change go to sleep time
                if (data_length != 4)
                    return;
                
                inactivityTimeLimitSeconds = ((data[1] - 0x30) * 100) + ((data[2] - 0x30) * 10) + (data[3] - 0x30);
                outMessageLength = sprintf(outMessage, "Timeout set to %u seconds", inactivityTimeLimitSeconds);
                break;
            
            case 'g':  // Go to Sleep
                inactivityTimeLimitSeconds = 1;
                outMessageLength = sprintf(outMessage, "Sleep now");
                break;
                             
            case 'm':
                inactivityTimeLimitSeconds = UINT32_MAX;
                outMessageLength = sprintf(outMessage, "Life test mode enabled");            
                break;
            
            default:
                outMessageLength = sprintf(outMessage, "Invalid");
                break;
        }	

        if (outMessageLength != 0)
            ble_text_send_wrapper();
    }
}

//******************************************************************************
void ble_text_send_wrapper(void)
{
    char tempBuffer[50]; 

    memcpy(&tempBuffer[2], outMessage, outMessageLength);
    tempBuffer[0] = 0x01;              // text type response
    tempBuffer[1] = outMessageLength;  // length of data section without header and footer
    tempBuffer[outMessageLength + 2] = 0xFF;  // checksum goes here
    
    outMessageLength += 3;  // add in count for header and footer
    ble_nus_data_send(&m_nus, (uint8_t *)&tempBuffer, &outMessageLength, m_conn_handle);
}

//******************************************************************************
void ble_data_send_wrapper(uint8_t* transmit_data, uint16_t* transmit_data_length)
{
    ble_nus_data_send(&m_nus, transmit_data, transmit_data_length, m_conn_handle);
}


//******************************************************************************
void ble_nus_services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

