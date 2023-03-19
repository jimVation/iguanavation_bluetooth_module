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

#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

#include "ble.h"
#include "nrf_ble_gatt.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_nus_iss.h"
#include "ble_nus_main.h"
#include "ble_dis.h"

// dfu files
#include "ble_dfu.h"
#include "nrf_bootloader_info.h"
#include "nrf_power.h"

#include "app_error.h"
#include "app_timer.h"

// Smart Sack Files
#include "ble_service.h"
#include "ble_iss_peer_mngr.h"
#include "nrf_pwr_mgmt.h"
#include "ble_data_update.h"

// Device Information Service values
#define DIS_MANUFACTURER_NAME     "Iguanavation Inc"                      
#define DIS_FW_VERSION            "3.0"
#define DIS_HW_VERSION            "17MAR2023"
#define DIS_MODEL_NUMBER          "ISS001US"

// For ADV Interval, Apple recommends 152.5ms if their are problems (=244 * 0.625)
#define APP_ADV_INTERVAL                    128                                     /**< The advertising interval (in units of 0.625 ms). */
#define NUS_SERVICE_UUID_TYPE               BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the ISS UART Service (vendor specific). */

#define APP_ADV_DURATION                    BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising duration in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

//  Apple requires minimum connection interval >= 15 milliseconds
//  and Interval Min + 15 ms <= Interval Max
#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(15, UNIT_1_25_MS)         /**< Minimum acceptable connection interval */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(30, UNIT_1_25_MS)        /**< Maximum acceptable connection interval */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

// Expanded Module creation so it can be externed
ble_advertising_t advertising_module;               /**< Advertising module instance. */
NRF_SDH_BLE_OBSERVER(advertising_module_ble_obs, BLE_ADV_BLE_OBSERVER_PRIO, ble_advertising_on_ble_evt, &advertising_module);
NRF_SDH_SOC_OBSERVER(advertising_module_soc_obs, BLE_ADV_SOC_OBSERVER_PRIO, ble_advertising_on_sys_evt, &advertising_module);

uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

ble_advertising_init_t advertising_info;

static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

//***************************************************************
//Function for starting advertising
static void advertising_start()
{
    ret_code_t err_code = ble_advertising_start(&advertising_module, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

//***************************************************************
//Function for the GAP initialization.
// This function sets up all the necessary GAP (Generic Access Profile) parameters of the
// device including the device name, appearance, and the preferred connection parameters.
#define NAME_BYTES_LENGTH       7
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    uint8_t deviceNamePlusAddr[NAME_BYTES_LENGTH] = {"ISS"};
    uint32_t addressBytes;
    
    addressBytes = NRF_FICR->DEVICEADDR[0];
  
    deviceNamePlusAddr[3] = (uint8_t)((addressBytes >> 12) & 0x0F);
    if (deviceNamePlusAddr[3] < 10)
      deviceNamePlusAddr[3] += 0x30;
    else
      deviceNamePlusAddr[3] += 0x37;

    deviceNamePlusAddr[4] = (uint8_t)((addressBytes >> 8) & 0x0F);
    if (deviceNamePlusAddr[4] < 10)
      deviceNamePlusAddr[4] += 0x30;
    else
      deviceNamePlusAddr[4] += 0x37;

    deviceNamePlusAddr[5] = (uint8_t)((addressBytes >> 4) & 0x0F);
    if (deviceNamePlusAddr[5] < 10)
      deviceNamePlusAddr[5] += 0x30;
    else
      deviceNamePlusAddr[5] += 0x37;

    deviceNamePlusAddr[6] = (uint8_t)(addressBytes & 0x0F);
    if (deviceNamePlusAddr[6] < 10)
      deviceNamePlusAddr[6] += 0x30;
    else
      deviceNamePlusAddr[6] += 0x37;    


    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)deviceNamePlusAddr, NAME_BYTES_LENGTH);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

NRF_BLE_GATT_DEF(m_gatt);                           /**< GATT module instance. */

//***************************************************************
// GATT module event handler.
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
//        NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
//                     p_evt->conn_handle,
//                     p_evt->params.att_mtu_effective);
    }
}

//***************************************************************
//Function for initializing the GATT module.
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}

//***************************************************************
//Function for handling the Connection Parameters Module.
// *
// * @details This function will be called for all events in the Connection Parameters Module which
// *          are passed to the application.
// *          @note All this function does is to disconnect. This could have been done by simply
// *                setting the disconnect_on_fail config parameter, but instead we use the event
// *                handler mechanism to demonstrate its use.
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

//***************************************************************
// Function for handling a Connection Parameters error.
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

//***************************************************************
// Function for initializing the Connection Parameters module
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

//***************************************************************
//Function for handling BLE events.
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
//            NRF_LOG_INFO("Disconnected, reason %d.", p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            handleDisconnectForStreamingData();
            // advertising is restarted in ble_advertising_on_ble_evt()
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

//***************************************************************
//Function for initializing the BLE stack.
//Initializes the SoftDevice and the BLE event interrupt.
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

//*****************************************************************
static void sleep_mode_enter(void)
{
    uint32_t err_code;

    //Disable SoftDevice. It is required to be able to write to GPREGRET2 register (SoftDevice API blocks it).
    //GPREGRET2 register holds the information about skipping CRC check on next boot.
    err_code = nrf_sdh_disable_request();
    APP_ERROR_CHECK(err_code);
}

//*****************************************************************
// Function for handling advertising events.
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
//            NRF_LOG_INFO("Fast advertising.");
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}

//**********************************************************************
  // Initialize Device Information Service.
static void ble_device_info_service_init()
{
    ble_dis_init_t dis_init;
    uint32_t  err_code;
    uint32_t addressBytes;
    char deviceSerialNumber[13];
    
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)DIS_MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char *)DIS_FW_VERSION);
     ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char *)DIS_HW_VERSION);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)DIS_MODEL_NUMBER);
    ble_srv_ascii_to_utf8(&dis_init.sw_rev_str, (char *)DIS_FW_VERSION);
  
    addressBytes = NRF_FICR->DEVICEADDR[1];     //  Holds the top 4 characters
  
    // for the top byte, or with 1100 because top two bits must be 11 for static address
    deviceSerialNumber[0] = (uint8_t)(((addressBytes >> 12) & 0x0F) | 0x0C);
    if (deviceSerialNumber[0] < 10)
      deviceSerialNumber[0] += 0x30;
    else
      deviceSerialNumber[0] += 0x37;

    deviceSerialNumber[1] = (uint8_t)((addressBytes >> 8) & 0x0F);
    if (deviceSerialNumber[1] < 10)
      deviceSerialNumber[1] += 0x30;
    else
      deviceSerialNumber[1] += 0x37;

    deviceSerialNumber[2] = (uint8_t)((addressBytes >> 4) & 0x0F);
    if (deviceSerialNumber[2] < 10)
      deviceSerialNumber[2] += 0x30;
    else
      deviceSerialNumber[2] += 0x37;

    deviceSerialNumber[3] = (uint8_t)(addressBytes & 0x0F);
    if (deviceSerialNumber[3] < 10)
      deviceSerialNumber[3] += 0x30;
    else
      deviceSerialNumber[3] += 0x37;  

    addressBytes = NRF_FICR->DEVICEADDR[0];    // Holds the bottom 8 characters
    
    deviceSerialNumber[4] = (uint8_t)((addressBytes >> 28) & 0x0F);
    if (deviceSerialNumber[4] < 10)
      deviceSerialNumber[4] += 0x30;
    else
      deviceSerialNumber[4] += 0x37;

    deviceSerialNumber[5] = (uint8_t)((addressBytes >> 24) & 0x0F);
    if (deviceSerialNumber[5] < 10)
      deviceSerialNumber[5] += 0x30;
    else
      deviceSerialNumber[5] += 0x37;

    deviceSerialNumber[6] = (uint8_t)((addressBytes >> 20) & 0x0F);
    if (deviceSerialNumber[6] < 10)
      deviceSerialNumber[6] += 0x30;
    else
      deviceSerialNumber[6] += 0x37;

    deviceSerialNumber[7] = (uint8_t)((addressBytes >> 16) & 0x0F);
    if (deviceSerialNumber[7] < 10)
      deviceSerialNumber[7] += 0x30;
    else
      deviceSerialNumber[7] += 0x37;  

    deviceSerialNumber[8] = (uint8_t)((addressBytes >> 12) & 0x0F);
    if (deviceSerialNumber[8] < 10)
      deviceSerialNumber[8] += 0x30;
    else
      deviceSerialNumber[8] += 0x37;

    deviceSerialNumber[9] = (uint8_t)((addressBytes >> 8) & 0x0F);
    if (deviceSerialNumber[9] < 10)
      deviceSerialNumber[9] += 0x30;
    else
      deviceSerialNumber[9] += 0x37;

    deviceSerialNumber[10] = (uint8_t)((addressBytes >> 4) & 0x0F);
    if (deviceSerialNumber[10] < 10)
      deviceSerialNumber[10] += 0x30;
    else
      deviceSerialNumber[10] += 0x37;

    deviceSerialNumber[11] = (uint8_t)(addressBytes & 0x0F);
    if (deviceSerialNumber[11] < 10)
      deviceSerialNumber[11] += 0x30;
    else
      deviceSerialNumber[11] += 0x37;     

    deviceSerialNumber[12] = 0; // null terminator for string    
    
    ble_srv_ascii_to_utf8(&dis_init.serial_num_str, deviceSerialNumber);

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

//****************************************************************
static void advertising_init(void)
{
    ret_code_t   err_code;
    int8_t tx_power = 0;

    uint8_t temp_buff[MFG_DATA_BYTES_SIZE] = {0x00};
    ble_advdata_manuf_data_t    smart_sack_mfg_info;
    smart_sack_mfg_info.data.size = MFG_DATA_BYTES_SIZE;
    smart_sack_mfg_info.company_identifier = 0x0733;
    smart_sack_mfg_info.data.p_data = &temp_buff[0];

    memset(&advertising_info, 0, sizeof(ble_advertising_init_t));

// Primary advertising data	
    advertising_info.advdata.include_appearance      = false;
    advertising_info.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE; // LE General Discoverable Mode, BR/EDR not supported
    advertising_info.advdata.uuids_more_available.uuid_cnt   = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advertising_info.advdata.uuids_more_available.p_uuids    = m_adv_uuids;
// End primary data
    
// Scan response advertising data
    advertising_info.srdata.p_manuf_specific_data = &smart_sack_mfg_info;
    advertising_info.srdata.name_type            = BLE_ADVDATA_FULL_NAME;
    advertising_info.srdata.p_tx_power_level     = &tx_power;
// End scan response data

    advertising_info.config.ble_adv_fast_enabled  = true;
    advertising_info.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    advertising_info.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    advertising_info.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&advertising_module, &advertising_info);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&advertising_module, APP_BLE_CONN_CFG_TAG);
}

//****************************************************************
static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled  = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout  = APP_ADV_DURATION;
}

//****************************************************************
static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
}

//****************************************************************
void set_advertsing_power(int8_t advertisingPower)
{   // I tried testing this but I can't tell that it's working
    sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, advertising_module.adv_handle, advertisingPower);
}

//****************************************************************
static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

//****************************************************************
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};

//****************************************************************
//Function for handling dfu events from the Buttonless Secure DFU service
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
//            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

            // Prevent device from advertising on disconnect.
            ble_adv_modes_config_t config;
            advertising_config_get(&config);
            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&advertising_module, &config);

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
//            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
//            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
//            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
//            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
//            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}

#if (BLE_DFU_ENABLED == 1)
//***************************************************************
//Function for initializing services that will be used by the application.
static void dfu_service_init(void)
{
    uint32_t                  err_code;
    ble_dfu_buttonless_init_t dfus_init = {0};

    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
}
#endif

//***************************************************************
void start_ble_system(void)
{
    ble_stack_init();
    ble_nus_services_init();
    ble_device_info_service_init();
    peer_manager_init();
    gap_params_init();
    gatt_init();
    advertising_init();	
#if (BLE_DFU_ENABLED == 1)
    dfu_service_init();
#endif
    conn_params_init();	    
    advertising_start();
}
