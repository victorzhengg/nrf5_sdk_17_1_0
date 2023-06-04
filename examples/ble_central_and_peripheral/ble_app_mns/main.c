/**
 * Copyright (c) 2015 - 2021, Nordic Semiconductor ASA
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
/**
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_mnss.h"
#include "ble_mnss_c.h"
#include "ble_db_discovery.h"

#include "nrf_ble_scan.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "mns_control.h"
#include "ble_conn_state.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define GATT_CLIENT_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define GATT_SERVER_LED_1               BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define GATT_SERVER_LED_2               BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
#define MNSS_LED                        BSP_BOARD_LED_3                         /**< LED to be toggled with the help of the LED Button Service. */

#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME                     "Nordic_MNS"                         /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(8, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(16, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */




#define NRF_BLE_GQ_QUEUE_SIZE 4
#define MNS_TIMER_PERIOD      APP_TIMER_TICKS(20)       /*multi node synchronize */
#define MNSS_THRESHOLD   5
#define LED_ON_DELAY     APP_TIMER_TICKS(100)           /*duty of led on */
#define FICR_DEVICE_ADDR   ((uint32_t*)0x100000A4)

BLE_MNSS_DEF(m_mnss);                                                             /**< LED Button Service instance. */
BLE_MNSS_C_DEF(m_ble_mnss_c); 
BLE_DB_DISCOVERY_DEF(m_db_disc);  

NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);
NRF_BLE_SCAN_DEF(m_scan);  

APP_TIMER_DEF(m_mns_timer);
APP_TIMER_DEF(m_led_delay_timer);



static char const m_target_periph_name[] = "Nordic_MNS";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/
static uint16_t m_conn_handle[NRF_SDH_BLE_PERIPHERAL_LINK_COUNT] = {0};         /**< Handle of the current connection. */

static ble_mnss_data_t m_mnss_data = {
														   .sn =0xFFFF0001,           /**< serial number */
															 .counter_value = 0,        /**< counter value */
															 .period = 100,             /**< period value  */
															};
uint32_t local_mns_cnt = 0;
uint32_t remote_mns_cnt = 0;
static uint8_t sync_enable_flag = 0;															
static mns_control_t 	m_mns_control;														
															
															

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    ble_uuid_t adv_uuids[] = {{MNSS_UUID_SERVICE, m_mnss.uuid_type}};

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}



/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void mnss_write_handler(uint16_t conn_handle, ble_mnss_t* p_mnss, ble_mnss_data_t * p_data)
{
		static ble_mnss_data_t data;
		uint16_t len = sizeof(ble_mnss_data_t);
		uint32_t cnt_error;
	
		NRF_LOG_INFO("mnss_write_handler");
		NRF_LOG_INFO("mnss_write_handler: conn_handle = %x", conn_handle);
	
		memcpy(&data, p_data, len);
		NRF_LOG_INFO("data:");
		NRF_LOG_INFO("SN:%X, CNT:%X, PERIOD:%X", data.sn, data.counter_value, data.period);
		if(data.sn < m_mnss_data.sn)
		{
				if(data.counter_value > m_mnss_data.counter_value)
				{
						cnt_error = data.counter_value - m_mnss_data.counter_value;
				}
				else
				{
						cnt_error = m_mnss_data.counter_value - data.counter_value;
				}
				if(cnt_error > MNSS_THRESHOLD)
				{
						local_mns_cnt = data.counter_value;
				}
		}
}




/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
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


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t           err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
/*
static void advertising_stop(void)
{
    ret_code_t           err_code;

    err_code = sd_ble_gap_adv_stop(m_adv_handle);
    APP_ERROR_CHECK(err_code);
}
*/


/**@brief Function for handling the Connected event.
 *
 * @param[in] p_gap_evt GAP event received from the BLE stack.
 */
static void peripheral_on_connected(const ble_gap_evt_t * const p_gap_evt)
{
    uint32_t m_periph_link_cnt = 0;
		mns_node_t node;
		uint32_t error;

    m_periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.
	
		NRF_LOG_INFO("role peripheral was connected. address:");
		NRF_LOG_HEXDUMP_INFO(p_gap_evt->params.connected.peer_addr.addr, 6);	
		NRF_LOG_INFO("connect handle = 0x%x. total connection = %d", p_gap_evt->conn_handle, m_periph_link_cnt);
	  
		memset(&node, 0, sizeof(mns_node_t));
		node.conn_handle = p_gap_evt->conn_handle;
	  memcpy(&(node.peer_addr), 
	         &(p_gap_evt->params.connected.peer_addr), 
	         sizeof(ble_gap_addr_t));
	
	  error = mns_control_add_node(&m_mns_control, &node);
	  if(error != 0)
		{
				NRF_LOG_INFO("fail to add node to mns");
		}
	
    switch (m_periph_link_cnt)
		{
        case 1:
						m_conn_handle[0] = p_gap_evt->conn_handle;
						bsp_board_led_on(GATT_SERVER_LED_1);
						bsp_board_led_off(GATT_SERVER_LED_2);
						advertising_start();
            break;
				
        case 2: /*NRF_SDH_BLE_PERIPHERAL_LINK_COUNT*/
						m_conn_handle[1] = p_gap_evt->conn_handle;
						bsp_board_led_on(GATT_SERVER_LED_1);
						bsp_board_led_on(GATT_SERVER_LED_2);
            break;
				
        default:
            // No implementation needed.
            break;			
		}
}


/**@brief Function for handling the Disconnected event.
 *
 * @param[in] p_gap_evt GAP event received from the BLE stack.
 */
static void peripheral_on_disconnected(ble_gap_evt_t const * const p_gap_evt)
{
		uint32_t m_periph_link_cnt = 0;
		mns_node_t node;
	
    m_periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.

		NRF_LOG_INFO("peripheral Connection 0x%x has been disconnected. Reason: 0x%X, total:%d",
                 p_gap_evt->conn_handle,
                 p_gap_evt->params.disconnected.reason,
								 m_periph_link_cnt);

		mns_control_delete_node(&m_mns_control, p_gap_evt->conn_handle);
	
    switch (m_periph_link_cnt)
		{
        case 0:
						m_conn_handle[0] = BLE_CONN_HANDLE_INVALID;
						m_conn_handle[1] = BLE_CONN_HANDLE_INVALID;
						bsp_board_led_off(GATT_SERVER_LED_1);
						bsp_board_led_off(GATT_SERVER_LED_2);				
            break;
				
        case 1:
						if(p_gap_evt->conn_handle == m_conn_handle[0])
						{
								m_conn_handle[0] = m_conn_handle[1];
								m_conn_handle[1] = BLE_CONN_HANDLE_INVALID;
						}
						else if(p_gap_evt->conn_handle == m_conn_handle[1])
						{
								m_conn_handle[1] = BLE_CONN_HANDLE_INVALID;
						}
						bsp_board_led_on(GATT_SERVER_LED_1);
						bsp_board_led_off(GATT_SERVER_LED_2);
						advertising_start();
            break;				
				
        default:
            // No implementation needed.
            break;			
		}	
}

static void scan_start(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);
}

static void central_ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        // discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:
        {
						NRF_LOG_INFO("central_ble_evt_handler: Connected. address:");
						NRF_LOG_HEXDUMP_INFO(p_gap_evt->params.connected.peer_addr.addr, 6);
						NRF_LOG_INFO("central_ble_evt_handler: Connected. handle:%d", p_gap_evt->conn_handle);
					
            err_code = ble_mnss_c_handles_assign(&m_ble_mnss_c, p_gap_evt->conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_db_disc, p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);

						mns_node_t node;
	  
						memset(&node, 0, sizeof(mns_node_t));
						node.conn_handle = p_gap_evt->conn_handle;
						memcpy(&(node.peer_addr), 
									 &(p_gap_evt->params.connected.peer_addr), 
									 sizeof(ble_gap_addr_t));
	
						err_code = mns_control_add_node(&m_mns_control, &node);
						if(err_code != 0)
						{
								NRF_LOG_INFO("fail to add node to mns");
						}
						
            // Update LEDs status, and check if we should be looking for more
            // peripherals to connect to.
            bsp_board_led_on(GATT_CLIENT_LED);
        } break;

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
						NRF_LOG_INFO("central_ble_evt_handler: Disconnected.");
					
						mns_control_delete_node(&m_mns_control, p_gap_evt->conn_handle);
					
						bsp_board_led_off(GATT_CLIENT_LED);
					
            scan_start();
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("central_ble_evt_handler: Connection request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("central_ble_evt_handler: PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("central_ble_evt_handler: GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("central_ble_evt_handler: GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }	
}
static void peripheral_ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            peripheral_on_connected(&p_ble_evt->evt.gap_evt);

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            peripheral_on_disconnected(&p_ble_evt->evt.gap_evt);
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gap_evt.conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    uint16_t role        = ble_conn_state_role(conn_handle);

    // Based on the role this device plays in the connection, dispatch to the right handler.
    if (role == BLE_GAP_ROLE_PERIPH)
    {
        peripheral_ble_evt_handler(p_ble_evt, p_context);
    }
    else if (role == BLE_GAP_ROLE_CENTRAL)
    {
        central_ble_evt_handler(p_ble_evt, p_context);
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
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

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
  switch (pin_no)
    {
        case BSP_BUTTON_0:
				    if(button_action == 0)
						{
								NRF_LOG_INFO("enable the synchronize between multi nodes");
								sync_enable_flag = 1;
						}
            break;
						
        case BSP_BUTTON_1:
				    if(button_action == 0)
						{
								NRF_LOG_INFO("enable the synchronize between multi nodes");
								sync_enable_flag = 0;
						}
            break;
						
        case BSP_BUTTON_2:
				    if(button_action == 0)
						{
								NRF_LOG_INFO("button_event_handler: BSP_BUTTON_2");
						}
            break;

        case BSP_BUTTON_3:
				    if(button_action == 0)
						{
								NRF_LOG_INFO("button_event_handler: BSP_BUTTON_3");
						}
            break;
						
        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {BSP_BUTTON_0, false, BUTTON_PULL, button_event_handler},
				{BSP_BUTTON_1, false, BUTTON_PULL, button_event_handler},
				{BSP_BUTTON_2, false, BUTTON_PULL, button_event_handler},
				{BSP_BUTTON_3, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}


static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static void led_delay_timer_handler(void * p_context)
{
		bsp_board_led_off(MNSS_LED);
}


static void mns_timer_handler(void * p_context)
{
		ble_gatts_value_t     gatt_value;
	  uint16_t index;
	
		local_mns_cnt++;
		
		if((local_mns_cnt % m_mnss_data.period) == 0)
		{
				bsp_board_led_on(MNSS_LED);
				app_timer_start(m_led_delay_timer, LED_ON_DELAY, NULL);
		}
		
	  m_mnss_data.counter_value = local_mns_cnt;
	

		
		if(sync_enable_flag == 1)
		{
/*
				gatt_value.len = sizeof(ble_mnss_data_t);
				gatt_value.p_value = (uint8_t *)&m_mnss_data;
				gatt_value.offset = 0;
				for(index=0;index<m_periph_link_cnt;index++)
				{
						sd_ble_gatts_value_set(m_conn_handle[index],
																	 m_mnss.data_read_handle.value_handle, 
																	 &gatt_value);
				}
*/
		}
}

/**@brief Function for handling Scaning events.
 *
 * @param[in]   p_scan_evt   Scanning event.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
            break;
        default:
          break;
    }
}

static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    // Setting filters for scanning.
    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
    APP_ERROR_CHECK(err_code);	
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_mnss_on_db_disc_evt(&m_ble_mnss_c, p_evt);
}


static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(db_init));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);	
}




/**@brief Handles events coming from the LED Button central module.
 */
static void mnss_c_evt_handler(ble_mnss_c_t * p_mnss_c, ble_mnss_c_evt_t * p_mnss_c_evt)
{
    switch (p_mnss_c_evt->evt_type)
    {
        case BLE_MNSS_C_EVT_DISCOVERY_COMPLETE:
        {
            ret_code_t err_code;

            err_code = ble_mnss_c_handles_assign(&m_ble_mnss_c,
                                                p_mnss_c_evt->conn_handle,
                                                &p_mnss_c_evt->params.peer_db);
						APP_ERROR_CHECK(err_code);
					
						NRF_LOG_INFO("mnss_c_evt_handler: BLE_LBS_C_EVT_DISCOVERY_COMPLETE");
        } break; // BLE_LBS_C_EVT_DISCOVERY_COMPLETE

        case BLE_MNSS_C_EVT_WRITE:
        {
            NRF_LOG_INFO("BLE_MNSS_C_EVT_WRITE.");
 
        } break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION

				case BLE_MNSS_C_EVT_READ:
        {
            NRF_LOG_INFO("BLE_MNSS_C_EVT_READ");
 
        } break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION
				
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling the LED Button Service client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void mnss_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_mnss_init_t    init     = {0};
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize LBS.
    init.data_write_handler = mnss_write_handler;

		
		/*peripheral service initial*/
    err_code = ble_mnss_init(&m_mnss, &init);
    APP_ERROR_CHECK(err_code);
		
		
		/*central service initial*/
    ble_mnss_c_init_t mnss_c_init_obj;

    mnss_c_init_obj.evt_handler   = mnss_c_evt_handler;
    mnss_c_init_obj.p_gatt_queue  = &m_ble_gatt_queue;
    mnss_c_init_obj.error_handler = mnss_error_handler;

    err_code = ble_mnss_c_init(&m_ble_mnss_c, &mnss_c_init_obj);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for application main entry.
 */
int main(void)
{
		uint32_t err_code;
    // Initialize.
    log_init();
    leds_init();
	  
		m_mnss_data.sn = *FICR_DEVICE_ADDR;
		NRF_LOG_INFO("Multi Node Synchronize example started. SN: %X", m_mnss_data.sn);
	
    timers_init();
		app_timer_create(&m_mns_timer, APP_TIMER_MODE_REPEATED, mns_timer_handler);
	  app_timer_create(&m_led_delay_timer, APP_TIMER_MODE_SINGLE_SHOT, led_delay_timer_handler);
	
		app_timer_start(m_mns_timer, MNS_TIMER_PERIOD, NULL);
	
    buttons_init();
		err_code = app_button_enable();
		APP_ERROR_CHECK(err_code);
	
    power_management_init();
	
		mns_control_init(&m_mns_control);
	
    ble_stack_init();
		scan_init();
    gap_params_init();
    gatt_init();
		db_discovery_init();
		
    services_init();
    advertising_init();
    conn_params_init();
		
    

    // Start execution.
    
		scan_start();
    advertising_start();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
