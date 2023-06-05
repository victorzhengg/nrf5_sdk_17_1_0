/**
 * Copyright (c) 2012 - 2021, Nordic Semiconductor ASA
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
#include "sdk_common.h"

#include "ble_mnss_c.h"
#include "ble_db_discovery.h"
#include "ble_types.h"
#include "ble_gattc.h"

#include "../../../mns_control.h"

#define NRF_LOG_MODULE_NAME ble_mnss_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


/**@brief Function for handling the Disconnected event received from the SoftDevice.
 *
 * @details This function checks whether the disconnect event is happening on the link
 *          associated with the current instance of the module. If the event is happening, the function sets the instance's
 *          conn_handle to invalid.
 *
 * @param[in] p_ble_mnss_c Pointer to the Led Button Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_disconnected(ble_mnss_c_t * p_ble_mnss_c, ble_evt_t const * p_ble_evt)
{
    if (p_ble_mnss_c->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
    {
        p_ble_mnss_c->conn_handle                    = BLE_CONN_HANDLE_INVALID;
        p_ble_mnss_c->peer_mnss_db.data_write_handle  = BLE_GATT_HANDLE_INVALID;
        p_ble_mnss_c->peer_mnss_db.data_read_handle   = BLE_GATT_HANDLE_INVALID;
    }
}


void ble_mnss_on_db_disc_evt(ble_mnss_c_t * p_ble_mnss_c, ble_db_discovery_evt_t const * p_evt)
{
    // Check if the LED Button Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == MNSS_UUID_SERVICE &&
        p_evt->params.discovered_db.srv_uuid.type == p_ble_mnss_c->uuid_type)
    {
        ble_mnss_c_evt_t evt;

        evt.evt_type    = BLE_MNSS_C_EVT_DISCOVERY_COMPLETE;
        evt.conn_handle = p_evt->conn_handle;
				
        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            const ble_gatt_db_char_t * p_char = &(p_evt->params.discovered_db.charateristics[i]);
            switch (p_char->characteristic.uuid.uuid)
            {
                case MNSS_UUID_WRITE_CHAR:
                    evt.params.peer_db.data_write_handle = p_char->characteristic.handle_value;
                    break;
                case MNSS_UUID_READ_CHAR:
                    evt.params.peer_db.data_read_handle = p_char->characteristic.handle_value;
                    break;

                default:
                    break;
            }
        }

        NRF_LOG_DEBUG("LED Button Service discovered at peer.");
        //If the instance was assigned prior to db_discovery, assign the db_handles
        if (p_ble_mnss_c->conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            if ((p_ble_mnss_c->peer_mnss_db.data_write_handle == BLE_GATT_HANDLE_INVALID)&&
                (p_ble_mnss_c->peer_mnss_db.data_read_handle == BLE_GATT_HANDLE_INVALID))
            {
                p_ble_mnss_c->peer_mnss_db = evt.params.peer_db;
            }
        }

        if(p_ble_mnss_c->evt_handler != NULL)
				{
						p_ble_mnss_c->evt_handler(p_ble_mnss_c, &evt);
				}

    }
}


uint32_t ble_mnss_c_init(ble_mnss_c_t * p_ble_mnss_c, ble_mnss_c_init_t * p_ble_mnss_c_init)
{
    uint32_t      err_code;
    ble_uuid_t    mnss_uuid;
    ble_uuid128_t mnss_base_uuid = {MNSS_UUID_BASE};

    VERIFY_PARAM_NOT_NULL(p_ble_mnss_c);
    VERIFY_PARAM_NOT_NULL(p_ble_mnss_c_init);
    VERIFY_PARAM_NOT_NULL(p_ble_mnss_c_init->evt_handler);
    VERIFY_PARAM_NOT_NULL(p_ble_mnss_c_init->p_gatt_queue);

    p_ble_mnss_c->peer_mnss_db.data_write_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_mnss_c->peer_mnss_db.data_read_handle  = BLE_GATT_HANDLE_INVALID;
    p_ble_mnss_c->conn_handle                    = BLE_CONN_HANDLE_INVALID;
    p_ble_mnss_c->evt_handler                    = p_ble_mnss_c_init->evt_handler;
    p_ble_mnss_c->p_gatt_queue                   = p_ble_mnss_c_init->p_gatt_queue;
    p_ble_mnss_c->error_handler                  = p_ble_mnss_c_init->error_handler;

    err_code = sd_ble_uuid_vs_add(&mnss_base_uuid, &p_ble_mnss_c->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    VERIFY_SUCCESS(err_code);

    mnss_uuid.type = p_ble_mnss_c->uuid_type;
    mnss_uuid.uuid = MNSS_UUID_SERVICE;

    return ble_db_discovery_evt_register(&mnss_uuid);
}

uint8_t rx_buf[64];

void ble_mnss_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
		const ble_gattc_evt_read_rsp_t * p_val;
		ble_mnss_c_t * p_ble_mnss_c;
		uint16_t len;
		ble_mnss_c_evt_t evt;
			
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    p_ble_mnss_c = (ble_mnss_c_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:
            break;
				
				case BLE_GATTC_EVT_READ_RSP:				
						p_val = &p_ble_evt->evt.gattc_evt.params.read_rsp;
						len = p_val->len;
				
						evt.evt_type    = BLE_MNSS_C_EVT_READ;
						evt.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						memcpy(&evt.params.data, p_val->data, len);

						if(p_ble_mnss_c->evt_handler != NULL)
						{
								p_ble_mnss_c->evt_handler(p_ble_mnss_c, &evt);
						}
				
						break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(p_ble_mnss_c, p_ble_evt);
            break;

        default:
            break;
    }
}



uint32_t ble_mnss_c_handles_assign(ble_mnss_c_t    * p_ble_mnss_c,
                                  uint16_t         conn_handle,
                                  const mnss_db_t * p_peer_handles)
{
    VERIFY_PARAM_NOT_NULL(p_ble_mnss_c);

    p_ble_mnss_c->conn_handle = conn_handle;
    if (p_peer_handles != NULL)
    {
        p_ble_mnss_c->peer_mnss_db = *p_peer_handles;
    }
    return nrf_ble_gq_conn_handle_register(p_ble_mnss_c->p_gatt_queue, conn_handle);
}


uint32_t ble_mnss_write_data(ble_mnss_c_t * p_ble_mnss_c, ble_mnss_data_t* p_data)
{
		uint32_t error;
		
    VERIFY_PARAM_NOT_NULL(p_ble_mnss_c);

    if (p_ble_mnss_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }
		
		static uint8_t write_buf[sizeof(ble_gattc_write_params_t)];

		// Retrieve allocated data.
		ble_gattc_write_params_t write_params;
		memset(&write_params, 0, sizeof(ble_gattc_write_params_t));
		
		write_params.p_value = write_buf;
		memcpy(write_buf, p_data, sizeof(ble_gattc_write_params_t));
		
		write_params.write_op = BLE_GATT_OP_WRITE_CMD;
		write_params.handle = p_ble_mnss_c->peer_mnss_db.data_write_handle;
		write_params.len = sizeof(ble_gattc_write_params_t);

		error = sd_ble_gattc_write(p_ble_mnss_c->conn_handle, &write_params);
		//NRF_LOG_INFO("sd_ble_gattc_write: error=%d", error);
		return error;
}


uint32_t ble_mnss_read_data(ble_mnss_c_t * p_ble_mnss_c)
{
		uint32_t error;
	
    VERIFY_PARAM_NOT_NULL(p_ble_mnss_c);

    if (p_ble_mnss_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

		error = sd_ble_gattc_read(p_ble_mnss_c->conn_handle, 
			                        p_ble_mnss_c->peer_mnss_db.data_read_handle, 
															0);
		//NRF_LOG_INFO("sd_ble_gattc_read: error=%d", error);
		return error;
}
