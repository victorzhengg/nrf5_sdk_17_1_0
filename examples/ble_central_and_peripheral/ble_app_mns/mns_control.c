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
 * @brief multi node synchronize control
 *
 * This module enables the application to synchromize the timer in each node
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "mns_control.h"
#include "ble_mnss_c.h"


#define NRF_LOG_MODULE_NAME mns_control
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/**@brief Function for initialize the globle variables m_mns_control
 *
 * @param[in] 
 */
mns_node_t* mns_control_init(mns_control_t* p_mns_control,		
                             ble_mnss_t*    p_peripheral_service,
		                         ble_mnss_c_t*  p_central_service)
{
		mns_node_t* p_node= NULL;
		uint16_t index;
	  
		memset(p_mns_control, 0, sizeof(mns_control_t));
	  p_mns_control->p_peripheral_service = p_peripheral_service;
	  p_mns_control->p_central_service = p_central_service;
	
		p_mns_control->local_data.device_sn = *FICR_DEVICE_ADDR;
		p_mns_control->local_data.sn = p_mns_control->local_data.device_sn;
		p_mns_control->local_data.period = MNS_CONTROL_LED_PERIOD;
	
		for(index=0;index<MNS_MAX_NODE_NUM;index++)
		{
				p_mns_control->remote_node[index].conn_handle = BLE_CONN_HANDLE_INVALID;
		}
		return 	p_node;	
}

/**@brief Function for find the node according the connect handle
 *
 * @param[in] 
 */
mns_node_t* mns_control_find_node(mns_control_t* p_mns_control, uint16_t conn_handle)
{
		mns_node_t* p_node= NULL;
		uint16_t index;
	  
		for(index=0;index<MNS_MAX_NODE_NUM;index++)
		{
				if(p_mns_control->remote_node[index].conn_handle == conn_handle)
				{
						p_node = &(p_mns_control->remote_node[index]);
				}
		}
		return 	p_node;	
}

/**@brief Function for find the node according the connect handle
 *
 * @param[in] 
 */
uint32_t mns_control_add_node(mns_control_t* p_mns_control, mns_node_t* p_node)
{
		uint16_t index;
		uint32_t error = 1;
	
	  for(index=0;index<MNS_MAX_NODE_NUM;index++)
		{
				if(p_mns_control->remote_node[index].conn_handle == BLE_CONN_HANDLE_INVALID)
				{
						memcpy(&(p_mns_control->remote_node[index]), p_node, sizeof(mns_node_t));
						error = 0;
						NRF_LOG_INFO("mns_control_add_node:add handle %d to index %d",
					               p_mns_control->remote_node[index].conn_handle,
					               index);
						break;
				}
		}
		
		if(p_mns_control->total_connection < MNS_MAX_NODE_NUM)
		{
				p_mns_control->total_connection++;
				NRF_LOG_INFO("mns_control_add_node:total_connection = %d", p_mns_control->total_connection);
		}
		
		return error;
}

/**@brief Function for delete the node according the connect handle
 *
 * @param[in] 
 */
uint32_t mns_control_delete_node(mns_control_t* p_mns_control, uint16_t conn_handle)
{
		uint16_t index;
		uint32_t error = 1;
	
	  for(index=0;index<MNS_MAX_NODE_NUM;index++)
		{
				if(p_mns_control->remote_node[index].conn_handle == conn_handle)
				{
						memset(&(p_mns_control->remote_node[index]), 0, sizeof(mns_node_t));
						p_mns_control->remote_node[index].conn_handle = BLE_CONN_HANDLE_INVALID;
						error = 0;
						NRF_LOG_INFO("mns_control_delete_node:delete handle %d from index %d",
												 conn_handle,
						             index);
					  break;
				}
		}
		
		if(p_mns_control->total_connection > 0)
		{
				p_mns_control->total_connection--;
				NRF_LOG_INFO("mns_control_delete_node:total_connection = %d", p_mns_control->total_connection);
		}
		
		return error;		
}


/**@brief Function for synchronize the counter between each node
 *
 * @param[in] 
 */
uint32_t mns_control_synchronize_with_node(mns_control_t* p_mns_control)
{	
		ble_mnss_data_t* p_remote_data;
		ble_mnss_data_t* p_local_data = &(p_mns_control->local_data);
		uint16_t index;
		uint32_t current_sn;
		uint32_t current_cnt;
	
		current_sn = p_local_data->device_sn;
		current_cnt = p_local_data->cnt;
	
	  for(index=0;index<MNS_MAX_NODE_NUM;index++)
		{						
				p_remote_data = &(p_mns_control->remote_node[index].data);
			  if(p_mns_control->remote_node[index].update_flag == 1)
				{
						p_mns_control->remote_node[index].update_flag = 0;
						if(current_sn > p_remote_data->sn)  /*small SN is higher priority*/
						{
								current_cnt = p_remote_data->cnt;
								current_sn = p_remote_data->sn;
						}						
				}
		}
		
		if(p_local_data->sn > current_sn)
		{
				p_local_data->sn = current_sn;
				p_local_data->cnt = current_cnt;
		}
		return 0;
}



static uint16_t choose_one_node_to_disconnect(mns_control_t* p_mns_control, mns_node_t* p_node_x, mns_node_t* p_node_y)
{
		uint16_t conn_handle = BLE_CONN_HANDLE_INVALID;
	
	  if((p_node_x->ready_flag == 0) || (p_node_y->ready_flag == 0))
		{
				return conn_handle;
		}
	
		if(p_mns_control->local_data.device_sn > p_node_x->data.device_sn)
		{
				if(p_node_x->central_flag == 0)
				{
						if(p_node_x->conn_handle != BLE_CONN_HANDLE_INVALID)
						{
								conn_handle = p_node_x->conn_handle;
						}
				}
				else if(p_node_y->central_flag == 0)
				{
						if(p_node_y->conn_handle != BLE_CONN_HANDLE_INVALID)
						{
								conn_handle = p_node_y->conn_handle;
						}
				}
		}
		return conn_handle;
}

/**@brief Function for exchange the data between each node
 *
 * @param[in] 
 */
uint32_t mns_control_redundant_connection_handle(mns_control_t* p_mns_control)
{
		uint32_t err_code;
		uint32_t ret = 0;
	  uint16_t conn_handle = BLE_CONN_HANDLE_INVALID;
		mns_node_t* p_node_0 = &p_mns_control->remote_node[0];
		mns_node_t* p_node_1 = &p_mns_control->remote_node[1];
	  mns_node_t* p_node_2 = &p_mns_control->remote_node[2];

		/*compare the sn of each node and low priority node act the gatt client role */		
	  if(p_node_0->data.device_sn == p_node_1->data.device_sn)
		{
				conn_handle = choose_one_node_to_disconnect(p_mns_control, p_node_0, p_node_1);
		}
		else if(p_node_0->data.device_sn == p_node_2->data.device_sn)
		{
				conn_handle = choose_one_node_to_disconnect(p_mns_control, p_node_0, p_node_2);
		}
		else if(p_node_1->data.device_sn == p_node_2->data.device_sn)
		{
				conn_handle = choose_one_node_to_disconnect(p_mns_control, p_node_1, p_node_2);
		}
		
		
		if((conn_handle != BLE_CONN_HANDLE_INVALID) && (p_mns_control->disc_ongoing == false))
		{
				p_mns_control->disc_ongoing = true;
				err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			  ret = 1;
				NRF_LOG_INFO("disconnect the node conn_handle = %d, error:%d", conn_handle, err_code);			
		}
		return ret;
}


/**@brief Function for exchange the data between each node
 *
 * @param[in] 
 */
uint32_t mns_control_communicate_with_node(mns_control_t* p_mns_control)
{
		uint16_t index;
		ble_gatts_value_t gatt_value;
		ble_mnss_data_t* p_local_data = &(p_mns_control->local_data);

		/*exchange the data with each remote node*/		
	  for(index=0;index<MNS_MAX_NODE_NUM;index++)
		{
			
			  
				if(p_mns_control->remote_node[index].conn_handle != BLE_CONN_HANDLE_INVALID)
				{
						if(p_mns_control->remote_node[index].central_flag == 1)   
						{
								ble_mnss_write_data(p_mns_control->p_central_service, p_local_data);
								ble_mnss_read_data(p_mns_control->p_central_service);
						}
						else
						{

								gatt_value.len = sizeof(ble_mnss_data_t);
								gatt_value.p_value = (uint8_t*)p_local_data;
								gatt_value.offset = 0;

								sd_ble_gatts_value_set(p_mns_control->remote_node[index].conn_handle,
																			 p_mns_control->p_peripheral_service->data_read_handle.value_handle, 
																			 &gatt_value);							
						}
				}
							
		}
		return 0;
}


/**@brief Function for update the data of each node
 *
 * @param[in] 
 */
uint32_t mns_control_udpate_node(mns_control_t* p_mns_control, uint16_t conn_handle, ble_mnss_data_t* data)
{
		mns_node_t* p_node;
	
		p_node = mns_control_find_node(p_mns_control, conn_handle);
	  if(p_node != NULL)
		{
				p_node->update_flag = 1;
				p_node->ready_flag = 1;
				memcpy(&(p_node->data), data, sizeof(ble_mnss_data_t)); 
		}
		/*
		NRF_LOG_INFO("mns_control_udpate_node:");
	  NRF_LOG_INFO("SN = %X", data->sn);
		NRF_LOG_INFO("CNT = %d", data->cnt);
		NRF_LOG_INFO("PERIOD = %d", data->period);
		*/
		return 0;
}



/**@brief Function for check if node have already connect
 *
 * @param[in] 
 */
uint16_t mns_control_if_node_connected(mns_control_t* p_mns_control, const ble_gap_addr_t* p_addr)
{
		uint16_t index;
		int result;
		uint16_t ret = MNS_INVALID_INDEX;
	
	  for(index=0;index<MNS_MAX_NODE_NUM;index++)
		{
			  result = memcmp(&(p_mns_control->remote_node[index].peer_addr), p_addr, sizeof(ble_gap_addr_t));
				if(result == 0)
				{
						ret = index;
					  break;
				}
		}
		
		return ret;
}



/**@brief Function for synchronize the counter between each node
 *
 * @param[in] 
 */
void mns_control_display_nodes(mns_control_t* p_mns_control)
{
		uint16_t index;
		mns_node_t* p_node;

		NRF_LOG_INFO("total_connection = %d", p_mns_control->total_connection);
	  for(index=0;index<MNS_MAX_NODE_NUM;index++)
    {
				p_node = &p_mns_control->remote_node[index];
				NRF_LOG_INFO("node[%d]:", index);
			  NRF_LOG_INFO("conn_handle = %d", p_node->conn_handle);
				NRF_LOG_HEXDUMP_INFO(&p_node->peer_addr.addr, 6);
			  if(p_node->central_flag == 1)
				{
						NRF_LOG_INFO("role = central");
				}
				else
				{
						NRF_LOG_INFO("role = periphral");
				}
				NRF_LOG_INFO("sn = %X", p_node->data.sn);
				NRF_LOG_INFO("cnt = %X", p_node->data.cnt);
				NRF_LOG_INFO("period = %d", p_node->data.period);
		}	
}
/**
 * @}
 */
