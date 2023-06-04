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

#define NRF_LOG_MODULE_NAME mns_control
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/**@brief Function for initialize the globle variables m_mns_control
 *
 * @param[in] 
 */
mns_node_t* mns_control_init(mns_control_t* p_mns_control)
{
		mns_node_t* p_node= NULL;
		uint16_t index;
	  
		memset(p_mns_control, 0, sizeof(mns_control_t));
	
		for(index=0;index<MNS_MAX_NODE_NUM;index++)
		{
				p_mns_control->node[index].conn_handle = BLE_CONN_HANDLE_INVALID;
		}
		return 	p_node;	
}

/**@brief Function for find the node according the connect handle
 *
 * @param[in] 
 */
mns_node_t* mns_control_find_node(mns_control_t* p_mns, uint16_t conn_handle)
{
		mns_node_t* p_node= NULL;
		uint16_t index;
	  
		for(index=0;index<MNS_MAX_NODE_NUM;index++)
		{
				if(p_mns->node[index].conn_handle == conn_handle)
				{
						p_node = &(p_mns->node[index]);
				}
		}
		return 	p_node;	
}

/**@brief Function for find the node according the connect handle
 *
 * @param[in] 
 */
uint32_t mns_control_add_node(mns_control_t* p_mns, mns_node_t* p_node)
{
		uint16_t index;
		uint32_t error = 1;
	
	  for(index=0;index<MNS_MAX_NODE_NUM;index++)
		{
				if(p_mns->node[index].conn_handle == BLE_CONN_HANDLE_INVALID)
				{
						memcpy(&(p_mns->node[index]), p_node, sizeof(mns_node_t));
						error = 0;
						NRF_LOG_INFO("mns_control_add_node:add handle %d to index %d",
					               p_mns->node[index].conn_handle,
					               index);
						break;
				}
		}
		
		if(p_mns->total_connection < MNS_MAX_NODE_NUM)
		{
				p_mns->total_connection++;
				NRF_LOG_INFO("mns_control_add_node:total_connection = %d", p_mns->total_connection);
		}
		
		return error;
}

/**@brief Function for delete the node according the connect handle
 *
 * @param[in] 
 */
uint32_t mns_control_delete_node(mns_control_t* p_mns, uint16_t conn_handle)
{
		uint16_t index;
		uint32_t error = 1;
	
	  for(index=0;index<MNS_MAX_NODE_NUM;index++)
		{
				if(p_mns->node[index].conn_handle == conn_handle)
				{
						memset(&(p_mns->node[index]), 0, sizeof(mns_node_t));
						p_mns->node[index].conn_handle = BLE_CONN_HANDLE_INVALID;
						error = 0;
						NRF_LOG_INFO("mns_control_delete_node:delete handle %d from index %d",
												 conn_handle,
						             index);
					  break;
				}
		}
		
		if(p_mns->total_connection > 0)
		{
				p_mns->total_connection--;
				NRF_LOG_INFO("mns_control_delete_node:total_connection = %d", p_mns->total_connection);
		}
		
		return error;		
}

/**@brief Function for synchronize the counter between each node
 *
 * @param[in] 
 */
uint32_t mns_control_syncrhonize_node(mns_control_t* p_mns, uint32_t local_cnt)
{
}


/**
 * @}
 */
