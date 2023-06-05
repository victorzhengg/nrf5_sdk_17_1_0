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
/** @file   mns_control.h
 *
 * @defgroup multi node synchronize control
 * @{
 * @ingroup mns
 *
 * @brief multi node synchronize control
 *
 * @details This module enables the application to synchromize the timer in each node
 */

#ifndef MNS_CONTROL_H__
#define MNS_CONTROL_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "sdk_config.h"
#include "app_error.h"
#include "app_util.h"
#include "compiler_abstraction.h"
#include "nordic_common.h"

#include "ble_gap.h"

#include "ble_mnss.h"
#include "ble_mnss_c.h"

#ifdef __cplusplus
extern "C" {
#endif


#define MNS_MAX_NODE_NUM                  3
#define MNS_INVALID_INDEX                 0xFFFF
#define MNS_CONTROL_ERROR_THRESHOLD       5
#define FICR_DEVICE_ADDR                  ((uint32_t*)0x100000A4)
#define MNS_CONTROL_LED_PERIOD            100

/**@brief LED Button Client initialization structure. */
typedef struct
{
    uint16_t              conn_handle;
		ble_gap_addr_t        peer_addr; 
		ble_mnss_data_t       data;
		uint8_t               update_flag;
		uint8_t               central_flag;
} mns_node_t;

typedef struct
{
    uint16_t          total_connection;
		ble_mnss_t*       p_peripheral_service;
		ble_mnss_c_t*     p_central_service;
	  ble_mnss_data_t   local_data;
		mns_node_t        remote_node[MNS_MAX_NODE_NUM];
} mns_control_t;


/**@brief Function for find the node according the connect handle
 *
 * @param[in] 
 */
mns_node_t* mns_control_find_node(mns_control_t* p_mns_control, uint16_t conn_handle);


/**@brief Function to add a node to structure mns_control
 *
 * @param[in] 
 */
uint32_t mns_control_add_node(mns_control_t* p_mns_control, mns_node_t* p_node);


/**@brief Function for delete the node according the connect handle
 *
 * @param[in] 
 */
uint32_t mns_control_delete_node(mns_control_t* p_mns_control, uint16_t conn_handle);


/**@brief Function for synchronize the counter between each node
 *
 * @param[in] 
 */
uint32_t mns_control_synchronize_with_node(mns_control_t* p_mns_control);



/**@brief Function for exchange the data between each node
 *
 * @param[in] 
 */
uint32_t mns_control_communicate_with_node(mns_control_t* p_mns_control);



/**@brief Function for initialize the globle variables m_mns_control
 *
 * @param[in] 
 */
mns_node_t* mns_control_init(mns_control_t* p_mns_control,		
                             ble_mnss_t*    p_peripheral_service,
		                         ble_mnss_c_t*  p_central_service);


/**@brief Function for check if node have already connect
 *
 * @param[in] 
 * @retval index              If find return the index of node. Otherwise return the 
 */
uint16_t mns_control_if_node_connected(mns_control_t* p_mns_control, const ble_gap_addr_t* p_addr);


/**@brief Function for update the data of each node
 *
 * @param[in] 
 */
uint32_t mns_control_udpate_node(mns_control_t* p_mns_control, uint16_t conn_handle, ble_mnss_data_t* data);

#ifdef __cplusplus
}
#endif

#endif // MNS_CONTROL_H__

/** @} */
