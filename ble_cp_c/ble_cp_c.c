/**
 * Copyright (c) 2012 - 2019, Nordic Semiconductor ASA
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
#include "../ble_cp_c/ble_cp_c.h"

#include "sdk_common.h"

#include <string.h>
#include <assert.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_gattc.h"
#include "ble_date_time.h"
#include "ble_db_discovery.h"
#define NRF_LOG_MODULE_NAME ble_cp_c
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();

_Static_assert(sizeof(cycling_power_vector_flags_t) == 1, "cycling_power_vector_flags_t wrong size");
_Static_assert(sizeof(cycling_power_meas_flags_t) == 2  , "cycling_power_meas_flags_t wrong size");

#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */


/**@brief Function for intercepting errors of GATTC and the BLE GATT Queue.
 *
 * @param[in] nrf_error   Error code.
 * @param[in] p_ctx       Parameter from the event handler.
 * @param[in] conn_handle Connection handle.
 */
static void gatt_error_handler(uint32_t   nrf_error,
		void     * p_ctx,
		uint16_t   conn_handle)
{
	ble_cp_c_t * p_cp = (ble_cp_c_t *)p_ctx;

	NRF_LOG_DEBUG("A GATT Client error has occurred on conn_handle: 0X%X", conn_handle);

	if (p_cp->error_handler != NULL)
	{
		p_cp->error_handler(nrf_error);
	}
}


/**@brief Function for handling events from the Database Discovery module.
 *
 * @details This function handles an event from the Database Discovery module, and determines
 *          whether it relates to the discovery of CP at the peer. If it does, this function
 *          calls the application's event handler to indicate that the CP was
 *          discovered at the peer. The function also populates the event with service-related
 *          information before providing it to the application.
 *
 * @param[in] p_evt Pointer to the event received from the Database Discovery module.
 *
 */
void ble_cp_c_on_db_disc_evt(ble_cp_c_t * p_cp, ble_db_discovery_evt_t * p_evt)
{
	NRF_LOG_DEBUG("Database Discovery handler called with event 0x%x", p_evt->evt_type);

	ble_cp_c_evt_t evt;

	evt.conn_handle = p_evt->conn_handle;

	// Check if the CP was discovered.
	if (   (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE)
			&& (p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_CYCLING_POWER_SERVICE)
			&& (p_evt->params.discovered_db.srv_uuid.type == BLE_UUID_TYPE_BLE))
	{
		// Find the handles of the Current Time characteristic.
		for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
		{
			const ble_gatt_db_char_t * p_chars = &p_evt->params.discovered_db.charateristics[i];

			if (p_chars->characteristic.uuid.uuid == BLE_UUID_POWER_VECTOR_CHAR)
			{

				NRF_LOG_INFO("BLE_UUID_POWER_VECTOR_CHAR found 0x%04X",
						p_chars->characteristic.uuid.uuid);

				// Found Current Time characteristic. Store CCCD and value handle and break.
				evt.params.char_handles.vec_handle      = p_chars->characteristic.handle_value;
				evt.params.char_handles.vec_cccd_handle = p_chars->cccd_handle;
			}
			else if (p_chars->characteristic.uuid.uuid == BLE_UUID_POWER_MEASUREMENT_CHAR)
			{

				NRF_LOG_INFO("BLE_UUID_POWER_FEATURE_CHAR found 0x%04X",
						p_chars->characteristic.uuid.uuid);

				// Found Current Time characteristic. Store CCCD and value handle and break.
				evt.params.char_handles.power_handle      = p_chars->characteristic.handle_value;
				evt.params.char_handles.power_cccd_handle = p_chars->cccd_handle;

			} else {

				NRF_LOG_INFO("Unknown char 0x%04X",
						p_chars->characteristic.uuid.uuid);
			}
		}

		NRF_LOG_INFO("CP discovered at peer.");
		NRF_LOG_FLUSH();

		//If the instance has been assigned prior to db_discovery, assign the db_handles.
		if (p_cp->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
			if ((p_cp->char_handles.vec_cccd_handle == BLE_GATT_HANDLE_INVALID)&&
					(p_cp->char_handles.vec_handle == BLE_GATT_HANDLE_INVALID))
			{
				p_cp->char_handles = evt.params.char_handles;
			}
		}

		evt.evt_type    = BLE_CP_C_EVT_DISCOVERY_COMPLETE;
	}
	else if ((p_evt->evt_type == BLE_DB_DISCOVERY_SRV_NOT_FOUND) ||
			(p_evt->evt_type == BLE_DB_DISCOVERY_ERROR))
	{
		evt.evt_type    = BLE_CP_C_EVT_DISCOVERY_FAILED;
	}
	else
	{
		return;
	}

	p_cp->evt_handler(p_cp, &evt);
}


uint32_t ble_cp_c_init(ble_cp_c_t * p_cp, ble_cp_c_init_t const * p_cp_init)
{
	// Verify that the parameters needed to initialize this instance of CTS are not NULL.
	VERIFY_PARAM_NOT_NULL(p_cp);
	VERIFY_PARAM_NOT_NULL(p_cp_init);
	VERIFY_PARAM_NOT_NULL(p_cp_init->error_handler);
	VERIFY_PARAM_NOT_NULL(p_cp_init->evt_handler);
	VERIFY_PARAM_NOT_NULL(p_cp_init->p_gatt_queue);

	static ble_uuid_t cts_uuid;

	BLE_UUID_BLE_ASSIGN(cts_uuid, BLE_UUID_CYCLING_POWER_SERVICE);

	p_cp->evt_handler                  = p_cp_init->evt_handler;
	p_cp->error_handler                = p_cp_init->error_handler;
	p_cp->conn_handle                  = BLE_CONN_HANDLE_INVALID;
	p_cp->char_handles.vec_handle      = BLE_GATT_HANDLE_INVALID;
	p_cp->char_handles.vec_cccd_handle = BLE_GATT_HANDLE_INVALID;
	p_cp->char_handles.power_handle      = BLE_GATT_HANDLE_INVALID;
	p_cp->char_handles.power_cccd_handle = BLE_GATT_HANDLE_INVALID;
	p_cp->p_gatt_queue                 = p_cp_init->p_gatt_queue;

	return ble_db_discovery_evt_register(&cts_uuid);
}


/**@brief Function for decoding a read from the Current Time characteristic.
 *
 * @param[in] p_time  Current Time structure.
 * @param[in] p_data  Pointer to the buffer containing the Current Time.
 * @param[in] length  Length of the buffer containing the Current Time.
 *
 * @retval NRF_SUCCESS 			If the time struct is valid.
 * @retval NRF_ERROR_DATA_SIZE 	If the length does not match the expected size of the data.
 */
static uint32_t power_vector_decode(cycling_power_vector_char_t * p_pow_vec,
		ble_cp_c_evt_t *p_evt,
		uint8_t const       * p_data,
		uint32_t const        length)
{
	NRF_LOG_HEXDUMP_INFO(p_data, length);

	// indicate the evt
	p_evt->evt_type = BLE_CP_C_EVT_VECTOR_UPDATED;

	uint16_t i = p_pow_vec->array_size;
	uint16_t index = 0;

	p_pow_vec->uFlags = p_data[index++];

	if (p_pow_vec->flags.crank_rev_data) {

		// commit the result to the event
		memcpy(&p_evt->params.vector_evt, p_pow_vec, sizeof(cycling_power_vector_char_t));
		p_evt->evt_type = BLE_CP_C_EVT_VECTOR_RECV;

		// reset the array values
		p_pow_vec->array_size = i = 0;

		p_pow_vec->cumul_crank_rev = uint16_decode(&(p_data[index]));
		index += 2;

		p_pow_vec->last_crank_evt = uint16_decode(&(p_data[index]));
		index += 2;
	}

	if (p_pow_vec->flags.first_angle_data) {

		p_pow_vec->first_crank_angle = uint16_decode(&(p_data[index]));
		index += 2;
	}

	while (p_pow_vec->flags.inst_torque_array &&
			index + 1 < length &&
			i < sizeof(p_pow_vec->inst_torque_mag_array)/sizeof(p_pow_vec->inst_torque_mag_array[0])) {

		p_pow_vec->inst_torque_mag_array[i++] = (int16_t)uint16_decode(&(p_data[index]));
		index += 2;
	}

	while (p_pow_vec->flags.f_mag_array_array &&
			index + 1 < length &&
			i < sizeof(p_pow_vec->inst_force_mag_array)/sizeof(p_pow_vec->inst_force_mag_array[0])) {

		p_pow_vec->inst_force_mag_array[i++] = (int16_t)uint16_decode(&(p_data[index]));
		index += 2;
	}

	p_pow_vec->array_size = i;

	//lint -restore
	return NRF_SUCCESS;
}

static uint32_t power_measure_decode(cycling_power_meas_char_t * p_pow_meas,
		ble_cp_c_evt_t *p_evt,
		uint8_t const       * p_data,
		uint32_t const        length)
{
	NRF_LOG_HEXDUMP_INFO(p_data, length);

	uint16_t index = 0;

	p_pow_meas->uFlags = uint16_decode(&(p_data[index]));
	index += 2;

	p_pow_meas->inst_power = (int16_t)uint16_decode(&(p_data[index]));
	index += 2;

	if (p_pow_meas->flags.pedal_power_balance) {

		p_pow_meas->pedal_power_balance = p_data[index];
		index += 1;
	}

	if (p_pow_meas->flags.acc_torque) {

		p_pow_meas->acc_torque = uint16_decode(&(p_data[index]));
		index += 2;
	}

	if (p_pow_meas->flags.wheel_rev) {

		p_pow_meas->cumul_wheel_rev = uint32_decode(&(p_data[index]));
		index += 4;

		p_pow_meas->last_wheel_evt = uint16_decode(&(p_data[index]));
		index += 2;
	}

	if (p_pow_meas->flags.crank_rev) {

		p_pow_meas->cumul_crank_rev = uint16_decode(&(p_data[index]));
		index += 2;

		p_pow_meas->last_crank_evt = uint16_decode(&(p_data[index]));
		index += 2;
	}

	p_evt->evt_type = BLE_CP_C_EVT_POWER_RECV;

	//lint -restore
	return NRF_SUCCESS;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in] p_cp      CP client structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_disconnect(ble_cp_c_t * p_cp, ble_evt_t const * p_ble_evt)
{
	if (p_cp->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
	{
		p_cp->conn_handle = BLE_CONN_HANDLE_INVALID;
	}

	if (ble_cp_c_is_discovered(p_cp))
	{
		// There was a valid instance of CTS on the peer. Send an event to the
		// application, so that it can do any clean up related to this module.
		ble_cp_c_evt_t evt;

		evt.evt_type = BLE_CP_C_EVT_DISCONN_COMPLETE;

		p_cp->evt_handler(p_cp, &evt);
		p_cp->char_handles.vec_handle      = BLE_GATT_HANDLE_INVALID;
		p_cp->char_handles.vec_cccd_handle = BLE_GATT_HANDLE_INVALID;

		p_cp->char_handles.power_handle      = BLE_GATT_HANDLE_INVALID;
		p_cp->char_handles.power_cccd_handle = BLE_GATT_HANDLE_INVALID;
	}
}

/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details   This function uses the Handle Value Notification received from the SoftDevice
 *            and checks whether it is a notification of the heart rate measurement from the peer. If
 *            it is, this function decodes the heart rate measurement and sends it to the
 *            application.
 *
 * @param[in] p_ble_cp_c Pointer to the Heart Rate Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(ble_cp_c_t * p_ble_cp_c, const ble_evt_t * p_ble_evt)
{
	// Check if the event is on the link for this instance.
	if (p_ble_cp_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
	{
		NRF_LOG_DEBUG("Received HVX on link 0x%x, not associated to this instance. Ignore.",
				p_ble_evt->evt.gattc_evt.conn_handle);
		return;
	}

	NRF_LOG_DEBUG("Received HVX on link 0x%x, vector_handle 0x%x",
			p_ble_evt->evt.gattc_evt.params.hvx.handle,
			p_ble_cp_c->char_handles.vec_handle);

	// Check if this is a CP notification.
	if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_cp_c->char_handles.vec_handle)
	{
		ble_cp_c_evt_t evt;

		if (NRF_SUCCESS == power_vector_decode(&p_ble_cp_c->power_vector, &evt,
				p_ble_evt->evt.gattc_evt.params.hvx.data,
				p_ble_evt->evt.gattc_evt.params.hvx.len)) {

			p_ble_cp_c->evt_handler(p_ble_cp_c, &evt);
		} else {

			NRF_LOG_WARNING("power_vector_decode error");
		}

	} else if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_cp_c->char_handles.power_handle) {

		ble_cp_c_evt_t evt;

		if (NRF_SUCCESS == power_measure_decode(&evt.params.power_meas, &evt,
				p_ble_evt->evt.gattc_evt.params.hvx.data,
				p_ble_evt->evt.gattc_evt.params.hvx.len)) {

			p_ble_cp_c->evt_handler(p_ble_cp_c, &evt);
		} else {

			NRF_LOG_WARNING("power_vector_decode error");
		}

	}
}

void ble_cp_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
	ble_cp_c_t * p_cp = (ble_cp_c_t *)p_context;
	NRF_LOG_DEBUG("BLE event handler called with event 0x%x", p_ble_evt->header.evt_id);

	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GATTC_EVT_HVX:
		on_hvx(p_cp, p_ble_evt);
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		on_disconnect(p_cp, p_ble_evt);
		break;

	default:
		// No implementation needed.
		break;
	}
}

uint32_t ble_cp_c_handles_assign(ble_cp_c_t               * p_cp,
		const uint16_t              conn_handle,
		const ble_cp_c_handles_t * p_peer_handles)
{
	VERIFY_PARAM_NOT_NULL(p_cp);

	p_cp->conn_handle = conn_handle;
	if (p_peer_handles != NULL)
	{
		p_cp->char_handles.vec_cccd_handle = p_peer_handles->vec_cccd_handle;
		p_cp->char_handles.vec_handle      = p_peer_handles->vec_handle;

		p_cp->char_handles.power_cccd_handle = p_peer_handles->power_cccd_handle;
		p_cp->char_handles.power_handle      = p_peer_handles->power_handle;
	}

	return nrf_ble_gq_conn_handle_register(p_cp->p_gatt_queue, conn_handle);
}



/**@brief Function for creating a message for writing to the CCCD.
 */
static uint32_t cccd_configure(ble_cp_c_t * p_ble_cp_c, uint16_t cccd_handle, bool enable)
{
	NRF_LOG_DEBUG("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d",
			p_ble_cp_c->char_handles.vec_cccd_handle,
			p_ble_cp_c->conn_handle);

	uint8_t          cccd[WRITE_MESSAGE_LENGTH];
	uint16_t         cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;
	nrf_ble_gq_req_t rscs_c_req;

	cccd[0] = LSB_16(cccd_val);
	cccd[1] = MSB_16(cccd_val);

	memset(&rscs_c_req, 0, sizeof(rscs_c_req));
	rscs_c_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
	rscs_c_req.error_handler.cb            = gatt_error_handler;
	rscs_c_req.error_handler.p_ctx         = p_ble_cp_c;
	rscs_c_req.params.gattc_write.handle   = cccd_handle;
	rscs_c_req.params.gattc_write.len      = WRITE_MESSAGE_LENGTH;
	rscs_c_req.params.gattc_write.p_value  = cccd;
	rscs_c_req.params.gattc_write.offset   = 0;
	rscs_c_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;
	rscs_c_req.params.gattc_write.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;

	return nrf_ble_gq_item_add(p_ble_cp_c->p_gatt_queue, &rscs_c_req, p_ble_cp_c->conn_handle);
}


uint32_t ble_cp_c_pv_notif_enable(ble_cp_c_t * p_ble_cp_c)
{
	VERIFY_PARAM_NOT_NULL(p_ble_cp_c);

	if (p_ble_cp_c->conn_handle == BLE_CONN_HANDLE_INVALID)
	{
		return NRF_ERROR_INVALID_STATE;
	}

	uint32_t ret=0;
	if (BLE_CONN_HANDLE_INVALID != p_ble_cp_c->char_handles.power_cccd_handle)
		ret += cccd_configure(p_ble_cp_c,  p_ble_cp_c->char_handles.power_cccd_handle, true);
	if (BLE_CONN_HANDLE_INVALID != p_ble_cp_c->char_handles.vec_cccd_handle)
		ret += cccd_configure(p_ble_cp_c,  p_ble_cp_c->char_handles.vec_cccd_handle, true);

	return ret;
}

