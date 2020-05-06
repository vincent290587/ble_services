
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_LNS_C)
#include "app_util.h"
#include "app_util_bds.h"
#include "ble_lns_c.h"
#include "ble_db_discovery.h"
#include "ble_types.h"
#include "ble_srv_common.h"
#include "ble_gattc.h"

#include "segger_wrapper.h"

#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */


static __INLINE uint8_t bds_int32_decode(const uint8_t * p_encoded_data,
		int32_t       * p_decoded_val)
{
	uint32_t tmp = 0;
	uint8_t retval = bds_uint32_decode(4, p_encoded_data, &tmp);
	*p_decoded_val = (int32_t)tmp;
	return retval;
}


/**@brief Function for intercepting the errors of GATTC and the BLE GATT Queue.
 *
 * @param[in] nrf_error   Error code.
 * @param[in] p_ctx       Parameter from the event handler.
 * @param[in] conn_handle Connection handle.
 */
static void gatt_error_handler(uint32_t   nrf_error,
		void     * p_ctx,
		uint16_t   conn_handle)
{
	ble_lns_c_t * p_ble_lns_c = (ble_lns_c_t *)p_ctx;

	NRF_LOG_DEBUG("A GATT Client error has occurred on conn_handle: 0X%X", conn_handle);

	if (p_ble_lns_c->error_handler != NULL)
	{
		p_ble_lns_c->error_handler(nrf_error);
	}
}


/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details   This function will uses the Handle Value Notification received from the SoftDevice
 *            and checks if it is a notification of the heart rate measurement from the peer. If
 *            it is, this function will decode the heart rate measurement and send it to the
 *            application.
 *
 * @param[in] p_ble_lns_c Pointer to the Heart Rate Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(ble_lns_c_t * p_ble_lns_c, const ble_evt_t * p_ble_evt)
{
	// Check if the event is on the link for this instance
	if (p_ble_lns_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
	{
		NRF_LOG_DEBUG("received HVX on link 0x%x, not associated to this instance, ignore\r\n",
				p_ble_evt->evt.gattc_evt.conn_handle);
		return;
	}
	NRF_LOG_DEBUG("received HVX on handle 0x%x, lns_handle 0x%x\r\n",
			p_ble_evt->evt.gattc_evt.params.hvx.handle,
			p_ble_lns_c->peer_lns_db.lns_handle);

	// Check if this is a notification.
	if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_lns_c->peer_lns_db.lns_handle)
	{
		ble_lns_c_evt_t ble_lns_c_evt;
		uint32_t        index = 0;
		uint16_t        flags;

		ble_lns_c_evt.evt_type                    = BLE_LNS_C_EVT_LNS_NOTIFICATION;
		ble_lns_c_evt.conn_handle                 = p_ble_lns_c->conn_handle;

		flags = uint16_decode(&(p_ble_evt->evt.gattc_evt.params.hvx.data[index]));  //lint !e415 suppress Lint Warning 415: Likely access out of bond
		index += sizeof(uint16_t);

		ble_lns_c_evt.params.lns.flags = flags;

		if (flags & INST_SPEED_PRESENT)
		{
			ble_lns_c_evt.params.lns.inst_speed = uint16_decode(&(p_ble_evt->evt.gattc_evt.params.hvx.data[index]));  //lint !e415 suppress Lint Warning 415: Likely access out of bond
			index += sizeof(uint16_t);
		} else {
			ble_lns_c_evt.params.lns.inst_speed = BLE_LNS_INVALID_DATA;
		}
		if (flags & TOT_DIST_PRESENT)
		{
			index += bds_uint24_decode(3, &(p_ble_evt->evt.gattc_evt.params.hvx.data[index]),
					&ble_lns_c_evt.params.lns.tot_distance);
		} else {
			ble_lns_c_evt.params.lns.tot_distance = BLE_LNS_INVALID_DATA;
		}
		if (flags & LOC_PRESENT)
		{
			index += bds_int32_decode(&(p_ble_evt->evt.gattc_evt.params.hvx.data[index]), &ble_lns_c_evt.params.lns.lat);  //lint !e415 suppress Lint Warning 415: Likely access out of bond

			index += bds_int32_decode(&(p_ble_evt->evt.gattc_evt.params.hvx.data[index]), &ble_lns_c_evt.params.lns.lon);  //lint !e415 suppress Lint Warning 415: Likely access out of bond
		}
		if (1) {
			if (flags & ELE_PRESENT)
			{
				int32_t tmp_ele = p_ble_evt->evt.gattc_evt.params.hvx.data[index++] << 8;
				tmp_ele |= p_ble_evt->evt.gattc_evt.params.hvx.data[index++] << 16;
				tmp_ele |= p_ble_evt->evt.gattc_evt.params.hvx.data[index++] << 24;

				ble_lns_c_evt.params.lns.ele = tmp_ele / 0xFF;

			} else {
				ble_lns_c_evt.params.lns.ele = BLE_LNS_INVALID_DATA;
			}
			if (flags & HEADING_PRESENT)
			{
				ble_lns_c_evt.params.lns.heading = uint16_decode(&(p_ble_evt->evt.gattc_evt.params.hvx.data[index]));  //lint !e415 suppress Lint Warning 415: Likely access out of bond
				index += sizeof(uint16_t);
			}
			if (flags & ROLL_TIME_PRESENT)
			{
				ble_lns_c_evt.params.lns.roll_time = p_ble_evt->evt.gattc_evt.params.hvx.data[index];  //lint !e415 suppress Lint Warning 415: Likely access out of bond
				index += sizeof(uint8_t);
			}
			if (flags & UTC_TIME_PRESENT)
			{
				index += bds_ble_date_time_decode(0, &(p_ble_evt->evt.gattc_evt.params.hvx.data[index]),
						&ble_lns_c_evt.params.lns.utc_time);
			}
		}

		p_ble_lns_c->evt_handler(p_ble_lns_c, &ble_lns_c_evt);
	}
}


/**@brief     Function for handling Disconnected event received from the SoftDevice.
 *
 * @details   This function check if the disconnect event is happening on the link
 *            associated with the current instance of the module, if so it will set its
 *            conn_handle to invalid.
 *
 * @param[in] p_ble_lns_c Pointer to the Heart Rate Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_disconnected(ble_lns_c_t * p_ble_lns_c, const ble_evt_t * p_ble_evt)
{
	if (p_ble_lns_c->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
	{
		p_ble_lns_c->conn_handle                 = BLE_CONN_HANDLE_INVALID;
		p_ble_lns_c->peer_lns_db.lns_cccd_handle = BLE_GATT_HANDLE_INVALID;
		p_ble_lns_c->peer_lns_db.lns_handle      = BLE_GATT_HANDLE_INVALID;
	}
}


void ble_lns_c_on_db_disc_evt(ble_lns_c_t * p_ble_lns_c, const ble_db_discovery_evt_t * p_evt)
{
	// Check if the Rate Service was discovered.
	if ((p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE) &&
			p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_LOCATION_AND_NAVIGATION_SERVICE &&
			p_evt->params.discovered_db.srv_uuid.type == BLE_UUID_TYPE_BLE)
	{
		// Find the CCCD Handle of the Heart Rate Measurement characteristic.
		uint32_t i;
		ble_lns_c_evt_t evt;

		LOG_INFO("Database Discovery handler called with event 0x%x\r\n", p_evt->evt_type);

		evt.conn_handle = p_evt->conn_handle;
		evt.evt_type = BLE_LNS_C_EVT_DISCOVERY_COMPLETE;

		LOG_INFO("Database Discovery complete\r\n");

		for (i = 0; i < p_evt->params.discovered_db.char_count; i++)
		{

			if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
					BLE_UUID_LN_LOCATION_AND_SPEED_CHAR)
			{
				// Found LNS characteristic. Store CCCD handle and break.
				evt.params.peer_db.lns_cccd_handle =
						p_evt->params.discovered_db.charateristics[i].cccd_handle;
				evt.params.peer_db.lns_handle =
						p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
			}

			LOG_INFO("Discovered LNS CHAR: 0x%X\r\n", p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid);
			//NRF_LOG_FLUSH();
		}

		LOG_INFO("LNS Service discovered at peer.\r\n");
		//If the instance has been assigned prior to db_discovery, assign the db_handles
		if (p_ble_lns_c->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
			if ((p_ble_lns_c->peer_lns_db.lns_cccd_handle == BLE_GATT_HANDLE_INVALID)&&
					(p_ble_lns_c->peer_lns_db.lns_handle == BLE_GATT_HANDLE_INVALID))
			{
				p_ble_lns_c->peer_lns_db = evt.params.peer_db;
			}
		}


		p_ble_lns_c->evt_handler(p_ble_lns_c, &evt);
	}
}


uint32_t ble_lns_c_init(ble_lns_c_t * p_ble_lns_c, ble_lns_c_init_t * p_ble_lns_c_init)
{
	VERIFY_PARAM_NOT_NULL(p_ble_lns_c);
	VERIFY_PARAM_NOT_NULL(p_ble_lns_c_init);
	VERIFY_PARAM_NOT_NULL(p_ble_lns_c_init->p_gatt_queue);

	ble_uuid_t lns_uuid;

	lns_uuid.type = BLE_UUID_TYPE_BLE;
	lns_uuid.uuid = BLE_UUID_LOCATION_AND_NAVIGATION_SERVICE;

	p_ble_lns_c->uuid_type = BLE_UUID_TYPE_BLE;
	p_ble_lns_c->evt_handler                 = p_ble_lns_c_init->evt_handler;
	p_ble_lns_c->error_handler               = p_ble_lns_c_init->error_handler;
	p_ble_lns_c->p_gatt_queue                = p_ble_lns_c_init->p_gatt_queue;
	p_ble_lns_c->conn_handle                 = BLE_CONN_HANDLE_INVALID;
	p_ble_lns_c->peer_lns_db.lns_cccd_handle = BLE_GATT_HANDLE_INVALID;
	p_ble_lns_c->peer_lns_db.lns_handle      = BLE_GATT_HANDLE_INVALID;

	return ble_db_discovery_evt_register(&lns_uuid);
}


void ble_lns_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
	ble_lns_c_t * p_ble_lns_c = (ble_lns_c_t *)p_context;

	if ((p_ble_lns_c == NULL) || (p_ble_evt == NULL))
	{
		return;
	}

	if ( (p_ble_lns_c->conn_handle == BLE_CONN_HANDLE_INVALID)
			||(p_ble_lns_c->conn_handle != p_ble_evt->evt.gap_evt.conn_handle)
	)
	{
		return;
	}

	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GATTC_EVT_HVX:
		on_hvx(p_ble_lns_c, p_ble_evt);
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		on_disconnected(p_ble_lns_c, p_ble_evt);
		break;

	default:
		break;
	}
}


/**@brief Function for creating a message for writing to the CCCD.
 */
static uint32_t cccd_configure(ble_lns_c_t * p_ble_lns_c, bool notification_enable)
{
	nrf_ble_gq_req_t cccd_req;
	uint8_t          cccd[BLE_CCCD_VALUE_LEN];
	uint16_t         cccd_val = notification_enable ? BLE_GATT_HVX_NOTIFICATION : 0;

	memset(&cccd_req, 0, sizeof(nrf_ble_gq_req_t));

	cccd[0] = LSB_16(cccd_val);
	cccd[1] = MSB_16(cccd_val);

	cccd_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
	cccd_req.error_handler.cb            = gatt_error_handler;
	cccd_req.error_handler.p_ctx         = p_ble_lns_c;
	cccd_req.params.gattc_write.handle   = p_ble_lns_c->peer_lns_db.lns_cccd_handle;
	cccd_req.params.gattc_write.len      = BLE_CCCD_VALUE_LEN;
	cccd_req.params.gattc_write.offset   = 0;
	cccd_req.params.gattc_write.p_value  = cccd;
	cccd_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;
	cccd_req.params.gattc_write.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;

	return nrf_ble_gq_item_add(p_ble_lns_c->p_gatt_queue, &cccd_req, p_ble_lns_c->conn_handle);
}

uint32_t ble_lns_c_pos_notif_enable(ble_lns_c_t * p_ble_lns_c)
{
	VERIFY_PARAM_NOT_NULL(p_ble_lns_c);

	if ( (p_ble_lns_c->conn_handle == BLE_CONN_HANDLE_INVALID)
			||(p_ble_lns_c->peer_lns_db.lns_cccd_handle == BLE_GATT_HANDLE_INVALID)
	)
	{
		return NRF_ERROR_INVALID_STATE;
	}
	return cccd_configure(p_ble_lns_c, true);
}


uint32_t ble_lns_c_handles_assign(ble_lns_c_t * p_ble_lns_c,
		uint16_t conn_handle,
		const lns_db_t * p_peer_lns_handles)
{
	VERIFY_PARAM_NOT_NULL(p_ble_lns_c);

	p_ble_lns_c->conn_handle = conn_handle;
	if (p_peer_lns_handles != NULL)
	{
		p_ble_lns_c->peer_lns_db.lns_cccd_handle = p_peer_lns_handles->lns_cccd_handle;
		p_ble_lns_c->peer_lns_db.lns_handle = p_peer_lns_handles->lns_handle;
	}

	return nrf_ble_gq_conn_handle_register(p_ble_lns_c->p_gatt_queue, conn_handle);
}
/** @}
 *  @endcond
 */
#endif // NRF_MODULE_ENABLED(BLE_LNS_C)
