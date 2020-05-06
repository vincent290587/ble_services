
#include "sdk_common.h"

#if NRF_MODULE_ENABLED(BLE_KOMOOT_C)
#include "app_util.h"
#include "app_util_bds.h"
#include "ble_komoot_c.h"
#include "ble_db_discovery.h"
#include "ble_types.h"
#include "ble_srv_common.h"
#include "ble_gattc.h"

#define NRF_LOG_MODULE_NAME ble_komoot_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */


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
	ble_komoot_c_t * p_ble_komoot_c = (ble_komoot_c_t *)p_ctx;

	NRF_LOG_DEBUG("A GATT Client error has occurred on conn_handle: 0X%X", conn_handle);

	if (p_ble_komoot_c->error_handler != NULL)
	{
		p_ble_komoot_c->error_handler(nrf_error);
	}
}

/**@brief     Function for handling write response events.
 *
 * @param[in] p_ble_komoot_c Pointer to the Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_read_rsp(ble_komoot_c_t * p_ble_komoot_c, const ble_evt_t * p_ble_evt)
{
	const ble_gattc_evt_read_rsp_t * p_response;
	uint32_t        err_code = NRF_SUCCESS;

	// Check if the event is on the same connection as this cts instance
	if (p_ble_komoot_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
	{
		return;
	}

	p_response = &p_ble_evt->evt.gattc_evt.params.read_rsp;

	if (p_ble_evt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_SUCCESS)
	{
		NRF_LOG_INFO("on_read_rsp len: %u", p_response->len);
		NRF_LOG_HEXDUMP_DEBUG(p_response->data, p_response->len);

		if (err_code == NRF_SUCCESS)
		{
			uint32_t        index = 0;
			ble_komoot_c_evt_t ble_komoot_c_evt;

			// The data length was invalid, decoding was not completed.
			ble_komoot_c_evt.evt_type = BLE_KOMOOT_C_EVT_KOMOOT_NAVIGATION;

			ble_komoot_c_evt.params.komoot.identifier = uint32_decode(&(p_response->data[index]));
			index += sizeof(uint32_t);

			ble_komoot_c_evt.params.komoot.direction  = p_response->data[index++];

			ble_komoot_c_evt.params.komoot.distance   = uint32_decode(&(p_response->data[index]));
			index += sizeof(uint32_t);

			p_ble_komoot_c->evt_handler(p_ble_komoot_c, &ble_komoot_c_evt);
		}
	}
}


/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details   This function will uses the Handle Value Notification received from the SoftDevice
 *            and checks if it is a notification of the heart rate measurement from the peer. If
 *            it is, this function will decode the heart rate measurement and send it to the
 *            application.
 *
 * @param[in] p_ble_komoot_c Pointer to the Heart Rate Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(ble_komoot_c_t * p_ble_komoot_c, const ble_evt_t * p_ble_evt)
{
	// Check if the event is on the link for this instance
	if (p_ble_komoot_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
	{
		NRF_LOG_DEBUG("received HVX on link 0x%x, not associated to this instance, ignore\r\n",
				p_ble_evt->evt.gattc_evt.conn_handle);
		return;
	}
	NRF_LOG_DEBUG("received HVX on handle 0x%x, komoot_handle 0x%x\r\n",
			p_ble_evt->evt.gattc_evt.params.hvx.handle,
			p_ble_komoot_c->peer_komoot_db.komoot_handle);

	// Check if this is a notification.
	if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_komoot_c->peer_komoot_db.komoot_handle)
	{
		ble_komoot_c_evt_t ble_komoot_c_evt;

		ble_komoot_c_evt.evt_type                    = BLE_KOMOOT_C_EVT_KOMOOT_NOTIFICATION;
		ble_komoot_c_evt.conn_handle                 = p_ble_komoot_c->conn_handle;

		NRF_LOG_DEBUG("Read HVX: ");
		NRF_LOG_HEXDUMP_DEBUG(p_ble_evt->evt.gattc_evt.params.hvx.data, p_ble_evt->evt.gattc_evt.params.hvx.len);

		if (p_ble_evt->evt.gattc_evt.params.hvx.len == 4) {
			p_ble_komoot_c->evt_handler(p_ble_komoot_c, &ble_komoot_c_evt);
		}

	}
}


/**@brief     Function for handling Disconnected event received from the SoftDevice.
 *
 * @details   This function check if the disconnect event is happening on the link
 *            associated with the current instance of the module, if so it will set its
 *            conn_handle to invalid.
 *
 * @param[in] p_ble_komoot_c Pointer to the Heart Rate Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_disconnected(ble_komoot_c_t * p_ble_komoot_c, const ble_evt_t * p_ble_evt)
{
	if (p_ble_komoot_c->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
	{
		p_ble_komoot_c->conn_handle                 = BLE_CONN_HANDLE_INVALID;
		p_ble_komoot_c->peer_komoot_db.komoot_cccd_handle = BLE_GATT_HANDLE_INVALID;
		p_ble_komoot_c->peer_komoot_db.komoot_handle      = BLE_GATT_HANDLE_INVALID;
	}
}


void ble_komoot_c_on_db_disc_evt(ble_komoot_c_t * p_ble_komoot_c, const ble_db_discovery_evt_t * p_evt)
{
	// Check if the Rate Service was discovered.
	if ((p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE) &&
			p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_KOMOOT_SERVICE &&
			p_evt->params.discovered_db.srv_uuid.type == p_ble_komoot_c->uuid_type)
	{
		// Find the CCCD Handle of the characteristic.
		uint32_t i;
		ble_komoot_c_evt_t evt;

		NRF_LOG_DEBUG("Database Discovery handler called with event 0x%x\r\n", p_evt->evt_type);

		evt.conn_handle = p_evt->conn_handle;
		evt.evt_type    = BLE_KOMOOT_C_EVT_DISCOVERY_FAILED;

		evt.evt_type    = BLE_KOMOOT_C_EVT_DISCOVERY_COMPLETE;

		NRF_LOG_INFO("Database Discovery complete: %u CHAR found\r\n",
				p_evt->params.discovered_db.char_count);

		for (i = 0; i < p_evt->params.discovered_db.char_count; i++)
		{

			NRF_LOG_INFO("Discovered CHAR: 0x%04X\r\n", p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid);

			if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
					BLE_UUID_KOMOOT_CHAR) {

				// Found KOMOOT characteristic. Store CCCD handle and break.
				evt.params.peer_db.komoot_cccd_handle =
						p_evt->params.discovered_db.charateristics[i].cccd_handle;
				evt.params.peer_db.komoot_handle =
						p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;

				NRF_LOG_INFO("Storing CCCD handle.\r\n");
			}

		}

		NRF_LOG_INFO("KOMOOT Service discovered at peer.\r\n");
		//If the instance has been assigned prior to db_discovery, assign the db_handles
		if (p_ble_komoot_c->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
			if ((p_ble_komoot_c->peer_komoot_db.komoot_cccd_handle == BLE_GATT_HANDLE_INVALID)&&
					(p_ble_komoot_c->peer_komoot_db.komoot_handle == BLE_GATT_HANDLE_INVALID))
			{
				p_ble_komoot_c->peer_komoot_db = evt.params.peer_db;
			}
		}

		p_ble_komoot_c->evt_handler(p_ble_komoot_c, &evt);
	}
}


uint32_t ble_komoot_c_init(ble_komoot_c_t * p_ble_komoot_c, ble_komoot_c_init_t * p_ble_komoot_c_init)
{
	VERIFY_PARAM_NOT_NULL(p_ble_komoot_c);
	VERIFY_PARAM_NOT_NULL(p_ble_komoot_c_init);
	VERIFY_PARAM_NOT_NULL(p_ble_komoot_c_init->p_gatt_queue);

	ble_uuid_t komoot_uuid;
	ble_uuid128_t base_uuid_service = {BLE_BASE_UUID_KOMOOT_SERVICE};

	// Assign UUID types.
	ret_code_t err_code;
	err_code = sd_ble_uuid_vs_add(&base_uuid_service, &p_ble_komoot_c->uuid_type);
	VERIFY_SUCCESS(err_code);

	komoot_uuid.uuid = BLE_UUID_KOMOOT_SERVICE;
	komoot_uuid.type = p_ble_komoot_c->uuid_type;

	p_ble_komoot_c->conn_handle                 = BLE_CONN_HANDLE_INVALID;
	p_ble_komoot_c->evt_handler                 = p_ble_komoot_c_init->evt_handler;
	p_ble_komoot_c->error_handler               = p_ble_komoot_c_init->error_handler;
	p_ble_komoot_c->p_gatt_queue                = p_ble_komoot_c_init->p_gatt_queue;
	p_ble_komoot_c->peer_komoot_db.komoot_cccd_handle = BLE_GATT_HANDLE_INVALID;
	p_ble_komoot_c->peer_komoot_db.komoot_handle      = BLE_GATT_HANDLE_INVALID;

	return ble_db_discovery_evt_register(&komoot_uuid);
}


void ble_komoot_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
	ble_komoot_c_t * p_ble_komoot_c = (ble_komoot_c_t *)p_context;

	if ((p_ble_komoot_c == NULL) || (p_ble_evt == NULL))
	{
		return;
	}

	if ( (p_ble_komoot_c->conn_handle == BLE_CONN_HANDLE_INVALID)
			||(p_ble_komoot_c->conn_handle != p_ble_evt->evt.gap_evt.conn_handle)
	)
	{
		return;
	}

	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GATTC_EVT_HVX:
		on_hvx(p_ble_komoot_c, p_ble_evt);
		break;

	case BLE_GATTC_EVT_READ_RSP:
		on_read_rsp(p_ble_komoot_c, p_ble_evt);
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		on_disconnected(p_ble_komoot_c, p_ble_evt);
		break;

	default:
		break;
	}
}

/**@brief Function for creating a message for writing to the CCCD.
 */
static uint32_t cccd_configure(ble_komoot_c_t * p_ble_komoot_c, bool notification_enable)
{
	NRF_LOG_DEBUG("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d",
			p_ble_komoot_c->peer_komoot_db.komoot_cccd_handle,
			p_ble_komoot_c->conn_handle);

	nrf_ble_gq_req_t cccd_req;
	uint8_t          cccd[BLE_CCCD_VALUE_LEN];
	uint16_t         cccd_val = notification_enable ? BLE_GATT_HVX_NOTIFICATION : 0;

	memset(&cccd_req, 0, sizeof(nrf_ble_gq_req_t));

	cccd[0] = LSB_16(cccd_val);
	cccd[1] = MSB_16(cccd_val);

	cccd_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
	cccd_req.error_handler.cb            = gatt_error_handler;
	cccd_req.error_handler.p_ctx         = p_ble_komoot_c;
	cccd_req.params.gattc_write.handle   = p_ble_komoot_c->peer_komoot_db.komoot_cccd_handle;
	cccd_req.params.gattc_write.len      = BLE_CCCD_VALUE_LEN;
	cccd_req.params.gattc_write.offset   = 0;
	cccd_req.params.gattc_write.p_value  = cccd;
	cccd_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;
	//	 cccd_req.params.gattc_write.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;

	return nrf_ble_gq_item_add(p_ble_komoot_c->p_gatt_queue, &cccd_req, p_ble_komoot_c->conn_handle);
}

uint32_t ble_komoot_c_nav_read(ble_komoot_c_t * p_ble_komoot_c)
{
	VERIFY_PARAM_NOT_NULL(p_ble_komoot_c);

	nrf_ble_gq_req_t write_req;

	memset(&write_req, 0, sizeof(nrf_ble_gq_req_t));

	if (p_ble_komoot_c->conn_handle == BLE_CONN_HANDLE_INVALID)
	{
		return NRF_ERROR_INVALID_STATE;
	}

	write_req.type                        = NRF_BLE_GQ_REQ_GATTC_READ;
	write_req.error_handler.cb            = gatt_error_handler;
	write_req.error_handler.p_ctx         = p_ble_komoot_c;
	write_req.params.gattc_read.handle    = p_ble_komoot_c->peer_komoot_db.komoot_handle;
	write_req.params.gattc_read.offset    = 0;

	return nrf_ble_gq_item_add(p_ble_komoot_c->p_gatt_queue, &write_req, p_ble_komoot_c->conn_handle);
}

uint32_t ble_komoot_c_pos_notif_enable(ble_komoot_c_t * p_ble_komoot_c)
{
	VERIFY_PARAM_NOT_NULL(p_ble_komoot_c);

	if ( (p_ble_komoot_c->conn_handle == BLE_CONN_HANDLE_INVALID)
			||(p_ble_komoot_c->peer_komoot_db.komoot_cccd_handle == BLE_GATT_HANDLE_INVALID)
	)
	{
		return NRF_ERROR_INVALID_STATE;
	}
	return cccd_configure(p_ble_komoot_c, true);
}

uint32_t ble_komoot_c_handles_assign(ble_komoot_c_t * p_ble_komoot_c,
		uint16_t conn_handle,
		const komoot_db_t * p_peer_komoot_handles)
{
	VERIFY_PARAM_NOT_NULL(p_ble_komoot_c);

	p_ble_komoot_c->conn_handle = conn_handle;
	if (p_peer_komoot_handles != NULL)
	{
		p_ble_komoot_c->peer_komoot_db.komoot_cccd_handle = p_peer_komoot_handles->komoot_cccd_handle;
		p_ble_komoot_c->peer_komoot_db.komoot_handle = p_peer_komoot_handles->komoot_handle;
	}

	return nrf_ble_gq_conn_handle_register(p_ble_komoot_c->p_gatt_queue, conn_handle);
}
/** @}
 *  @endcond
 */
#endif // NRF_MODULE_ENABLED(BLE_KOMOOT_C)
