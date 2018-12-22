
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_LNS_C)
#include "app_util.h"
#include "app_util_bds.h"
#include "ble_lns_c.h"
#include "ble_db_discovery.h"
#include "ble_types.h"
#include "ble_srv_common.h"
#include "ble_gattc.h"

#define NRF_LOG_MODULE_NAME ble_lns_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define TX_BUFFER_MASK         0x07                  /**< TX Buffer mask, must be a mask of continuous zeroes, followed by continuous sequence of ones: 000...111. */
#define TX_BUFFER_SIZE         (TX_BUFFER_MASK + 1)  /**< Size of send buffer, which is 1 higher than the mask. */

#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */

typedef enum
{
	READ_REQ,  /**< Type identifying that this tx_message is a read request. */
	WRITE_REQ  /**< Type identifying that this tx_message is a write request. */
} tx_request_t;

/**@brief Structure for writing a message to the peer, i.e. CCCD.
 */
typedef struct
{
	uint8_t                  gattc_value[WRITE_MESSAGE_LENGTH];  /**< The message to write. */
	ble_gattc_write_params_t gattc_params;                       /**< GATTC parameters for this message. */
} write_params_t;

/**@brief Structure for holding data to be transmitted to the connected central.
 */
typedef struct
{
	uint16_t     conn_handle;  /**< Connection handle to be used when transmitting this message. */
	tx_request_t type;         /**< Type of this message, i.e. read or write message. */
	union
	{
		uint16_t       read_handle;  /**< Read request message. */
		write_params_t write_req;    /**< Write request message. */
	} req;
} tx_message_t;


static tx_message_t  m_tx_buffer[TX_BUFFER_SIZE];  /**< Transmit buffer for messages to be transmitted to the central. */
static uint32_t      m_tx_insert_index = 0;        /**< Current index in the transmit buffer where the next message should be inserted. */
static uint32_t      m_tx_index = 0;               /**< Current index in the transmit buffer from where the next message to be transmitted resides. */


static __INLINE uint8_t bds_int32_decode(const uint8_t * p_encoded_data,
		int32_t       * p_decoded_val)
{
	uint32_t tmp = 0;
	uint8_t retval = bds_uint32_decode(4, p_encoded_data, &tmp);
	*p_decoded_val = (int32_t)tmp;
	return retval;
}


/**@brief Function for passing any pending request from the buffer to the stack.
 */
static void tx_buffer_process(void)
{
	if (m_tx_index != m_tx_insert_index)
	{
		uint32_t err_code;

		if (m_tx_buffer[m_tx_index].type == READ_REQ)
		{
			err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
					m_tx_buffer[m_tx_index].req.read_handle, 0);
		}
		else
		{
			err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
					&m_tx_buffer[m_tx_index].req.write_req.gattc_params);
		}
		if (err_code == NRF_SUCCESS)
		{
			LOG_INFO("SD Read/Write API returns Success..\r\n");
			m_tx_index++;
			m_tx_index &= TX_BUFFER_MASK;
		}
		else
		{
			LOG_INFO("SD Read/Write API returns error. This message sending will be "
					"attempted again..\r\n");
		}
	}
}


/**@brief     Function for handling write response events.
 *
 * @param[in] p_ble_lns_c Pointer to the Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_write_rsp(ble_lns_c_t * p_ble_lns_c, const ble_evt_t * p_ble_evt)
{
	// Check if the event if on the link for this instance
	if (p_ble_lns_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
	{
		return;
	}

	// Check if there is any message to be sent across to the peer and send it.
	tx_buffer_process();
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

		m_tx_insert_index = m_tx_index;
	}
}


void ble_lns_c_on_db_disc_evt(ble_lns_c_t * p_ble_lns_c, const ble_db_discovery_evt_t * p_evt)
{
	// Check if the Rate Service was discovered.
	if ((p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE) &&
			p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_LOCATION_AND_NAVIGATION_SERVICE &&
			p_evt->params.discovered_db.srv_uuid.type == p_ble_lns_c->uuid_type)
	{
		// Find the CCCD Handle of the Heart Rate Measurement characteristic.
		uint32_t i;
		ble_lns_c_evt_t evt;

		LOG_INFO("Database Discovery handler called with event 0x%x\r\n", p_evt->evt_type);

		evt.conn_handle = p_evt->conn_handle;
		evt.evt_type    = BLE_LNS_C_EVT_DISCOVERY_FAILED;

		evt.evt_type    = BLE_LNS_C_EVT_DISCOVERY_COMPLETE;

		LOG_INFO("Database Discovery complete\r\n");

		for (i = 0; i < p_evt->params.discovered_db.char_count; i++)
		{

			// TODO check CHAR
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

	ble_uuid_t lns_uuid;

	lns_uuid.type = BLE_UUID_TYPE_BLE;
	lns_uuid.uuid = BLE_UUID_LOCATION_AND_NAVIGATION_SERVICE;

	p_ble_lns_c->uuid_type = BLE_UUID_TYPE_BLE;
	p_ble_lns_c->evt_handler                 = p_ble_lns_c_init->evt_handler;
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

	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GATTC_EVT_HVX:
		on_hvx(p_ble_lns_c, p_ble_evt);
		break;

	case BLE_GATTC_EVT_WRITE_RSP:
		on_write_rsp(p_ble_lns_c, p_ble_evt);
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		on_disconnected(p_ble_lns_c, p_ble_evt);
		break;

	default:
		tx_buffer_process();
		break;
	}
}


/**@brief Function for creating a message for writing to the CCCD.
 */
 static uint32_t cccd_configure(uint16_t conn_handle, uint16_t handle_cccd, bool enable)
 {
	 NRF_LOG_INFO("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d\r\n",
			 handle_cccd,conn_handle);

	 tx_message_t * p_msg;
	 uint16_t       cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;

	 p_msg              = &m_tx_buffer[m_tx_insert_index++];
	 m_tx_insert_index &= TX_BUFFER_MASK;

	 p_msg->req.write_req.gattc_params.handle   = handle_cccd;
         p_msg->req.write_req.gattc_params.len      = sizeof(cccd_val);
	 p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
	 p_msg->req.write_req.gattc_params.offset   = 0;
	 p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
	 p_msg->req.write_req.gattc_value[0]        = LSB_16(cccd_val);
	 p_msg->req.write_req.gattc_value[1]        = MSB_16(cccd_val);
	 p_msg->conn_handle                         = conn_handle;
	 p_msg->type                                = WRITE_REQ;

	 tx_buffer_process();
	 return NRF_SUCCESS;
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
	 return cccd_configure(p_ble_lns_c->conn_handle, p_ble_lns_c->peer_lns_db.lns_cccd_handle, true);
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
	 return NRF_SUCCESS;
 }
 /** @}
  *  @endcond
  */
#endif // NRF_MODULE_ENABLED(BLE_LNS_C)
