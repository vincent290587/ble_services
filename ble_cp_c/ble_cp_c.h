
/** @file
 *
 * @defgroup ble_cp_c Cycling Power Client
 * @{
 * @ingroup ble_sdk_srv
 * @brief Cycling Power Client module.
 *
 * @details This module implements the Cycling Power (CTS) client-peripheral role of
 *          the Time Profile. After security is established, the module tries to discover the
 *          Cycling Power and its characteristic on the central side. If this succeeds,
 *          the application can trigger a read of the current time from the connected server.
 *
 *          The module informs the application about the successful discovery with the
 *          @ref BLE_CP_C_EVT_DISCOVERY_COMPLETE event. The handles for the CTS server are now
 *          available in the @ref ble_cp_c_evt_t structure. These handles must be assigned to an
 *          instance of CTS_C with @ref ble_cp_c_handles_assign. For more information about the
 *          service discovery, see the ble_discovery module documentation: @ref lib_ble_db_discovery.
 *
 *          After assigning the handles to an instance of CTS_C, the application can use the function 
 *          @ref ble_cp_c_current_time_read to read the current time. If the read succeeds, it triggers either
 *          a @ref BLE_CP_C_EVT_CURRENT_TIME event or a @ref BLE_CP_C_EVT_INVALID_TIME event
 *          (depending whether the data that was read was actually a valid time). Then the read result is sent
 *          to the application. The current time is then available in the params field of the
 *          passed @ref ble_cp_c_evt_t structure.
 *
 * @note    The application must register this module as the BLE event observer by using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_cp_c_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_CP_C_BLE_OBSERVER_PRIO,
 *                                   ble_cp_c_on_ble_evt, &instance);
 *          @endcode
 */

#ifndef BLE_CP_C_H__
#define BLE_CP_C_H__

#include "ble_srv_common.h"
#include "ble_gattc.h"
#include "ble.h"
#include "ble_db_discovery.h"
#include "nrf_ble_gq.h"
#include "nrf_sdh_ble.h"
#include <stdint.h>


#define BLE_UUID_CYCLING_POWER_SERVICE                            0x1818
#define BLE_UUID_POWER_MEASUREMENT_CHAR                           0x2A63
#define BLE_UUID_POWER_VECTOR_CHAR                                0x2A64

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_cp instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_CP_C_DEF(_name)                                                                        \
static ble_cp_c_t _name;                                                                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_CP_C_BLE_OBSERVER_PRIO,                                                   \
                     ble_cp_c_on_ble_evt, &_name)

/** @brief Macro for defining multiple ble_cp_c instances.
 *
 * @param   _name   Name of the array of instances.
 * @param   _cnt    Number of instances to define.
 * @hideinitializer
 */
#define BLE_CP_C_ARRAY_DEF(_name, _cnt)                 \
static ble_cp_c_t _name[_cnt];                          \
NRF_SDH_BLE_OBSERVERS(_name ## _obs,                     \
                      BLE_CP_C_BLE_OBSERVER_PRIO,       \
                      ble_cp_c_on_ble_evt, &_name, _cnt)

typedef struct
{
	uint8_t crank_rev_data : 1;
	uint8_t first_angle_data : 1;
	uint8_t f_mag_array_array : 1;
	uint8_t inst_torque_array : 1;
	uint8_t inst_direction_data : 2;
	uint8_t reserved : 2;
} cycling_power_vector_flags_t;

/**@brief Data structure for the Current Time characteristic. */
typedef struct
{
	union {
		cycling_power_vector_flags_t flags;
		uint8_t uFlags;
	};
	uint16_t cumul_crank_rev;
	uint16_t last_crank_evt;       // Unit is in seconds with a resolution of 1/1024.
	uint16_t first_crank_angle;    // Unit is in degrees with a resolution of 1.
	union {
		int16_t inst_force_mag_array[64];  // The unit is in newtons with a resolution of 1
		int16_t inst_torque_mag_array[64]; // Unit is in newton/meter with a resolution of 1/32
	};
	uint16_t array_size;
} cycling_power_vector_char_t;

/**@brief Data structure for the Current Time characteristic. */
typedef cycling_power_vector_char_t cycling_power_vector_evt_t;

typedef struct
{
	uint8_t pedal_power_balance : 1;
	uint8_t pedal_power_balance_ref : 1;
	uint8_t acc_torque : 1;
	uint8_t acc_torque_source : 1;
	uint8_t wheel_rev : 1;
	uint8_t crank_rev : 1;
	uint8_t extr_force_mag : 1;
	uint8_t extr_torque_mag : 1;
	uint8_t extr_angles : 1;
	uint8_t top_dead_spot_angle : 1;
	uint8_t bottom_dead_spot_angle : 1;
	uint8_t acc_energy : 1;
	uint8_t offset_comp_ind : 1;
	uint8_t reserved :3;
} cycling_power_meas_flags_t;

/**@brief Data structure for the Current Time characteristic. */
typedef struct
{
	union {
		cycling_power_meas_flags_t flags;
		uint16_t uFlags;
	};
	int16_t inst_power; // LSB 1 Watt
	uint8_t pedal_power_balance;
	uint16_t acc_torque;
	uint32_t cumul_wheel_rev;
	uint16_t last_wheel_evt; // Unit is in seconds with a resolution of 1/2048.
	uint16_t cumul_crank_rev;
	uint16_t last_crank_evt; // Unit is in seconds with a resolution of 1/1024.
	int16_t max_force_mag;   // Unit is in newtons with a resolution of 1.
	int16_t min_force_mag;   // Unit is in newtons with a resolution of 1.
	int16_t max_torque_mag;  // Unit is in newton metres with a resolution of 1/32.
	int16_t min_torque_mag;  // Unit is in newton metres with a resolution of 1/32.
	uint16_t max_extr_angle; // sent over 12 bits, degrees
	uint16_t min_extr_angle; // sent over 12 bits, degrees
	uint16_t top_dead_spot_angle;
	uint16_t bottom_dead_spot_angle;
	uint16_t acc_energy;     // Unit is in kilojoules with a resolution of 1.
} cycling_power_meas_char_t;

// Forward declaration of the ble_cp_c_t type.
typedef struct ble_cp_c_s ble_cp_c_t;

/**@brief Cycling Power client event type. */
typedef enum
{
    BLE_CP_C_EVT_DISCOVERY_COMPLETE, /**< The Cycling Power was found at the peer. */
    BLE_CP_C_EVT_DISCOVERY_FAILED,   /**< The Cycling Power was not found at the peer. */
    BLE_CP_C_EVT_DISCONN_COMPLETE,   /**< Event indicating that the Cycling Power Client module finished processing the BLE_GAP_EVT_DISCONNECTED event. This event is triggered only if a valid instance of the Cycling Power was found at the server. The application can use this event to do a cleanup related to the Cycling Power client.*/
    BLE_CP_C_EVT_VECTOR_UPDATED,
	BLE_CP_C_EVT_VECTOR_RECV,
	BLE_CP_C_EVT_POWER_RECV,
} ble_cp_c_evt_type_t;

/**@brief Structure containing the handles related to the Heart Rate Service found on the peer. */
typedef struct
{
    uint16_t vec_handle;       /**< Handle of the Current Time characteristic, as provided by the SoftDevice. */
    uint16_t vec_cccd_handle;  /**< Handle of the CCCD of the Current Time characteristic. */
    uint16_t power_handle;
    uint16_t power_cccd_handle;
} ble_cp_c_handles_t;

/**@brief Cycling Power client event. */
typedef struct
{
    ble_cp_c_evt_type_t evt_type; /**< Type of event. */
    uint16_t              conn_handle; /**< Connection handle on which the Cycling Power service was discovered on the peer device. This is filled if the evt_type is @ref BLE_CP_C_EVT_DISCOVERY_COMPLETE.*/
    union
    {
        ble_cp_c_handles_t char_handles;  /**< Handles related to CP. This is filled when the evt_type is @ref BLE_HRS_C_EVT_DISCOVERY_COMPLETE.*/
        cycling_power_vector_evt_t  vector_evt;
        cycling_power_meas_char_t   power_meas;
    } params;
} ble_cp_c_evt_t;

/**@brief Cycling Power client event handler type. */
typedef void (* ble_cp_c_evt_handler_t) (ble_cp_c_t * p_cp, ble_cp_c_evt_t * p_evt);


/**@brief Cycling Power client structure. This structure contains status information for the client. */
struct ble_cp_c_s
{
    ble_cp_c_evt_handler_t     evt_handler;         /**< Event handler to be called for handling events from the Cycling Power client. */
    ble_srv_error_handler_t    error_handler;       /**< Function to be called if an error occurs. */
    ble_cp_c_handles_t         char_handles;        /**< Handles of Current Time characteristic at the peer. (Handles are provided by the BLE stack through the Database Discovery module.) */
    uint16_t                   conn_handle;         /**< Handle of the current connection. BLE_CONN_HANDLE_INVALID if not in a connection. */
    nrf_ble_gq_t             * p_gatt_queue;        /**< Pointer to the BLE GATT Queue instance. */
    cycling_power_vector_char_t power_vector;       /**< Current power vector being constructed. */
};

/**@brief Cycling Power client init structure. This structure contains all options and data needed for the initialization of the client.*/
typedef struct
{
    ble_cp_c_evt_handler_t    evt_handler;   /**< Event handler to be called for handling events from the Cycling Power client. */
    ble_srv_error_handler_t   error_handler; /**< Function to be called if an error occurs. */
    nrf_ble_gq_t            * p_gatt_queue;  /**< Pointer to the BLE GATT Queue instance. */
} ble_cp_c_init_t;


/**@brief Function for initializing the Cycling Power client.
 *
 * @details This function must be used by the application to initialize the Cycling Power client.
 *
 * @param[out] p_cp Cycling Power client structure. This structure must
 *                   be supplied by the application. It is initialized by this
 *                   function and can later be used to identify this particular client
 *                   instance.
 * @param[in]  p_cp_init Information needed to initialize the Cycling Power client.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully.
 */
uint32_t ble_cp_c_init(ble_cp_c_t * p_cp, const ble_cp_c_init_t * p_cp_init);


/**@brief Function for handling events from the Database Discovery module.
 *
 * @details This function handles an event from the Database Discovery module, and determines
 *          whether it relates to the discovery of CTS at the peer. If it does, the function
 *          calls the application's event handler to indicate that CTS was
 *          discovered. The function also populates the event with service-related
 *          information before providing it to the application.
 *
 * @param[in] p_cp  Pointer to the CTS client structure.
 * @param[in] p_evt  Pointer to the event received from the Database Discovery module.
 */
 void ble_cp_c_on_db_disc_evt(ble_cp_c_t * p_cp, ble_db_discovery_evt_t * p_evt);


/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the
 *          Cycling Power client. This is a callback function that must be dispatched
 *          from the main application context.
 *
 * @param[in] p_ble_evt     Event received from the BLE stack.
 * @param[in] p_context     Cycling Power client structure.
 */
void ble_cp_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for checking whether the peer's Cycling Power instance and the Current Time
 *        Characteristic have been discovered.
 *
 * @param[in] p_cp  Cycling Power client structure.
 */
static __INLINE bool ble_cp_c_is_discovered(const ble_cp_c_t * p_cp)
{
    return (p_cp->char_handles.vec_handle != BLE_GATT_HANDLE_INVALID) && (p_cp->char_handles.power_handle != BLE_GATT_HANDLE_INVALID);
}


uint32_t ble_cp_c_pv_notif_enable(ble_cp_c_t * p_ble_cp_c);


/**@brief Function for assigning handles to this instance of cp_c.
 *
 * @details Call this function when a link has been established with a peer to
 *          associate the link to this instance of the module. This association makes it
 *          possible to handle several links and associate each link to a particular
 *          instance of this module. The connection handle and attribute handles are
 *          provided from the discovery event @ref BLE_CP_C_EVT_DISCOVERY_COMPLETE.
 *
 * @param[in] p_cp          Pointer to the CTS client structure instance for associating the link.
 * @param[in] conn_handle    Connection handle to associate with the given CTS instance.
 * @param[in] p_peer_handles Attribute handles for the CTS server you want this CTS client to
 *                           interact with.
 *
 * @retval NRF_SUCCESS    If the operation was successful.
 * @retval NRF_ERROR_NULL If a p_cp was a NULL pointer.
 * @retval err_code       Otherwise, this API propagates the error code returned by function
 *                        @ref nrf_ble_gq_conn_handle_register.
 */
uint32_t ble_cp_c_handles_assign(ble_cp_c_t               * p_cp,
                                  const uint16_t              conn_handle,
                                  const ble_cp_c_handles_t * p_peer_handles);

#ifdef __cplusplus
}
#endif

#endif // BLE_CP_C_H__

/** @} */
