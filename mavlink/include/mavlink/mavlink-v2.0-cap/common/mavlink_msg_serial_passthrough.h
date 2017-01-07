// MESSAGE SERIAL_PASSTHROUGH PACKING

#define MAVLINK_MSG_ID_SERIAL_PASSTHROUGH 181

MAVPACKED(
typedef struct __mavlink_serial_passthrough_t {
 uint8_t target_system; /*< System which should execute the command*/
 uint8_t target_component; /*< Component which should execute the command, 0 for all components*/
 uint8_t confirmation; /*< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)*/
 uint8_t data[128]; /*< Data*/
 uint8_t size; /*< Data size*/
 uint8_t number; /*< Message number used for acknowledge*/
}) mavlink_serial_passthrough_t;

#define MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN 133
#define MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_MIN_LEN 133
#define MAVLINK_MSG_ID_181_LEN 133
#define MAVLINK_MSG_ID_181_MIN_LEN 133

#define MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_CRC 9
#define MAVLINK_MSG_ID_181_CRC 9

#define MAVLINK_MSG_SERIAL_PASSTHROUGH_FIELD_DATA_LEN 128

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SERIAL_PASSTHROUGH { \
	181, \
	"SERIAL_PASSTHROUGH", \
	6, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_serial_passthrough_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_serial_passthrough_t, target_component) }, \
         { "confirmation", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_serial_passthrough_t, confirmation) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 128, 3, offsetof(mavlink_serial_passthrough_t, data) }, \
         { "size", NULL, MAVLINK_TYPE_UINT8_T, 0, 131, offsetof(mavlink_serial_passthrough_t, size) }, \
         { "number", NULL, MAVLINK_TYPE_UINT8_T, 0, 132, offsetof(mavlink_serial_passthrough_t, number) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SERIAL_PASSTHROUGH { \
	"SERIAL_PASSTHROUGH", \
	6, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_serial_passthrough_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_serial_passthrough_t, target_component) }, \
         { "confirmation", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_serial_passthrough_t, confirmation) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 128, 3, offsetof(mavlink_serial_passthrough_t, data) }, \
         { "size", NULL, MAVLINK_TYPE_UINT8_T, 0, 131, offsetof(mavlink_serial_passthrough_t, size) }, \
         { "number", NULL, MAVLINK_TYPE_UINT8_T, 0, 132, offsetof(mavlink_serial_passthrough_t, number) }, \
         } \
}
#endif

/**
 * @brief Pack a serial_passthrough message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param data Data
 * @param size Data size
 * @param number Message number used for acknowledge
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_passthrough_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t confirmation, const uint8_t *data, uint8_t size, uint8_t number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, confirmation);
	_mav_put_uint8_t(buf, 131, size);
	_mav_put_uint8_t(buf, 132, number);
	_mav_put_uint8_t_array(buf, 3, data, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN);
#else
	mavlink_serial_passthrough_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.confirmation = confirmation;
	packet.size = size;
	packet.number = number;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_PASSTHROUGH;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_MIN_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_CRC);
}

/**
 * @brief Pack a serial_passthrough message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param data Data
 * @param size Data size
 * @param number Message number used for acknowledge
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_passthrough_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t confirmation,const uint8_t *data,uint8_t size,uint8_t number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, confirmation);
	_mav_put_uint8_t(buf, 131, size);
	_mav_put_uint8_t(buf, 132, number);
	_mav_put_uint8_t_array(buf, 3, data, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN);
#else
	mavlink_serial_passthrough_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.confirmation = confirmation;
	packet.size = size;
	packet.number = number;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_PASSTHROUGH;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_MIN_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_CRC);
}

/**
 * @brief Encode a serial_passthrough struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_passthrough C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_passthrough_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_passthrough_t* serial_passthrough)
{
	return mavlink_msg_serial_passthrough_pack(system_id, component_id, msg, serial_passthrough->target_system, serial_passthrough->target_component, serial_passthrough->confirmation, serial_passthrough->data, serial_passthrough->size, serial_passthrough->number);
}

/**
 * @brief Encode a serial_passthrough struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param serial_passthrough C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_passthrough_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_serial_passthrough_t* serial_passthrough)
{
	return mavlink_msg_serial_passthrough_pack_chan(system_id, component_id, chan, msg, serial_passthrough->target_system, serial_passthrough->target_component, serial_passthrough->confirmation, serial_passthrough->data, serial_passthrough->size, serial_passthrough->number);
}

/**
 * @brief Send a serial_passthrough message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param data Data
 * @param size Data size
 * @param number Message number used for acknowledge
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_passthrough_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t confirmation, const uint8_t *data, uint8_t size, uint8_t number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, confirmation);
	_mav_put_uint8_t(buf, 131, size);
	_mav_put_uint8_t(buf, 132, number);
	_mav_put_uint8_t_array(buf, 3, data, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH, buf, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_MIN_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_CRC);
#else
	mavlink_serial_passthrough_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.confirmation = confirmation;
	packet.size = size;
	packet.number = number;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_MIN_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_CRC);
#endif
}

/**
 * @brief Send a serial_passthrough message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_serial_passthrough_send_struct(mavlink_channel_t chan, const mavlink_serial_passthrough_t* serial_passthrough)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_serial_passthrough_send(chan, serial_passthrough->target_system, serial_passthrough->target_component, serial_passthrough->confirmation, serial_passthrough->data, serial_passthrough->size, serial_passthrough->number);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH, (const char *)serial_passthrough, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_MIN_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_CRC);
#endif
}

#if MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_serial_passthrough_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t confirmation, const uint8_t *data, uint8_t size, uint8_t number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, confirmation);
	_mav_put_uint8_t(buf, 131, size);
	_mav_put_uint8_t(buf, 132, number);
	_mav_put_uint8_t_array(buf, 3, data, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH, buf, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_MIN_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_CRC);
#else
	mavlink_serial_passthrough_t *packet = (mavlink_serial_passthrough_t *)msgbuf;
	packet->target_system = target_system;
	packet->target_component = target_component;
	packet->confirmation = confirmation;
	packet->size = size;
	packet->number = number;
	mav_array_memcpy(packet->data, data, sizeof(uint8_t)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH, (const char *)packet, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_MIN_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_CRC);
#endif
}
#endif

#endif

// MESSAGE SERIAL_PASSTHROUGH UNPACKING


/**
 * @brief Get field target_system from serial_passthrough message
 *
 * @return System which should execute the command
 */
static inline uint8_t mavlink_msg_serial_passthrough_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from serial_passthrough message
 *
 * @return Component which should execute the command, 0 for all components
 */
static inline uint8_t mavlink_msg_serial_passthrough_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field confirmation from serial_passthrough message
 *
 * @return 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 */
static inline uint8_t mavlink_msg_serial_passthrough_get_confirmation(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field data from serial_passthrough message
 *
 * @return Data
 */
static inline uint16_t mavlink_msg_serial_passthrough_get_data(const mavlink_message_t* msg, uint8_t *data)
{
	return _MAV_RETURN_uint8_t_array(msg, data, 128,  3);
}

/**
 * @brief Get field size from serial_passthrough message
 *
 * @return Data size
 */
static inline uint8_t mavlink_msg_serial_passthrough_get_size(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  131);
}

/**
 * @brief Get field number from serial_passthrough message
 *
 * @return Message number used for acknowledge
 */
static inline uint8_t mavlink_msg_serial_passthrough_get_number(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  132);
}

/**
 * @brief Decode a serial_passthrough message into a struct
 *
 * @param msg The message to decode
 * @param serial_passthrough C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_passthrough_decode(const mavlink_message_t* msg, mavlink_serial_passthrough_t* serial_passthrough)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	serial_passthrough->target_system = mavlink_msg_serial_passthrough_get_target_system(msg);
	serial_passthrough->target_component = mavlink_msg_serial_passthrough_get_target_component(msg);
	serial_passthrough->confirmation = mavlink_msg_serial_passthrough_get_confirmation(msg);
	mavlink_msg_serial_passthrough_get_data(msg, serial_passthrough->data);
	serial_passthrough->size = mavlink_msg_serial_passthrough_get_size(msg);
	serial_passthrough->number = mavlink_msg_serial_passthrough_get_number(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN? msg->len : MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN;
        memset(serial_passthrough, 0, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_LEN);
	memcpy(serial_passthrough, _MAV_PAYLOAD(msg), len);
#endif
}
