// MESSAGE SERIAL_PASSTHROUGH_ACK PACKING

#define MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK 182

MAVPACKED(
typedef struct __mavlink_serial_passthrough_ack_t {
 uint8_t number; /*< Message number to acknowledge*/
 uint8_t error; /*< Error. 0: OK, 1: Warning: unexpected message number, 10: Error: recently received - blocked*/
}) mavlink_serial_passthrough_ack_t;

#define MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN 2
#define MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_MIN_LEN 2
#define MAVLINK_MSG_ID_182_LEN 2
#define MAVLINK_MSG_ID_182_MIN_LEN 2

#define MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_CRC 247
#define MAVLINK_MSG_ID_182_CRC 247



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SERIAL_PASSTHROUGH_ACK { \
	182, \
	"SERIAL_PASSTHROUGH_ACK", \
	2, \
	{  { "number", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_serial_passthrough_ack_t, number) }, \
         { "error", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_serial_passthrough_ack_t, error) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SERIAL_PASSTHROUGH_ACK { \
	"SERIAL_PASSTHROUGH_ACK", \
	2, \
	{  { "number", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_serial_passthrough_ack_t, number) }, \
         { "error", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_serial_passthrough_ack_t, error) }, \
         } \
}
#endif

/**
 * @brief Pack a serial_passthrough_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param number Message number to acknowledge
 * @param error Error. 0: OK, 1: Warning: unexpected message number, 10: Error: recently received - blocked
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_passthrough_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t number, uint8_t error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN];
	_mav_put_uint8_t(buf, 0, number);
	_mav_put_uint8_t(buf, 1, error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN);
#else
	mavlink_serial_passthrough_ack_t packet;
	packet.number = number;
	packet.error = error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_MIN_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_CRC);
}

/**
 * @brief Pack a serial_passthrough_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param number Message number to acknowledge
 * @param error Error. 0: OK, 1: Warning: unexpected message number, 10: Error: recently received - blocked
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_passthrough_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t number,uint8_t error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN];
	_mav_put_uint8_t(buf, 0, number);
	_mav_put_uint8_t(buf, 1, error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN);
#else
	mavlink_serial_passthrough_ack_t packet;
	packet.number = number;
	packet.error = error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_MIN_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_CRC);
}

/**
 * @brief Encode a serial_passthrough_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_passthrough_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_passthrough_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_passthrough_ack_t* serial_passthrough_ack)
{
	return mavlink_msg_serial_passthrough_ack_pack(system_id, component_id, msg, serial_passthrough_ack->number, serial_passthrough_ack->error);
}

/**
 * @brief Encode a serial_passthrough_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param serial_passthrough_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_passthrough_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_serial_passthrough_ack_t* serial_passthrough_ack)
{
	return mavlink_msg_serial_passthrough_ack_pack_chan(system_id, component_id, chan, msg, serial_passthrough_ack->number, serial_passthrough_ack->error);
}

/**
 * @brief Send a serial_passthrough_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param number Message number to acknowledge
 * @param error Error. 0: OK, 1: Warning: unexpected message number, 10: Error: recently received - blocked
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_passthrough_ack_send(mavlink_channel_t chan, uint8_t number, uint8_t error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN];
	_mav_put_uint8_t(buf, 0, number);
	_mav_put_uint8_t(buf, 1, error);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK, buf, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_MIN_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_CRC);
#else
	mavlink_serial_passthrough_ack_t packet;
	packet.number = number;
	packet.error = error;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_MIN_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_CRC);
#endif
}

/**
 * @brief Send a serial_passthrough_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_serial_passthrough_ack_send_struct(mavlink_channel_t chan, const mavlink_serial_passthrough_ack_t* serial_passthrough_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_serial_passthrough_ack_send(chan, serial_passthrough_ack->number, serial_passthrough_ack->error);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK, (const char *)serial_passthrough_ack, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_MIN_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_serial_passthrough_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t number, uint8_t error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, number);
	_mav_put_uint8_t(buf, 1, error);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK, buf, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_MIN_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_CRC);
#else
	mavlink_serial_passthrough_ack_t *packet = (mavlink_serial_passthrough_ack_t *)msgbuf;
	packet->number = number;
	packet->error = error;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK, (const char *)packet, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_MIN_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE SERIAL_PASSTHROUGH_ACK UNPACKING


/**
 * @brief Get field number from serial_passthrough_ack message
 *
 * @return Message number to acknowledge
 */
static inline uint8_t mavlink_msg_serial_passthrough_ack_get_number(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field error from serial_passthrough_ack message
 *
 * @return Error. 0: OK, 1: Warning: unexpected message number, 10: Error: recently received - blocked
 */
static inline uint8_t mavlink_msg_serial_passthrough_ack_get_error(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a serial_passthrough_ack message into a struct
 *
 * @param msg The message to decode
 * @param serial_passthrough_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_passthrough_ack_decode(const mavlink_message_t* msg, mavlink_serial_passthrough_ack_t* serial_passthrough_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	serial_passthrough_ack->number = mavlink_msg_serial_passthrough_ack_get_number(msg);
	serial_passthrough_ack->error = mavlink_msg_serial_passthrough_ack_get_error(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN? msg->len : MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN;
        memset(serial_passthrough_ack, 0, MAVLINK_MSG_ID_SERIAL_PASSTHROUGH_ACK_LEN);
	memcpy(serial_passthrough_ack, _MAV_PAYLOAD(msg), len);
#endif
}
