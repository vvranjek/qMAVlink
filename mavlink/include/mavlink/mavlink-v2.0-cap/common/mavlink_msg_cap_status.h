// MESSAGE CAP_STATUS PACKING

#define MAVLINK_MSG_ID_CAP_STATUS 180

MAVPACKED(
typedef struct __mavlink_cap_status_t {
 uint16_t number_of_photos; /*< Number of photos taken in a mission*/
 uint16_t air_time; /*< Time since take-off*/
 uint8_t parachute_status; /*< Prachute status*/
 uint8_t airspeed_cal_status; /*< Airspeed sensor calibration status*/
 uint8_t home_init_status; /*< Home location status*/
 uint8_t land_detected_status; /*< Land detection status*/
 uint8_t altitude_override_status; /*< Altitude override status*/
 uint8_t speed_override_status; /*< Speed override status*/
 uint8_t failsafe_status; /*< Failsafe status*/
}) mavlink_cap_status_t;

#define MAVLINK_MSG_ID_CAP_STATUS_LEN 11
#define MAVLINK_MSG_ID_CAP_STATUS_MIN_LEN 11
#define MAVLINK_MSG_ID_180_LEN 11
#define MAVLINK_MSG_ID_180_MIN_LEN 11

#define MAVLINK_MSG_ID_CAP_STATUS_CRC 22
#define MAVLINK_MSG_ID_180_CRC 22



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAP_STATUS { \
	180, \
	"CAP_STATUS", \
	9, \
	{  { "number_of_photos", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_cap_status_t, number_of_photos) }, \
         { "air_time", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_cap_status_t, air_time) }, \
         { "parachute_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_cap_status_t, parachute_status) }, \
         { "airspeed_cal_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_cap_status_t, airspeed_cal_status) }, \
         { "home_init_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_cap_status_t, home_init_status) }, \
         { "land_detected_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_cap_status_t, land_detected_status) }, \
         { "altitude_override_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_cap_status_t, altitude_override_status) }, \
         { "speed_override_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_cap_status_t, speed_override_status) }, \
         { "failsafe_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_cap_status_t, failsafe_status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAP_STATUS { \
	"CAP_STATUS", \
	9, \
	{  { "number_of_photos", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_cap_status_t, number_of_photos) }, \
         { "air_time", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_cap_status_t, air_time) }, \
         { "parachute_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_cap_status_t, parachute_status) }, \
         { "airspeed_cal_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_cap_status_t, airspeed_cal_status) }, \
         { "home_init_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_cap_status_t, home_init_status) }, \
         { "land_detected_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_cap_status_t, land_detected_status) }, \
         { "altitude_override_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_cap_status_t, altitude_override_status) }, \
         { "speed_override_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_cap_status_t, speed_override_status) }, \
         { "failsafe_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_cap_status_t, failsafe_status) }, \
         } \
}
#endif

/**
 * @brief Pack a cap_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param parachute_status Prachute status
 * @param airspeed_cal_status Airspeed sensor calibration status
 * @param home_init_status Home location status
 * @param land_detected_status Land detection status
 * @param altitude_override_status Altitude override status
 * @param speed_override_status Speed override status
 * @param number_of_photos Number of photos taken in a mission
 * @param failsafe_status Failsafe status
 * @param air_time Time since take-off
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cap_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t parachute_status, uint8_t airspeed_cal_status, uint8_t home_init_status, uint8_t land_detected_status, uint8_t altitude_override_status, uint8_t speed_override_status, uint16_t number_of_photos, uint8_t failsafe_status, uint16_t air_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CAP_STATUS_LEN];
	_mav_put_uint16_t(buf, 0, number_of_photos);
	_mav_put_uint16_t(buf, 2, air_time);
	_mav_put_uint8_t(buf, 4, parachute_status);
	_mav_put_uint8_t(buf, 5, airspeed_cal_status);
	_mav_put_uint8_t(buf, 6, home_init_status);
	_mav_put_uint8_t(buf, 7, land_detected_status);
	_mav_put_uint8_t(buf, 8, altitude_override_status);
	_mav_put_uint8_t(buf, 9, speed_override_status);
	_mav_put_uint8_t(buf, 10, failsafe_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAP_STATUS_LEN);
#else
	mavlink_cap_status_t packet;
	packet.number_of_photos = number_of_photos;
	packet.air_time = air_time;
	packet.parachute_status = parachute_status;
	packet.airspeed_cal_status = airspeed_cal_status;
	packet.home_init_status = home_init_status;
	packet.land_detected_status = land_detected_status;
	packet.altitude_override_status = altitude_override_status;
	packet.speed_override_status = speed_override_status;
	packet.failsafe_status = failsafe_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAP_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CAP_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAP_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAP_STATUS_LEN, MAVLINK_MSG_ID_CAP_STATUS_CRC);
}

/**
 * @brief Pack a cap_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param parachute_status Prachute status
 * @param airspeed_cal_status Airspeed sensor calibration status
 * @param home_init_status Home location status
 * @param land_detected_status Land detection status
 * @param altitude_override_status Altitude override status
 * @param speed_override_status Speed override status
 * @param number_of_photos Number of photos taken in a mission
 * @param failsafe_status Failsafe status
 * @param air_time Time since take-off
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cap_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t parachute_status,uint8_t airspeed_cal_status,uint8_t home_init_status,uint8_t land_detected_status,uint8_t altitude_override_status,uint8_t speed_override_status,uint16_t number_of_photos,uint8_t failsafe_status,uint16_t air_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CAP_STATUS_LEN];
	_mav_put_uint16_t(buf, 0, number_of_photos);
	_mav_put_uint16_t(buf, 2, air_time);
	_mav_put_uint8_t(buf, 4, parachute_status);
	_mav_put_uint8_t(buf, 5, airspeed_cal_status);
	_mav_put_uint8_t(buf, 6, home_init_status);
	_mav_put_uint8_t(buf, 7, land_detected_status);
	_mav_put_uint8_t(buf, 8, altitude_override_status);
	_mav_put_uint8_t(buf, 9, speed_override_status);
	_mav_put_uint8_t(buf, 10, failsafe_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAP_STATUS_LEN);
#else
	mavlink_cap_status_t packet;
	packet.number_of_photos = number_of_photos;
	packet.air_time = air_time;
	packet.parachute_status = parachute_status;
	packet.airspeed_cal_status = airspeed_cal_status;
	packet.home_init_status = home_init_status;
	packet.land_detected_status = land_detected_status;
	packet.altitude_override_status = altitude_override_status;
	packet.speed_override_status = speed_override_status;
	packet.failsafe_status = failsafe_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAP_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CAP_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAP_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAP_STATUS_LEN, MAVLINK_MSG_ID_CAP_STATUS_CRC);
}

/**
 * @brief Encode a cap_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cap_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cap_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cap_status_t* cap_status)
{
	return mavlink_msg_cap_status_pack(system_id, component_id, msg, cap_status->parachute_status, cap_status->airspeed_cal_status, cap_status->home_init_status, cap_status->land_detected_status, cap_status->altitude_override_status, cap_status->speed_override_status, cap_status->number_of_photos, cap_status->failsafe_status, cap_status->air_time);
}

/**
 * @brief Encode a cap_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cap_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cap_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_cap_status_t* cap_status)
{
	return mavlink_msg_cap_status_pack_chan(system_id, component_id, chan, msg, cap_status->parachute_status, cap_status->airspeed_cal_status, cap_status->home_init_status, cap_status->land_detected_status, cap_status->altitude_override_status, cap_status->speed_override_status, cap_status->number_of_photos, cap_status->failsafe_status, cap_status->air_time);
}

/**
 * @brief Send a cap_status message
 * @param chan MAVLink channel to send the message
 *
 * @param parachute_status Prachute status
 * @param airspeed_cal_status Airspeed sensor calibration status
 * @param home_init_status Home location status
 * @param land_detected_status Land detection status
 * @param altitude_override_status Altitude override status
 * @param speed_override_status Speed override status
 * @param number_of_photos Number of photos taken in a mission
 * @param failsafe_status Failsafe status
 * @param air_time Time since take-off
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cap_status_send(mavlink_channel_t chan, uint8_t parachute_status, uint8_t airspeed_cal_status, uint8_t home_init_status, uint8_t land_detected_status, uint8_t altitude_override_status, uint8_t speed_override_status, uint16_t number_of_photos, uint8_t failsafe_status, uint16_t air_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CAP_STATUS_LEN];
	_mav_put_uint16_t(buf, 0, number_of_photos);
	_mav_put_uint16_t(buf, 2, air_time);
	_mav_put_uint8_t(buf, 4, parachute_status);
	_mav_put_uint8_t(buf, 5, airspeed_cal_status);
	_mav_put_uint8_t(buf, 6, home_init_status);
	_mav_put_uint8_t(buf, 7, land_detected_status);
	_mav_put_uint8_t(buf, 8, altitude_override_status);
	_mav_put_uint8_t(buf, 9, speed_override_status);
	_mav_put_uint8_t(buf, 10, failsafe_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAP_STATUS, buf, MAVLINK_MSG_ID_CAP_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAP_STATUS_LEN, MAVLINK_MSG_ID_CAP_STATUS_CRC);
#else
	mavlink_cap_status_t packet;
	packet.number_of_photos = number_of_photos;
	packet.air_time = air_time;
	packet.parachute_status = parachute_status;
	packet.airspeed_cal_status = airspeed_cal_status;
	packet.home_init_status = home_init_status;
	packet.land_detected_status = land_detected_status;
	packet.altitude_override_status = altitude_override_status;
	packet.speed_override_status = speed_override_status;
	packet.failsafe_status = failsafe_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAP_STATUS, (const char *)&packet, MAVLINK_MSG_ID_CAP_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAP_STATUS_LEN, MAVLINK_MSG_ID_CAP_STATUS_CRC);
#endif
}

/**
 * @brief Send a cap_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_cap_status_send_struct(mavlink_channel_t chan, const mavlink_cap_status_t* cap_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_cap_status_send(chan, cap_status->parachute_status, cap_status->airspeed_cal_status, cap_status->home_init_status, cap_status->land_detected_status, cap_status->altitude_override_status, cap_status->speed_override_status, cap_status->number_of_photos, cap_status->failsafe_status, cap_status->air_time);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAP_STATUS, (const char *)cap_status, MAVLINK_MSG_ID_CAP_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAP_STATUS_LEN, MAVLINK_MSG_ID_CAP_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAP_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_cap_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t parachute_status, uint8_t airspeed_cal_status, uint8_t home_init_status, uint8_t land_detected_status, uint8_t altitude_override_status, uint8_t speed_override_status, uint16_t number_of_photos, uint8_t failsafe_status, uint16_t air_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, number_of_photos);
	_mav_put_uint16_t(buf, 2, air_time);
	_mav_put_uint8_t(buf, 4, parachute_status);
	_mav_put_uint8_t(buf, 5, airspeed_cal_status);
	_mav_put_uint8_t(buf, 6, home_init_status);
	_mav_put_uint8_t(buf, 7, land_detected_status);
	_mav_put_uint8_t(buf, 8, altitude_override_status);
	_mav_put_uint8_t(buf, 9, speed_override_status);
	_mav_put_uint8_t(buf, 10, failsafe_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAP_STATUS, buf, MAVLINK_MSG_ID_CAP_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAP_STATUS_LEN, MAVLINK_MSG_ID_CAP_STATUS_CRC);
#else
	mavlink_cap_status_t *packet = (mavlink_cap_status_t *)msgbuf;
	packet->number_of_photos = number_of_photos;
	packet->air_time = air_time;
	packet->parachute_status = parachute_status;
	packet->airspeed_cal_status = airspeed_cal_status;
	packet->home_init_status = home_init_status;
	packet->land_detected_status = land_detected_status;
	packet->altitude_override_status = altitude_override_status;
	packet->speed_override_status = speed_override_status;
	packet->failsafe_status = failsafe_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAP_STATUS, (const char *)packet, MAVLINK_MSG_ID_CAP_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAP_STATUS_LEN, MAVLINK_MSG_ID_CAP_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE CAP_STATUS UNPACKING


/**
 * @brief Get field parachute_status from cap_status message
 *
 * @return Prachute status
 */
static inline uint8_t mavlink_msg_cap_status_get_parachute_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field airspeed_cal_status from cap_status message
 *
 * @return Airspeed sensor calibration status
 */
static inline uint8_t mavlink_msg_cap_status_get_airspeed_cal_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field home_init_status from cap_status message
 *
 * @return Home location status
 */
static inline uint8_t mavlink_msg_cap_status_get_home_init_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field land_detected_status from cap_status message
 *
 * @return Land detection status
 */
static inline uint8_t mavlink_msg_cap_status_get_land_detected_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field altitude_override_status from cap_status message
 *
 * @return Altitude override status
 */
static inline uint8_t mavlink_msg_cap_status_get_altitude_override_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field speed_override_status from cap_status message
 *
 * @return Speed override status
 */
static inline uint8_t mavlink_msg_cap_status_get_speed_override_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field number_of_photos from cap_status message
 *
 * @return Number of photos taken in a mission
 */
static inline uint16_t mavlink_msg_cap_status_get_number_of_photos(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field failsafe_status from cap_status message
 *
 * @return Failsafe status
 */
static inline uint8_t mavlink_msg_cap_status_get_failsafe_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field air_time from cap_status message
 *
 * @return Time since take-off
 */
static inline uint16_t mavlink_msg_cap_status_get_air_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a cap_status message into a struct
 *
 * @param msg The message to decode
 * @param cap_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_cap_status_decode(const mavlink_message_t* msg, mavlink_cap_status_t* cap_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	cap_status->number_of_photos = mavlink_msg_cap_status_get_number_of_photos(msg);
	cap_status->air_time = mavlink_msg_cap_status_get_air_time(msg);
	cap_status->parachute_status = mavlink_msg_cap_status_get_parachute_status(msg);
	cap_status->airspeed_cal_status = mavlink_msg_cap_status_get_airspeed_cal_status(msg);
	cap_status->home_init_status = mavlink_msg_cap_status_get_home_init_status(msg);
	cap_status->land_detected_status = mavlink_msg_cap_status_get_land_detected_status(msg);
	cap_status->altitude_override_status = mavlink_msg_cap_status_get_altitude_override_status(msg);
	cap_status->speed_override_status = mavlink_msg_cap_status_get_speed_override_status(msg);
	cap_status->failsafe_status = mavlink_msg_cap_status_get_failsafe_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAP_STATUS_LEN? msg->len : MAVLINK_MSG_ID_CAP_STATUS_LEN;
        memset(cap_status, 0, MAVLINK_MSG_ID_CAP_STATUS_LEN);
	memcpy(cap_status, _MAV_PAYLOAD(msg), len);
#endif
}
