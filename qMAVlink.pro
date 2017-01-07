#-------------------------------------------------
#
# Project created by QtCreator 2016-03-30T15:40:52
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 5): QT += widgets
# Works with 5.2.1

TARGET = qMAVlink
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/common.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_actuator_control_target.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_adsb_vehicle.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_altitude.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_att_pos_mocap.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_attitude.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_attitude_quaternion.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_attitude_quaternion_cov.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_attitude_target.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_auth_key.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_autopilot_version.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_battery_status.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_camera_trigger.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_cap_status.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_change_operator_control.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_change_operator_control_ack.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_command_ack.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_command_int.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_command_long.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_control_system_state.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_data_stream.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_data_transmission_handshake.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_debug.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_debug_vect.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_distance_sensor.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_encapsulated_data.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_estimator_status.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_extended_sys_state.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_file_transfer_protocol.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_follow_target.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_global_position_int.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_global_position_int_cov.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_global_vision_position_estimate.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_gps2_raw.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_gps2_rtk.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_gps_global_origin.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_gps_inject_data.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_gps_raw_int.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_gps_rtcm_data.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_gps_rtk.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_gps_status.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_heartbeat.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_highres_imu.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_hil_controls.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_hil_gps.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_hil_optical_flow.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_hil_rc_inputs_raw.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_hil_sensor.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_hil_state.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_hil_state_quaternion.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_home_position.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_landing_target.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_local_position_ned.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_local_position_ned_cov.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_local_position_ned_system_global_offset.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_log_data.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_log_entry.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_log_erase.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_log_request_data.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_log_request_end.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_log_request_list.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_manual_control.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_manual_setpoint.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_memory_vect.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_message_interval.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_mission_ack.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_mission_clear_all.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_mission_count.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_mission_current.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_mission_item.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_mission_item_int.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_mission_item_reached.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_mission_request.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_mission_request_int.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_mission_request_list.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_mission_request_partial_list.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_mission_set_current.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_mission_write_partial_list.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_named_value_float.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_named_value_int.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_nav_controller_output.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_optical_flow.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_optical_flow_rad.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_param_map_rc.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_param_request_list.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_param_request_read.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_param_set.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_param_value.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_ping.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_position_target_global_int.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_position_target_local_ned.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_power_status.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_radio_status.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_raw_imu.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_raw_pressure.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_rc_channels.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_rc_channels_override.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_rc_channels_raw.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_rc_channels_scaled.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_request_data_stream.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_resource_request.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_safety_allowed_area.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_safety_set_allowed_area.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_scaled_imu.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_scaled_imu2.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_scaled_imu3.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_scaled_pressure.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_scaled_pressure2.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_scaled_pressure3.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_serial_control.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_servo_output_raw.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_set_actuator_control_target.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_set_attitude_target.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_set_gps_global_origin.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_set_home_position.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_set_mode.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_set_position_target_global_int.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_set_position_target_local_ned.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_setup_signing.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_sim_state.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_statustext.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_sys_status.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_system_time.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_terrain_check.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_terrain_data.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_terrain_report.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_terrain_request.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_timesync.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_v2_extension.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_vfr_hud.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_vibration.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_vicon_position_estimate.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_vision_position_estimate.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_vision_speed_estimate.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/mavlink_msg_wind_cov.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/testsuite.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/common/version.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/checksum.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/mavlink_conversions.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/mavlink_get_info.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/mavlink_helpers.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/mavlink_sha256.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/mavlink_types.h \
    mavlink/include/mavlink/mavlink-v2.0-cap/protocol.h \


FORMS    += mainwindow.ui

include(qextserialport/src/qextserialport.pri)

INCLUDEPATH += mavlink/include/mavlink/mavlink-v2.0-cap
