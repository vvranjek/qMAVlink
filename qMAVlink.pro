#-------------------------------------------------
#
# Project created by QtCreator 2016-03-30T15:40:52
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

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
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/ardupilotmega.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_ahrs.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_ahrs2.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_ahrs3.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_airspeed_autocal.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_ap_adc.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_autopilot_version_request.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_battery2.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_camera_feedback.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_camera_status.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_compassmot_status.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_data16.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_data32.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_data64.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_data96.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_digicam_configure.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_digicam_control.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_ekf_status_report.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_fence_fetch_point.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_fence_point.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_fence_status.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_gimbal_control.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_gimbal_report.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_gimbal_torque_cmd_report.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_gopro_get_request.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_gopro_get_response.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_gopro_heartbeat.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_gopro_set_request.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_gopro_set_response.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_hwstatus.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_led_control.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_limits_status.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_mag_cal_progress.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_mag_cal_report.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_meminfo.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_mount_configure.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_mount_control.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_mount_status.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_pid_tuning.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_radio.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_rally_fetch_point.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_rally_point.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_rangefinder.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_remote_log_block_status.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_remote_log_data_block.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_rpm.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_sensor_offsets.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_set_mag_offsets.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_simstate.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/mavlink_msg_wind.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/testsuite.h \
    mavlink/include/mavlink/v1 (copy).0/ardupilotmega/version.h \
    mavlink/include/mavlink/v1 (copy).0/ASLUAV/ASLUAV.h \
    mavlink/include/mavlink/v1 (copy).0/ASLUAV/mavlink.h \
    mavlink/include/mavlink/v1 (copy).0/ASLUAV/mavlink_msg_asl_obctrl.h \
    mavlink/include/mavlink/v1 (copy).0/ASLUAV/mavlink_msg_aslctrl_data.h \
    mavlink/include/mavlink/v1 (copy).0/ASLUAV/mavlink_msg_aslctrl_debug.h \
    mavlink/include/mavlink/v1 (copy).0/ASLUAV/mavlink_msg_asluav_status.h \
    mavlink/include/mavlink/v1 (copy).0/ASLUAV/mavlink_msg_ekf_ext.h \
    mavlink/include/mavlink/v1 (copy).0/ASLUAV/mavlink_msg_fw_soaring_data.h \
    mavlink/include/mavlink/v1 (copy).0/ASLUAV/mavlink_msg_sens_atmos.h \
    mavlink/include/mavlink/v1 (copy).0/ASLUAV/mavlink_msg_sens_batmon.h \
    mavlink/include/mavlink/v1 (copy).0/ASLUAV/mavlink_msg_sens_mppt.h \
    mavlink/include/mavlink/v1 (copy).0/ASLUAV/mavlink_msg_sens_power.h \
    mavlink/include/mavlink/v1 (copy).0/ASLUAV/mavlink_msg_sensorpod_status.h \
    mavlink/include/mavlink/v1 (copy).0/ASLUAV/testsuite.h \
    mavlink/include/mavlink/v1 (copy).0/ASLUAV/version.h \
    mavlink/include/mavlink/v1 (copy).0/autoquad/autoquad.h \
    mavlink/include/mavlink/v1 (copy).0/autoquad/mavlink.h \
    mavlink/include/mavlink/v1 (copy).0/autoquad/mavlink_msg_aq_esc_telemetry.h \
    mavlink/include/mavlink/v1 (copy).0/autoquad/mavlink_msg_aq_telemetry_f.h \
    mavlink/include/mavlink/v1 (copy).0/autoquad/testsuite.h \
    mavlink/include/mavlink/v1 (copy).0/autoquad/version.h \
    mavlink/include/mavlink/v1 (copy).0/common/common.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_actuator_control_target.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_adsb_vehicle.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_altitude.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_att_pos_mocap.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_attitude.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_attitude_quaternion.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_attitude_quaternion_cov.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_attitude_target.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_auth_key.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_autopilot_version.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_battery_status.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_camera_trigger.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_change_operator_control.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_change_operator_control_ack.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_command_ack.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_command_int.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_command_long.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_control_system_state.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_data_stream.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_data_transmission_handshake.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_debug.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_debug_vect.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_distance_sensor.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_encapsulated_data.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_extended_sys_state.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_file_transfer_protocol.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_follow_target.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_global_position_int.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_global_position_int_cov.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_global_vision_position_estimate.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_gps2_raw.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_gps2_rtk.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_gps_global_origin.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_gps_inject_data.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_gps_raw_int.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_gps_rtk.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_gps_status.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_heartbeat.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_highres_imu.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_hil_controls.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_hil_gps.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_hil_optical_flow.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_hil_rc_inputs_raw.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_hil_sensor.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_hil_state.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_hil_state_quaternion.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_home_position.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_landing_target.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_local_position_ned.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_local_position_ned_cov.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_local_position_ned_system_global_offset.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_log_data.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_log_entry.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_log_erase.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_log_request_data.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_log_request_end.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_log_request_list.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_manual_control.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_manual_setpoint.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_memory_vect.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_message_interval.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_mission_ack.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_mission_clear_all.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_mission_count.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_mission_current.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_mission_item.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_mission_item_int.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_mission_item_reached.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_mission_request.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_mission_request_list.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_mission_request_partial_list.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_mission_set_current.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_mission_write_partial_list.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_named_value_float.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_named_value_int.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_nav_controller_output.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_optical_flow.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_optical_flow_rad.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_param_map_rc.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_param_request_list.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_param_request_read.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_param_set.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_param_value.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_ping.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_position_target_global_int.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_position_target_local_ned.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_power_status.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_radio_status.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_raw_imu.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_raw_pressure.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_rc_channels.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_rc_channels_override.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_rc_channels_raw.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_rc_channels_scaled.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_request_data_stream.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_resource_request.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_safety_allowed_area.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_safety_set_allowed_area.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_scaled_imu.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_scaled_imu2.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_scaled_imu3.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_scaled_pressure.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_scaled_pressure2.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_scaled_pressure3.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_serial_control.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_servo_output_raw.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_set_actuator_control_target.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_set_attitude_target.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_set_gps_global_origin.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_set_home_position.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_set_mode.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_set_position_target_global_int.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_set_position_target_local_ned.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_sim_state.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_statustext.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_sys_status.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_system_time.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_terrain_check.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_terrain_data.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_terrain_report.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_terrain_request.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_timesync.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_v2_extension.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_vfr_hud.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_vibration.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_vicon_position_estimate.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_vision_position_estimate.h \
    mavlink/include/mavlink/v1 (copy).0/common/mavlink_msg_vision_speed_estimate.h \
    mavlink/include/mavlink/v1 (copy).0/common/testsuite.h \
    mavlink/include/mavlink/v1 (copy).0/common/version.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/matrixpilot.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_airspeeds.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_altitudes.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_flexifunction_buffer_function.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_flexifunction_buffer_function_ack.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_flexifunction_command.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_flexifunction_command_ack.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_flexifunction_directory.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_flexifunction_directory_ack.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_flexifunction_read_req.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_flexifunction_set.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_serial_udb_extra_f13.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_serial_udb_extra_f14.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_serial_udb_extra_f15.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_serial_udb_extra_f16.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_serial_udb_extra_f2_a.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_serial_udb_extra_f2_b.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_serial_udb_extra_f4.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_serial_udb_extra_f5.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_serial_udb_extra_f6.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_serial_udb_extra_f7.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/mavlink_msg_serial_udb_extra_f8.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/testsuite.h \
    mavlink/include/mavlink/v1 (copy).0/matrixpilot/version.h \
    mavlink/include/mavlink/v1 (copy).0/minimal/mavlink.h \
    mavlink/include/mavlink/v1 (copy).0/minimal/mavlink_msg_heartbeat.h \
    mavlink/include/mavlink/v1 (copy).0/minimal/minimal.h \
    mavlink/include/mavlink/v1 (copy).0/minimal/testsuite.h \
    mavlink/include/mavlink/v1 (copy).0/minimal/version.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_boot.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_control_surface.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_cpu_load.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_ctrl_srfc_pt.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_data_log.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_diagnostic.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_gps_date_time.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_isr_location.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_mid_lvl_cmds.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_novatel_diag.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_ptz_status.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_sensor_bias.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_sensor_diag.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_slugs_camera_order.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_slugs_configuration_camera.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_slugs_mobile_location.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_slugs_navigation.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_status_gps.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_uav_status.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/mavlink_msg_volt_sensor.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/slugs.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/testsuite.h \
    mavlink/include/mavlink/v1 (copy).0/slugs/version.h \
    mavlink/include/mavlink/v1 (copy).0/test/mavlink.h \
    mavlink/include/mavlink/v1 (copy).0/test/mavlink_msg_test_types.h \
    mavlink/include/mavlink/v1 (copy).0/test/test.h \
    mavlink/include/mavlink/v1 (copy).0/test/testsuite.h \
    mavlink/include/mavlink/v1 (copy).0/test/version.h \
    mavlink/include/mavlink/v1 (copy).0/ualberta/mavlink.h \
    mavlink/include/mavlink/v1 (copy).0/ualberta/mavlink_msg_nav_filter_bias.h \
    mavlink/include/mavlink/v1 (copy).0/ualberta/mavlink_msg_radio_calibration.h \
    mavlink/include/mavlink/v1 (copy).0/ualberta/mavlink_msg_ualberta_sys_status.h \
    mavlink/include/mavlink/v1 (copy).0/ualberta/testsuite.h \
    mavlink/include/mavlink/v1 (copy).0/ualberta/ualberta.h \
    mavlink/include/mavlink/v1 (copy).0/ualberta/version.h \
    mavlink/include/mavlink/v1 (copy).0/checksum.h \
    mavlink/include/mavlink/v1 (copy).0/mavlink_conversions.h \
    mavlink/include/mavlink/v1 (copy).0/mavlink_helpers.h \
    mavlink/include/mavlink/v1 (copy).0/mavlink_types.h \
    mavlink/include/mavlink/v1 (copy).0/protocol.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/ardupilotmega.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_ahrs.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_ahrs2.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_ahrs3.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_airspeed_autocal.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_ap_adc.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_autopilot_version_request.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_battery2.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_camera_feedback.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_camera_status.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_compassmot_status.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_data16.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_data32.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_data64.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_data96.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_digicam_configure.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_digicam_control.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_ekf_status_report.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_fence_fetch_point.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_fence_point.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_fence_status.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_control.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_report.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_torque_cmd_report.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_gopro_get_request.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_gopro_get_response.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_gopro_heartbeat.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_gopro_set_request.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_gopro_set_response.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_hwstatus.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_led_control.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_limits_status.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_mag_cal_progress.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_mag_cal_report.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_meminfo.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_mount_configure.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_mount_control.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_mount_status.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_pid_tuning.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_radio.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_rally_fetch_point.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_rally_point.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_rangefinder.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_remote_log_block_status.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_remote_log_data_block.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_rpm.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_sensor_offsets.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_set_mag_offsets.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_simstate.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_wind.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/testsuite.h \
    mavlink/include/mavlink/v1.0/ardupilotmega/version.h \
    mavlink/include/mavlink/v1.0/ASLUAV/ASLUAV.h \
    mavlink/include/mavlink/v1.0/ASLUAV/mavlink.h \
    mavlink/include/mavlink/v1.0/ASLUAV/mavlink_msg_asl_obctrl.h \
    mavlink/include/mavlink/v1.0/ASLUAV/mavlink_msg_aslctrl_data.h \
    mavlink/include/mavlink/v1.0/ASLUAV/mavlink_msg_aslctrl_debug.h \
    mavlink/include/mavlink/v1.0/ASLUAV/mavlink_msg_asluav_status.h \
    mavlink/include/mavlink/v1.0/ASLUAV/mavlink_msg_ekf_ext.h \
    mavlink/include/mavlink/v1.0/ASLUAV/mavlink_msg_fw_soaring_data.h \
    mavlink/include/mavlink/v1.0/ASLUAV/mavlink_msg_sens_atmos.h \
    mavlink/include/mavlink/v1.0/ASLUAV/mavlink_msg_sens_batmon.h \
    mavlink/include/mavlink/v1.0/ASLUAV/mavlink_msg_sens_mppt.h \
    mavlink/include/mavlink/v1.0/ASLUAV/mavlink_msg_sens_power.h \
    mavlink/include/mavlink/v1.0/ASLUAV/mavlink_msg_sensorpod_status.h \
    mavlink/include/mavlink/v1.0/ASLUAV/testsuite.h \
    mavlink/include/mavlink/v1.0/ASLUAV/version.h \
    mavlink/include/mavlink/v1.0/autoquad/autoquad.h \
    mavlink/include/mavlink/v1.0/autoquad/mavlink.h \
    mavlink/include/mavlink/v1.0/autoquad/mavlink_msg_aq_esc_telemetry.h \
    mavlink/include/mavlink/v1.0/autoquad/mavlink_msg_aq_telemetry_f.h \
    mavlink/include/mavlink/v1.0/autoquad/testsuite.h \
    mavlink/include/mavlink/v1.0/autoquad/version.h \
    mavlink/include/mavlink/v1.0/cap/cap.h \
    mavlink/include/mavlink/v1.0/cap/mavlink.h \
    mavlink/include/mavlink/v1.0/cap/mavlink_msg_cap_status.h \
    mavlink/include/mavlink/v1.0/cap/testsuite.h \
    mavlink/include/mavlink/v1.0/cap/version.h \
    mavlink/include/mavlink/v1.0/common/common.h \
    mavlink/include/mavlink/v1.0/common/mavlink.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_actuator_control_target.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_adsb_vehicle.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_altitude.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_att_pos_mocap.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_attitude.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_attitude_quaternion.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_attitude_quaternion_cov.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_attitude_target.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_auth_key.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_autopilot_version.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_battery_status.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_camera_trigger.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_cap_status.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_change_operator_control.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_change_operator_control_ack.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_command_ack.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_command_int.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_command_long.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_control_system_state.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_data_stream.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_data_transmission_handshake.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_debug.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_debug_vect.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_distance_sensor.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_encapsulated_data.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_estimator_status.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_extended_sys_state.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_file_transfer_protocol.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_follow_target.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_global_position_int.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_global_position_int_cov.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_global_vision_position_estimate.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_gps2_raw.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_gps2_rtk.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_gps_global_origin.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_gps_inject_data.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_gps_raw_int.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_gps_rtk.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_gps_status.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_heartbeat.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_highres_imu.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_hil_controls.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_hil_gps.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_hil_optical_flow.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_hil_rc_inputs_raw.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_hil_sensor.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_hil_state.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_hil_state_quaternion.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_home_position.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_landing_target.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_local_position_ned.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_local_position_ned_cov.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_local_position_ned_system_global_offset.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_log_data.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_log_entry.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_log_erase.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_log_request_data.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_log_request_end.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_log_request_list.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_manual_control.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_manual_setpoint.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_memory_vect.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_message_interval.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_mission_ack.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_mission_clear_all.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_mission_count.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_mission_current.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_mission_item.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_mission_item_int.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_mission_item_reached.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_mission_request.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_mission_request_list.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_mission_request_partial_list.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_mission_set_current.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_mission_write_partial_list.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_named_value_float.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_named_value_int.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_nav_controller_output.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_optical_flow.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_optical_flow_rad.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_param_map_rc.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_param_request_list.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_param_request_read.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_param_set.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_param_value.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_ping.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_position_target_global_int.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_position_target_local_ned.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_power_status.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_radio_status.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_raw_imu.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_raw_pressure.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_rc_channels.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_rc_channels_override.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_rc_channels_raw.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_rc_channels_scaled.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_request_data_stream.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_resource_request.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_safety_allowed_area.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_safety_set_allowed_area.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_scaled_imu.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_scaled_imu2.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_scaled_imu3.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_scaled_pressure.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_scaled_pressure2.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_scaled_pressure3.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_serial_control.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_servo_output_raw.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_set_actuator_control_target.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_set_attitude_target.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_set_gps_global_origin.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_set_home_position.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_set_mode.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_set_position_target_global_int.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_set_position_target_local_ned.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_sim_state.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_statustext.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_sys_status.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_system_time.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_terrain_check.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_terrain_data.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_terrain_report.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_terrain_request.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_timesync.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_v2_extension.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_vfr_hud.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_vibration.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_vicon_position_estimate.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_vision_position_estimate.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_vision_speed_estimate.h \
    mavlink/include/mavlink/v1.0/common/mavlink_msg_wind.h \
    mavlink/include/mavlink/v1.0/common/testsuite.h \
    mavlink/include/mavlink/v1.0/common/version.h \
    mavlink/include/mavlink/v1.0/common-bak/common.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_actuator_control_target.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_adsb_vehicle.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_altitude.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_att_pos_mocap.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_attitude.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_attitude_quaternion.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_attitude_quaternion_cov.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_attitude_target.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_auth_key.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_autopilot_version.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_battery_status.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_camera_trigger.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_change_operator_control.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_change_operator_control_ack.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_command_ack.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_command_int.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_command_long.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_control_system_state.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_data_stream.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_data_transmission_handshake.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_debug.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_debug_vect.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_distance_sensor.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_encapsulated_data.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_estimator_status.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_extended_sys_state.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_file_transfer_protocol.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_follow_target.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_global_position_int.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_global_position_int_cov.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_global_vision_position_estimate.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_gps2_raw.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_gps2_rtk.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_gps_global_origin.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_gps_inject_data.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_gps_raw_int.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_gps_rtcm_data.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_gps_rtk.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_gps_status.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_heartbeat.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_highres_imu.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_hil_controls.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_hil_gps.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_hil_optical_flow.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_hil_rc_inputs_raw.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_hil_sensor.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_hil_state.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_hil_state_quaternion.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_home_position.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_landing_target.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_local_position_ned.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_local_position_ned_cov.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_local_position_ned_system_global_offset.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_log_data.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_log_entry.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_log_erase.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_log_request_data.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_log_request_end.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_log_request_list.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_manual_control.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_manual_setpoint.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_memory_vect.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_message_interval.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_mission_ack.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_mission_clear_all.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_mission_count.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_mission_current.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_mission_item.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_mission_item_int.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_mission_item_reached.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_mission_request.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_mission_request_int.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_mission_request_list.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_mission_request_partial_list.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_mission_set_current.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_mission_write_partial_list.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_named_value_float.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_named_value_int.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_nav_controller_output.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_optical_flow.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_optical_flow_rad.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_param_map_rc.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_param_request_list.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_param_request_read.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_param_set.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_param_value.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_ping.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_position_target_global_int.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_position_target_local_ned.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_power_status.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_radio_status.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_raw_imu.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_raw_pressure.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_rc_channels.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_rc_channels_override.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_rc_channels_raw.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_rc_channels_scaled.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_request_data_stream.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_resource_request.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_safety_allowed_area.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_safety_set_allowed_area.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_scaled_imu.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_scaled_imu2.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_scaled_imu3.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_scaled_pressure.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_scaled_pressure2.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_scaled_pressure3.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_serial_control.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_servo_output_raw.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_set_actuator_control_target.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_set_attitude_target.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_set_gps_global_origin.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_set_home_position.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_set_mode.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_set_position_target_global_int.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_set_position_target_local_ned.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_sim_state.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_statustext.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_sys_status.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_system_time.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_terrain_check.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_terrain_data.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_terrain_report.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_terrain_request.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_timesync.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_v2_extension.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_vfr_hud.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_vibration.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_vicon_position_estimate.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_vision_position_estimate.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_vision_speed_estimate.h \
    mavlink/include/mavlink/v1.0/common-bak/mavlink_msg_wind_cov.h \
    mavlink/include/mavlink/v1.0/common-bak/testsuite.h \
    mavlink/include/mavlink/v1.0/common-bak/version.h \
    mavlink/include/mavlink/v1.0/matrixpilot/matrixpilot.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_airspeeds.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_altitudes.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_flexifunction_buffer_function.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_flexifunction_buffer_function_ack.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_flexifunction_command.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_flexifunction_command_ack.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_flexifunction_directory.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_flexifunction_directory_ack.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_flexifunction_read_req.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_flexifunction_set.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f13.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f14.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f15.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f16.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f2_a.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f2_b.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f4.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f5.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f6.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f7.h \
    mavlink/include/mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f8.h \
    mavlink/include/mavlink/v1.0/matrixpilot/testsuite.h \
    mavlink/include/mavlink/v1.0/matrixpilot/version.h \
    mavlink/include/mavlink/v1.0/minimal/mavlink.h \
    mavlink/include/mavlink/v1.0/minimal/mavlink_msg_heartbeat.h \
    mavlink/include/mavlink/v1.0/minimal/minimal.h \
    mavlink/include/mavlink/v1.0/minimal/testsuite.h \
    mavlink/include/mavlink/v1.0/minimal/version.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_boot.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_control_surface.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_cpu_load.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_ctrl_srfc_pt.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_data_log.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_diagnostic.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_gps_date_time.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_isr_location.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_mid_lvl_cmds.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_novatel_diag.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_ptz_status.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_sensor_bias.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_sensor_diag.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_slugs_camera_order.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_slugs_configuration_camera.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_slugs_mobile_location.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_slugs_navigation.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_status_gps.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_uav_status.h \
    mavlink/include/mavlink/v1.0/slugs/mavlink_msg_volt_sensor.h \
    mavlink/include/mavlink/v1.0/slugs/slugs.h \
    mavlink/include/mavlink/v1.0/slugs/testsuite.h \
    mavlink/include/mavlink/v1.0/slugs/version.h \
    mavlink/include/mavlink/v1.0/test/mavlink.h \
    mavlink/include/mavlink/v1.0/test/mavlink_msg_test_types.h \
    mavlink/include/mavlink/v1.0/test/test.h \
    mavlink/include/mavlink/v1.0/test/testsuite.h \
    mavlink/include/mavlink/v1.0/test/version.h \
    mavlink/include/mavlink/v1.0/ualberta/mavlink.h \
    mavlink/include/mavlink/v1.0/ualberta/mavlink_msg_nav_filter_bias.h \
    mavlink/include/mavlink/v1.0/ualberta/mavlink_msg_radio_calibration.h \
    mavlink/include/mavlink/v1.0/ualberta/mavlink_msg_ualberta_sys_status.h \
    mavlink/include/mavlink/v1.0/ualberta/testsuite.h \
    mavlink/include/mavlink/v1.0/ualberta/ualberta.h \
    mavlink/include/mavlink/v1.0/ualberta/version.h \
    mavlink/include/mavlink/v1.0/checksum.h \
    mavlink/include/mavlink/v1.0/mavlink_conversions.h \
    mavlink/include/mavlink/v1.0/mavlink_helpers.h \
    mavlink/include/mavlink/v1.0/mavlink_types.h \
    mavlink/include/mavlink/v1.0/protocol.h

FORMS    += mainwindow.ui

include(qextserialport/src/qextserialport.pri)

INCLUDEPATH += mavlink/include/mavlink/mavlink-v2.0-cap
