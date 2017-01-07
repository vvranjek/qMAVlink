#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "qextserialport.h"
#include "common/mavlink.h"
//#include "cap/cap.h"
#include "qelapsedtimer.h"

namespace Ui {
class MainWindow;
}


// Struct containing information on the MAV we are currently connected to

struct Mavlink_Messages {

    int sysid;
    int compid;

    // Heartbeat
    mavlink_heartbeat_t heartbeat;
    // System Status
    mavlink_sys_status_t sys_status;
    // Battery Status
    mavlink_battery_status_t battery_status;
    // Radio Status
    mavlink_radio_status_t radio_status;
    // Local Position
    mavlink_local_position_ned_t local_position_ned;
    // Global Position
    mavlink_global_position_int_t global_position_int;
    // Local Position Target
    mavlink_position_target_local_ned_t position_target_local_ned;
    // Global Position Target
    mavlink_position_target_global_int_t position_target_global_int;
    // HiRes IMU
    mavlink_highres_imu_t highres_imu;
    // Attitude
    mavlink_attitude_t attitude;

    mavlink_rc_channels_t rc_channels;

    mavlink_rc_channels_override_t rc_channels_override;

    mavlink_param_request_list_t param_request_list;

    mavlink_command_long_t command_long;

    mavlink_altitude_t altitude;

    mavlink_vibration_t vibration;

    mavlink_extended_sys_state_t extended_sys_state;

    mavlink_mission_request_list_t mission_request_list;
    mavlink_param_set_t param_set;
    mavlink_mission_ack_t mission_ack;
    mavlink_statustext_t statustext;
    mavlink_set_mode_t set_mode;
    mavlink_param_value_t param_value;
    //mavlink_wind_cov_t wind;
    mavlink_cap_status_t cap_status;
    mavlink_serial_passthrough_t serial_passthrough;
    mavlink_wind_cov_t wind_cov;
    mavlink_nav_controller_output_t nav_controller_output;


    // System Parameters?


    // Time Stamps
  //  Time_Stamps time_stamps;

    void
    reset_timestamps()
    {
 //       time_stamps.reset_timestamps();
    }

};

struct Time_Stamps
{
    Time_Stamps()
    {
        reset_timestamps();
    }

    uint64_t heartbeat;
    uint64_t sys_status;
    uint64_t battery_status;
    uint64_t radio_status;
    uint64_t local_position_ned;
    uint64_t global_position_int;
    uint64_t position_target_local_ned;
    uint64_t position_target_global_int;
    uint64_t highres_imu;
    uint64_t attitude;
    uint64_t rc_channels;
    uint64_t rc_channels_override;

    void reset_timestamps()
    {
        heartbeat = 0;
        sys_status = 0;
        battery_status = 0;
        radio_status = 0;
        local_position_ned = 0;
        global_position_int = 0;
        position_target_local_ned = 0;
        position_target_global_int = 0;
        highres_imu = 0;
        attitude = 0;
    }

};



class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    QextSerialPort * port = new QextSerialPort();

    // MAVlink
    Mavlink_Messages current_messages;
    char reading_status;
    char writing_status;
    char control_status;
    uint64_t write_count;
    bool debug;
    char *uart_name;
    int  baudrate;
    int  status;
    int system_id;
    int autopilot_id;
    int companion_id;

    QElapsedTimer etimer;

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

    void main_slot();
    void read_messages();
    int read_message(mavlink_message_t &message);
    int write_message(mavlink_message_t &message);

    int _refresh_serial();
    int connect_serial();

    void heartbeat();

    void on_PortRefresh_released();
    void on_sendButton_released();
    void on_portBox_currentIndexChanged(const QString &arg1);
    void on_connectButton_released();
    void on_pushButton_released();

    void on_parachuteButton_released();

    void on_setServoButton_released();

    void on_setHome_button_released();

    void on_forward_serialButton_released();

    void on_serialpassthroughClear_released();

private:
    bool success;
    bool time_to_exit;
    QTimer* tmrTimer;
    QTimer *heartbeatTimer;
    QTimer* tmr_serial_timeout;


    // MAVlink
    int  fd;
    mavlink_status_t lastStatus;
    pthread_mutex_t  lock;

    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
