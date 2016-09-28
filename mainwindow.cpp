#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QTimer"
#include "qextserialenumerator.h"
#include "iostream"
#include "fstream"
#include <sys/time.h>
#include <QThread>
#include <QLabel>
#include <string>
#include <ostream>


#define PRINT_MAVLINK_ID(x)   ui->mavlinkIn->append(x+QString::fromLatin1("   ")+QString::number(etimer.elapsed()));
#define PRINT_MAVLINK_DATA(x)   ui->mavlinkIn->append(QString::fromLatin1("   ")+x);



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    on_PortRefresh_released();
    
    tmrTimer = new QTimer(this);
    connect(tmrTimer,  SIGNAL(timeout()),this,SLOT(main_slot()));
    tmrTimer->start(30);
    heartbeatTimer = new QTimer(this);
    connect(heartbeatTimer,  SIGNAL(timeout()),this,SLOT(heartbeat()));
    heartbeatTimer->start(100);

    //etimer = new QElapsedTimer(this);
    etimer.start();

    debug = 1;


    QStringList _baud_rates;
    _baud_rates << "4800" << "9600" << "19200" << "38400" << "57600" << "115200" << "230400";
    ui->baudBox->addItems(_baud_rates);
    ui->baudBox->setCurrentIndex(ui->baudBox->findText("57600"));
    ui->parachuteSpinBox->setValue(3);
    ui->forwardSerialtextEdit->setText("Testing");
    ui->ignoreRegularCheckBox->setChecked(1);

    _refresh_serial();
    connect_serial();

}

void MainWindow::main_slot()
{
    //printf("test\n");
    bool *ok;
    //ui->serialBrowser->append(port->readAll().toHex());
    
    read_messages();
}

void MainWindow::on_PortRefresh_released()
{
    _refresh_serial();
}


void MainWindow::read_messages()
{
    bool success;               // receive success flag
    bool received_all = false;  // receive only one message
    Time_Stamps this_timestamps;
    
    // Log filr
    std::ofstream mavlinkFile;
    mavlinkFile.open ("log.txt", std::ofstream::out | std::ofstream::app);
    
    // Blocking wait for new data
    static int stop;
    stop = 0;
    //while (!received_all && stop < 1000) //Temp solution with 'stop'
    while (!port->atEnd() && stop < 100) //Temp solution with 'stop'
    {
        stop++;
        if (stop > 100){

        }
        
        //   READ MESSAGE
        mavlink_message_t message;
        success = read_message(message);
        
        // Mavlink logfile
        timeval tv;
        gettimeofday (&tv, NULL);
        uint64_t time = tv.tv_usec;
        
        //   HANDLE MESSAGE
        if( success )
        {
            // Store message sysid and compid.
            // Note this doesn't handle multiple message sources.
            current_messages.sysid  = message.sysid;
            current_messages.compid = message.compid;
            
            // Handle Message ID
            switch (message.msgid)
            {
            case MAVLINK_MSG_ID_HEARTBEAT:
            {
                ui->mavlinkIn->append(" ");
                PRINT_MAVLINK_ID("HEARTBEAT ( #0 )");
                mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
//                PRINT_MAVLINK_DATA("sysid: " + QString::number(message.sysid));
//                PRINT_MAVLINK_DATA("compid: " + QString::number(message.compid));
//                PRINT_MAVLINK_DATA("msgid: " + QString::number(message.msgid));
//                PRINT_MAVLINK_DATA("seq: " + QString::number(message.seq));
//                PRINT_MAVLINK_DATA("custom: " + QString::number(current_messages.heartbeat.custom_mode));
                break;
            }
            case MAVLINK_MSG_ID_SYS_STATUS:            {
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("SYS_STATUS ( #1 )");
                mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
                break;
            }
            case MAVLINK_MSG_ID_BATTERY_STATUS:            {
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("BATTERY_STATUS ( #147 )");
                mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
                break;
            }
            case MAVLINK_MSG_ID_RADIO_STATUS:            {
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("RADIO_STATUS ( #109 )");
                mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
                break;
            }
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED:            {
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("LOCAL_POSITION_NED ( #32 )");
                mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
                break;
            }
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:            {
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("GLOBAL_POSITION_INT ( #33 )");
                mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
                break;
            }
            case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:            {
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("SET_POSITION_TARGET_LOCAL_NED ( #84 )");
                mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
                break;
            }
            case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:            {
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("POSITION_TARGET_GLOBAL_INT ( #87 )");
                mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
                break;
            }
            case MAVLINK_MSG_ID_HIGHRES_IMU:            {
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("HIGHRES_IMU ( #105 )");
                mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
                break;
            }
            case MAVLINK_MSG_ID_ATTITUDE:            {
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("ATTITUDE ( #30 )");
                mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
                break;
            }
            case MAVLINK_MSG_ID_RC_CHANNELS:            {
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("RC_CHANNELS ( #65 )");
                mavlink_msg_rc_channels_decode(&message, &(current_messages.rc_channels));
                break;
            }
            case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:            {
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("RC_CHANNELS_OVERRIDE ( #70 )");
                mavlink_msg_rc_channels_override_decode(&message, &(current_messages.rc_channels_override));
                break;
            }
            case MAVLINK_MSG_ID_ATTITUDE_TARGET:            {
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("ATTITUDE_TARGET ( #83 )");
                mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
                break;
            }
            case MAVLINK_MSG_ID_VFR_HUD:            {
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("VFR_HUD ( #74 )");
                break;
            }
            case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:            {
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("SERVO_OUTPUT_RAW ( #36 )");
                break;
            }
            case MAVLINK_MSG_ID_MISSION_CURRENT:            {
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("MISSION_CURRENT ( #42 )");
                break;
            }
            case MAVLINK_MSG_ID_GPS_RAW_INT:            {
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("GPS_RAW_INT ( #24 )");
                break;
            }
            case MAVLINK_MSG_ID_ALTITUDE:{
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("ALTITUDE (141)");
                mavlink_msg_altitude_decode(&message, &(current_messages.altitude));
                break;
            }

            case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:{
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("EXTENDED_SYS_STATE ( #245 )");
                mavlink_msg_extended_sys_state_decode(&message, &(current_messages.extended_sys_state));
                break;
            }
            case MAVLINK_MSG_ID_VIBRATION:{
                if(!ui->ignoreRegularCheckBox->isChecked())
                PRINT_MAVLINK_ID("VIBRATION (241)");
                mavlink_msg_vibration_decode(&message, &(current_messages.vibration));
                break;
            }
            case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:{
                PRINT_MAVLINK_ID("MISSION_REQUEST_LIST ( #43 )");
                mavlink_msg_mission_request_list_decode(&message, &(current_messages.mission_request_list));
                break;
            }
            case MAVLINK_MSG_ID_PARAM_SET:{
                PRINT_MAVLINK_ID("PARAM_SET ( #23 )");
                mavlink_msg_param_set_decode(&message, &(current_messages.param_set));
                PRINT_MAVLINK_DATA("param_id: "+QString::fromLatin1(current_messages.param_set.param_id));
                PRINT_MAVLINK_DATA("param_type: " + QString::number(current_messages.param_set.param_type));
                PRINT_MAVLINK_DATA("param_value: " + QString::number(current_messages.param_set.param_value));
                PRINT_MAVLINK_DATA("target_component: " + QString::number(current_messages.param_set.target_component));
                PRINT_MAVLINK_DATA("target_system: " + QString::number(current_messages.param_set.target_system));
                break;
            }
            case MAVLINK_MSG_ID_MISSION_ACK:{
                PRINT_MAVLINK_ID("MISSION_ACK ( #47 )");
                mavlink_msg_mission_ack_decode(&message, &(current_messages.mission_ack));
                PRINT_MAVLINK_DATA("type: " + QString::number(current_messages.mission_ack.type));
                break;
                }
            case MAVLINK_MSG_ID_STATUSTEXT:{
                PRINT_MAVLINK_ID("STATUSTEXT ( #253 )");

                mavlink_msg_statustext_decode(&message, &(current_messages.statustext));
                ui->statusText->append("  (" + QString::number(current_messages.statustext.severity) + ") ");
                ui->statusText->insertPlainText(QString::fromLocal8Bit(current_messages.statustext.text));
                printf("STATUS: %s\n", current_messages.statustext.text);
                break;
            }
            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:            {
                PRINT_MAVLINK_ID("PARAM_REQUEST_LIST ( #21 )");
                mavlink_msg_param_request_list_decode(&message, &(current_messages.param_request_list));
                break;
            }
            case MAVLINK_MSG_ID_COMMAND_LONG:            {
                PRINT_MAVLINK_ID("COMMAND_LONG ( #76 )");
                mavlink_msg_command_long_decode(&message, &(current_messages.command_long));
                PRINT_MAVLINK_DATA("sysid: " + QString::number(message.sysid));
                PRINT_MAVLINK_DATA("compid: " + QString::number(message.compid));
                PRINT_MAVLINK_DATA("seq: " + QString::number(message.seq));
                PRINT_MAVLINK_DATA("checksum: " + QString::number(message.checksum));
                PRINT_MAVLINK_DATA("magic: " + QString::number(message.magic));
                PRINT_MAVLINK_DATA("len: " + QString::number(message.len));
                for(int i = 0; i <= message.len; i++){
                    PRINT_MAVLINK_DATA("payload" + QString::number(i) + ": " + QString::number(message.payload64[i]));
                }
                PRINT_MAVLINK_DATA("command: " + QString::number(current_messages.command_long.command));
                PRINT_MAVLINK_DATA("confirmation: " + QString::number(current_messages.command_long.confirmation));
                PRINT_MAVLINK_DATA("target_component: " + QString::number(current_messages.command_long.target_component));
                PRINT_MAVLINK_DATA("target_system: " + QString::number(current_messages.command_long.target_system));
                PRINT_MAVLINK_DATA("param1: " + QString::number(current_messages.command_long.param1));
                PRINT_MAVLINK_DATA("param2: " + QString::number(current_messages.command_long.param2));
                PRINT_MAVLINK_DATA("param3: " + QString::number(current_messages.command_long.param3));
                PRINT_MAVLINK_DATA("param4: " + QString::number(current_messages.command_long.param4));
                PRINT_MAVLINK_DATA("param5: " + QString::number(current_messages.command_long.param5));
                PRINT_MAVLINK_DATA("param6: " + QString::number(current_messages.command_long.param6));
                PRINT_MAVLINK_DATA("param7: " + QString::number(current_messages.command_long.param7));
                break;
            }
            case MAVLINK_MSG_ID_SET_MODE:            {
                PRINT_MAVLINK_ID("SET_MODE ( #11 )");
                mavlink_msg_set_mode_decode(&message, &(current_messages.set_mode));
                PRINT_MAVLINK_DATA("base_mode: " + QString::number(current_messages.set_mode.base_mode));
                PRINT_MAVLINK_DATA("custom_mode: " + QString::number(current_messages.set_mode.custom_mode));
                PRINT_MAVLINK_DATA("target_system: " + QString::number(current_messages.set_mode.target_system));
                break;
            }
            case MAVLINK_MSG_ID_PARAM_VALUE:            {
                PRINT_MAVLINK_ID("PARAM_VALUE ( #22 )");
                mavlink_msg_param_value_decode(&message, &(current_messages.param_value));
                PRINT_MAVLINK_DATA("param_index: " + QString::number(current_messages.param_value.param_index));
                PRINT_MAVLINK_DATA("param_count: " + QString::number(current_messages.param_value.param_count));
                PRINT_MAVLINK_DATA("param_type: " + QString::number(current_messages.param_value.param_type));
                PRINT_MAVLINK_DATA("param_value: " + QString::number(current_messages.param_value.param_value));
                break;
            }
            case MAVLINK_MSG_ID_CAP_STATUS:            {
                PRINT_MAVLINK_ID("CAP_STATUS");
                mavlink_msg_cap_status_decode(&message, &(current_messages.cap_status));
                PRINT_MAVLINK_DATA("Parachute: " + QString::number(current_messages.cap_status.parachute_status));
                PRINT_MAVLINK_DATA("Home_init: " + QString::number(current_messages.cap_status.home_init_status));
                PRINT_MAVLINK_DATA("Airspeed_init: " + QString::number(current_messages.cap_status.airspeed_cal_status));

                break;
            }
            case MAVLINK_MSG_ID_SERIAL_PASSTHROUGH:            {
                PRINT_MAVLINK_ID("SERIAL_FORWARD");
                mavlink_msg_serial_passthrough_decode(&message, &(current_messages.serial_passthrough));
               // std::string str((char*)current_messages.serial_passthrough.data, (char*)current_messages.serial_passthrough.data + current_messages.serial_passthrough.size);

                char* temp = (char*)current_messages.serial_passthrough.data;


                std::string str(temp, temp+current_messages.serial_passthrough.size);
                //std::string str(temp, temp + 5);

                ui->serialPassthroughText->append(QString::fromUtf8(str.c_str()));


                PRINT_MAVLINK_DATA(QString::fromUtf8(str.c_str()));
                PRINT_MAVLINK_DATA("Size: " + QString::number(current_messages.serial_passthrough.size));
                PRINT_MAVLINK_DATA("Number: " + QString::number(current_messages.serial_passthrough.number));

                /* Send ACK */

                mavlink_serial_passthrough_ack_t cmd;

                cmd.number = current_messages.serial_passthrough.number;
                cmd.error = 0;

                mavlink_message_t msg;
                mavlink_msg_serial_passthrough_ack_encode(255, 1, &msg, &cmd);
                write_message(msg);

                break;
            }

            case MAVLINK_MSG_ID_WIND_COV:            {
                if(!ui->ignoreRegularCheckBox->isChecked()) {
                    PRINT_MAVLINK_ID("WIND_COV");
                    mavlink_msg_wind_cov_decode(&message, &(current_messages.wind_cov));
                }
                break;
            }
            case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:            {
                if(!ui->ignoreRegularCheckBox->isChecked())
                    PRINT_MAVLINK_ID("NAV_CONTROLLER_OUTPUT");
                //mavlink_msg__decode(&message, &(current_messages.highres_imu));
                break;
            }
                //            case MAVLINK_MSG_ID_:            {
                //            if(!ui->ignoreRegularCheckBox->isChecked()) {
                //                PRINT_MAVLINK_ID("");
                //                //mavlink_msg__decode(&message, &(current_messages.highres_imu));
                //            }
                //                break;
                //            }
                //            case MAVLINK_MSG_ID_:            {
                //            if(!ui->ignoreRegularCheckBox->isChecked()) {
                //                PRINT_MAVLINK_ID("");
                //                //mavlink_msg__decode(&message, &(current_messages.highres_imu));
                //            }
                //                break;
                //            }
                //            case MAVLINK_MSG_ID_:            {
                //            if(!ui->ignoreRegularCheckBox->isChecked()) {
                //                PRINT_MAVLINK_ID("");
                //                //mavlink_msg__decode(&message, &(current_messages.highres_imu));
                //            }
                //                break;
                //            }
                //            case MAVLINK_MSG_ID_:            {
                //            if(!ui->ignoreRegularCheckBox->isChecked()) {
                //                PRINT_MAVLINK_ID("");
                //                //mavlink_msg__decode(&message, &(current_messages.highres_imu));
                //            }
                //                break;
                //            }
                //            case MAVLINK_MSG_ID_:            {
                //            if(!ui->ignoreRegularCheckBox->isChecked()) {
                //                PRINT_MAVLINK_ID("");
                //                //mavlink_msg__decode(&message, &(current_messages.highres_imu));
                //            }
                //                break;
                //            }



            default:
            {
                ui->debugText->append("Warning!!!! Did not handle message " + QString::number(message.msgid));
                printf("Warning!!!! Did not handle message %i\n", message.msgid);
                
                break;
            }
                
            } // end: switch msgid
            
        } // end: if read message
        
        // Check for receipt of all items
        received_all =
                this_timestamps.heartbeat                  &&
                this_timestamps.sys_status                 &&
                //				this_timestamps.battery_status             &&
                //				this_timestamps.radio_status               &&
                this_timestamps.local_position_ned         &&
                //				this_timestamps.global_position_int        &&
                //				this_timestamps.position_target_local_ned  &&
                this_timestamps.position_target_global_int &&
                this_timestamps.highres_imu                &&
                this_timestamps.attitude                   ;
        
        // give the write thread time to use the port
        if ( writing_status > false )
            QThread::usleep(100); // look for components of batches at 10kHz
        
    } // end: while not received all
    
    mavlinkFile.close();
    return;
}

void MainWindow::heartbeat(){
    if(ui->heatbeatCheckBox->isChecked()){

        mavlink_message_t msg;
        mavlink_heartbeat_t cmd;

        mavlink_msg_heartbeat_encode(255, 1, &msg, &cmd);

        write_message(msg);
    }
}

int MainWindow::read_message(mavlink_message_t &message)
{
    uint8_t          cp;
    char            cp_temp;
    mavlink_status_t status;
    uint8_t          msgReceived = false;
    
    //   READ FROM PORT
    // this function locks the port during read
    int result = port->read(&cp_temp, 1);
    cp = cp_temp;

    ui->serialBrowser->append(QString::number(cp_temp));
    ui->serialBrowser->append("\t");
    ui->serialBrowser->append(QString::number(cp_temp).toLatin1());

    //   PARSE MESSAGE
    if (result > 0)
    {
        // the parsing
        msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

        // check for dropped packets
        if ( (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
        {
            printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
            unsigned char v=cp;
            fprintf(stderr,"%02x ", v);
        }
        lastStatus = status;
    }
    
    // Couldn't read from port
    else
    {
        // fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
    }
    
    //   DEBUGGING REPORTS
    if(msgReceived && debug)
    {
        // Report info
        printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
        
        fprintf(stderr,"Received serial data: ");
        unsigned int i;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        
        // check message is write length
        unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
        
        // message length error
        if (messageLength > MAVLINK_MAX_PACKET_LEN)
        {
            fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
        }
        
        // print out the buffer
        else
        {
            for (i=0; i<messageLength; i++)
            {
                unsigned char v=buffer[i];
                fprintf(stderr,"%02x ", v);
            }
            fprintf(stderr,"\n");
        }
    }
    
    // Done!
    return msgReceived;
}

// ------------------------------------------------------------------------------
//   Write to Serial
// ------------------------------------------------------------------------------
int MainWindow::write_message(mavlink_message_t &message)
{
    char buf[300];

    // Translate message to buffer
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

    port->write(buf,len);

    // Write buffer to serial port, locks port while writing
    //_write_port(buf,len);

    return len;
}



MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_sendButton_released()
{
    // We specifically use COMMAND_LONG:MAV_CMD_COMPONENT_ARM_DISARM since it is supported by more flight stacks.

    mavlink_message_t msg;
    mavlink_command_long_t cmd;
    bool armed = 1;

    cmd.command = (uint16_t)MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.confirmation = 0;
    cmd.target_component = 0;
    cmd.param1 = 1;
    cmd.param2 = 0.0f;
    cmd.param3 = 0.0f;
    cmd.param4 = 0.0f;
    cmd.param5 = 0.0f;
    cmd.param6 = 0.0f;
    cmd.param7 = 0.0f;
    cmd.target_system = 1;
    cmd.target_component = 0;

    mavlink_msg_command_long_encode(255, 0, &msg, &cmd);
    write_message(msg);
}

void MainWindow::on_portBox_currentIndexChanged(const QString &arg1)
{
    // _refresh_serial();
}


int MainWindow::_refresh_serial(){
    ui->portBox->clear();
    // Serch for serial ports and put them in portBox
    foreach (QextPortInfo info, QextSerialEnumerator::getPorts()){
        if(info.portName.contains("ACM") || info.portName.contains("USB")) {
            ui->portBox->addItem(info.portName);
            ui->portBox->setCurrentIndex(ui->portBox->findText(info.portName));
        }
    }
}

int MainWindow::connect_serial(){

    port->close();

    port->setBaudRate(ui->baudBox->currentText().toInt());
    port->setPortName(ui->portBox->currentText());

    if (!port->open(QIODevice::ReadWrite)){
        printf("Connected to %s", ui->portBox->currentText().toStdString());
        return 0;
    }
    else {
        return 1;
    }
}

void MainWindow::on_connectButton_released() {
    if (!ui->connectButton->isChecked()){
        port->close();
    }
    else {
        connect_serial();
    }
}

void MainWindow::on_pushButton_released()
{
    ui->mavlinkIn->clear();
}

void MainWindow::on_parachuteButton_released()
{
    mavlink_message_t msg;
    mavlink_command_long_t cmd;

    cmd.command = (uint16_t)MAV_CMD_DO_PARACHUTE;
    cmd.confirmation = 1;
    cmd.target_component = 0;
    cmd.param1 = ui->parachuteSpinBox->value();
    cmd.param2 = 0.0f;
    cmd.param3 = 0.0f;
    cmd.param4 = 0.0f;
    cmd.param5 = 0.0f;
    cmd.param6 = 0.0f;
    cmd.param7 = 0.0f;
    cmd.target_system = 1;
    cmd.target_component = 0;

    mavlink_msg_command_long_encode(255, 0, &msg, &cmd);
    write_message(msg);
}

void MainWindow::on_setServoButton_released()
{

    mavlink_message_t msg;
    mavlink_command_long_t cmd;

    cmd.command = (uint16_t)MAV_CMD_DO_PARACHUTE;
    cmd.confirmation = 1;
    cmd.target_component = 0;
    cmd.param1 = ui->parachuteSpinBox->value();
    cmd.param2 = 0.0f;
    cmd.param3 = 0.0f;
    cmd.param4 = 0.0f;
    cmd.param5 = 0.0f;
    cmd.param6 = 0.0f;
    cmd.param7 = 0.0f;
    cmd.target_system = 1;
    cmd.target_component = 0;

    mavlink_msg_command_long_encode(255, 0, &msg, &cmd);
    write_message(msg);





//    mavlink_message_t msg;
//    mavlink_command_long_t cmd;

//    cmd.command = (uint16_t)MAV_CMD_DO_SET_SERVO;
//    cmd.confirmation = 1;
//    cmd.target_component = 0;
//    cmd.param1 = ui->setServoNUmberSpinBox->value();
//    cmd.param2 = ui->setServoSpinBox->value();
//    cmd.param3 = 0.0f;
//    cmd.param4 = 0.0f;
//    cmd.param5 = 0.0f;
//    cmd.param6 = 0.0f;
//    cmd.param7 = 0.0f;
//    cmd.target_system = 1;
//    cmd.target_component = 0;

//    mavlink_msg_command_long_encode(255, 0, &msg, &cmd);
//    write_message(msg);
}

void MainWindow::on_setHome_button_released()
{
    mavlink_message_t msg;
    mavlink_command_long_t cmd;

    cmd.command = (uint16_t)MAV_CMD_DO_SET_HOME;
    cmd.confirmation = 1;
    cmd.target_component = 0;
    cmd.param1 = 0.0f;
    cmd.param2 = 0.0f;
    cmd.param3 = 0.0f;
    cmd.param4 = 0.0f;
    cmd.param5 = 46.073126f;
    cmd.param6 = 14.514700f;
    cmd.param7 = 300.0f;
    cmd.target_system = 1;
    cmd.target_component = 0;

    mavlink_msg_command_long_encode(255, 0, &msg, &cmd);
    write_message(msg);
}

void MainWindow::on_forward_serialButton_released()
{
    static mavlink_serial_passthrough_t cmd;
    mavlink_message_t msg;

    static unsigned char number;

    QByteArray array = ui->forwardSerialtextEdit->toPlainText().toLocal8Bit();
//    array.append(": Number ");
//    array.append(QString::number(number));
    array.append("\n");
    char const* buffer = array.data();

    strcpy((char*)cmd.data, buffer);
    cmd.size = strlen(buffer);
    cmd.number = number;
    ui->debugText->append("Sending number " + QString::number(number));
    mavlink_msg_serial_passthrough_encode(255, 0, &msg, &cmd);
    write_message(msg);
    number++;

}
