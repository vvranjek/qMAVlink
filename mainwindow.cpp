#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QTimer"
#include "qextserialenumerator.h"
#include "iostream"
#include "fstream"
#include <sys/time.h>
#include <QThread>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    on_PortRefresh_released();
    
    tmrTimer = new QTimer(this);
    connect(tmrTimer,  SIGNAL(timeout()),this,SLOT(main_slot()));
    tmrTimer->start(30);

    QStringList _baud_rates;
    _baud_rates << "4800" << "9600" << "19200" << "38400" << "57600" << "115200" << "230400";
    ui->baudBox->addItems(_baud_rates);
    ui->baudBox->setCurrentIndex(ui->baudBox->findText("115200"));

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
    while (!port->atEnd()) //Temp solution with 'stop'
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
                            //printf("MAVLINK_MSG_ID_HEARTBEAT\n");
                            mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
                            //current_messages.time_stamps.heartbeat = get_time_usec();
                            //this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
                            ui->mavlinkIn->append("SYS_ID: " + QString::number(message.sysid));
                            ui->mavlinkIn->append("COM_ID: " + QString::number(message.compid));
                            break;
                        }
            
            //            case MAVLINK_MSG_ID_SYS_STATUS:
            //            {
            //                printf("MAVLINK_MSG_ID_SYS_STATUS\n");
            //                mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
            //                current_messages.time_stamps.sys_status = get_time_usec();
            //                this_timestamps.sys_status = current_messages.time_stamps.sys_status;
            //                break;
            //            }
            
            //            case MAVLINK_MSG_ID_BATTERY_STATUS:
            //            {
            //                printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
            //                mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
            //                current_messages.time_stamps.battery_status = get_time_usec();
            //                this_timestamps.battery_status = current_messages.time_stamps.battery_status;
            //                break;
            //            }
            
            //            case MAVLINK_MSG_ID_RADIO_STATUS:
            //            {
            //                printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
            //                mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
            //                current_messages.time_stamps.radio_status = get_time_usec();
            //                this_timestamps.radio_status = current_messages.time_stamps.radio_status;
            //                break;
            //            }
            
            //            case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            //            {
            //                printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
            //                mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
            //                current_messages.time_stamps.local_position_ned = get_time_usec();
            //                this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
            //                //printf("Roll: %f\n", current_messages.attitude.roll);
            //                break;
            //            }
            
            //            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            //            {
            //                printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
            //                mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
            //                current_messages.time_stamps.global_position_int = get_time_usec();
            //                this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
            //                break;
            //            }
            
            //            case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
            //            {
            //                printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
            //                mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
            //                current_messages.time_stamps.position_target_local_ned = get_time_usec();
            //                this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
            //                break;
            //            }
            
            //            case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
            //            {
            //                printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
            //                mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
            //                current_messages.time_stamps.position_target_global_int = get_time_usec();
            //                this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
            //                break;
            //            }
            
            //            case MAVLINK_MSG_ID_HIGHRES_IMU:
            //            {
            //                printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
            //                mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
            //                current_messages.time_stamps.highres_imu = get_time_usec();
            //                this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
            //                break;
            //            }
            
            //            case MAVLINK_MSG_ID_ATTITUDE:
            //            {
            //                //printf("MAVLINK_MSG_ID_ATTITUDE\n");
            //                mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
            //                current_messages.time_stamps.attitude = get_time_usec();
            //                this_timestamps.attitude = current_messages.time_stamps.attitude;
            //                //printf("par1: %f\n", current_messages.attitude.roll);
            //                //std::cout << "MAVLINK_MSG_ID_ATTITUDE" << "\n";
            
            
            //                break;
            //            }
            
            //            case MAVLINK_MSG_ID_RC_CHANNELS:
            //            {
            //                printf("MAVLINK_MSG_ID_CHANNELS\n");
            //                mavlink_msg_rc_channels_decode(&message, &(current_messages.rc_channels));
            //                current_messages.time_stamps.rc_channels = get_time_usec();
            //                this_timestamps.rc_channels = current_messages.time_stamps.rc_channels;
            //                //printf("chan1: %f\n", current_messages.rc_channels.chan1_raw);
            //                break;
            //            }
            
            //            case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
            //            {
            //                printf("MAVLINK_MSG_ID_CHANNELS\n");
            //                mavlink_msg_rc_channels_override_decode(&message, &(current_messages.rc_channels_override));
            //                current_messages.time_stamps.rc_channels_override = get_time_usec();
            //                this_timestamps.rc_channels_override = current_messages.time_stamps.rc_channels_override;
            //                //printf("chan1_override: %f\n", current_messages.rc_channels_override.chan1_raw);
            //                break;
            //            }
            //            case MAVLINK_MSG_ID_ATTITUDE_TARGET:
            //            {
            //                printf("MAVLINK_MSG_ID_ATTITUDE_TARGET\n");
            //                mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
            //                current_messages.time_stamps.attitude = get_time_usec();
            //                this_timestamps.attitude = current_messages.time_stamps.attitude;
            //                break;
            //            }
            
            case MAVLINK_MSG_ID_VFR_HUD:
            {
                ui->mavlinkIn->append("MAVLINK_MSG_ID_VFR_HUD");


                printf("MAVLINK_MSG_ID_VFR_HUD\n");
                break;
            }
            case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
            {
                printf("MAVLINK_MSG_ID_SERVO_OUTPUT_RAW\n");
                break;
            }
            case MAVLINK_MSG_ID_MISSION_CURRENT:
            {
                ui->mavlinkIn->append("MAVLINK_MSG_ID_MISSION_CURRENT");
                printf("MAVLINK_MSG_ID_MISSION_CURRENT\n");
                break;
            }
            case MAVLINK_MSG_ID_GPS_RAW_INT:
            {
                printf("MAVLINK_MSG_ID_GPS_RAW_INT\n");
                break;
            }
                //            case MAVLINK_MSG_ID_:
                //            {
                //                printf("MAVLINK_MSG_ID_\n");
                //                break;
                //            }
                //            case MAVLINK_MSG_ID_:
                //            {
                //                printf("MAVLINK_MSG_ID_\n");
                //                break;
                //            }
                //            case MAVLINK_MSG_ID_:
                //            {
                //                printf("MAVLINK_MSG_ID_\n");
                //                break;
                //            }
                //            case MAVLINK_MSG_ID_:
                //            {
                //                printf("MAVLINK_MSG_ID_\n");
                //                break;
                //            }
                //            case MAVLINK_MSG_ID_:
                //            {
                //                printf("MAVLINK_MSG_ID_\n");
                //                break;
                //            }
                //            case MAVLINK_MSG_ID_:
                //            {
                //                printf("MAVLINK_MSG_ID_\n");
                //                break;
                //            }
                
            default:
            {
                ui->mavlinkIn->append("Warning!!!! Did not handle message " + QString::number(message.msgid));
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
      cmd.param1 = armed ? 1.0f : 0.0f;
      cmd.param2 = 0.0f;
      cmd.param3 = 0.0f;
      cmd.param4 = 0.0f;
      cmd.param5 = 0.0f;
      cmd.param6 = 0.0f;
      cmd.param7 = 0.0f;
      cmd.target_system = 1;
      cmd.target_component = 0;

      mavlink_msg_command_long_encode(1, 1, &msg, &cmd);

      //write_message(msg);
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
   // port->setBaudRate(ui->baudBox->currentText().toInt());
    port->setPortName(ui->portBox->currentText());

    if (!port->open(QIODevice::ReadWrite)){
            printf("Connected to %s", ui->portBox->currentText().toStdString());
            return 0;
    }
    else {
        return 1;
    }
}




void MainWindow::on_connectButton_released()
{
    if (!ui->connectButton->isChecked()){
        port->close();
    }
    else {
        connect_serial();
    }
}
