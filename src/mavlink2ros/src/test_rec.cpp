#include "serial_device.h" // 确保包含了SerialDevice类的头文件
#include "FishCom/mavlink.h"
#include "ros/ros.h"
#include <iostream>
#include <serial/serial.h>

uint8_t MyTask() {
    /* 创建并初始化串口设备 */
    // robomaster::SerialDevice serial_device("/dev/robomaster", 115200);
    // if (!serial_device.Init()) {
    //     std::cerr << "Failed to initialize serial device." << std::endl;
    //     return -1;
    // }
    serial::Serial serial_device("/dev/robomaster", 115200, serial::Timeout::simpleTimeout(1000));
    if (!serial_device.isOpen()) {
        std::cerr << "Failed to open serial port." << std::endl;
        return -1;
    }

    mavlink_chs_odom_info_t chassis_feedback;
    /*创建Mavlink状态变量*/
    mavlink_status_t status;
    /*选择一个Mavlink通道*/
    int chan = MAVLINK_COMM_0;
    /*创建一个Mavlink消息结构体*/
    mavlink_message_t *msg =
        (mavlink_message_t *)malloc(sizeof(mavlink_message_t));
    /*清理这部分内存*/
    memset(msg, 0, sizeof(mavlink_message_t));

    /*接收缓冲区*/
    /*大小随意 注意可能出现多条消息粘包接收情况*/
    uint8_t *rxbuf = (uint8_t *)malloc(512);
    /*接收到的数据长度*/
    uint16_t RxLen = 0;
    /*接收与解析消息*/
    while (ros::ok()) {
        /*接收数据*/
        // serial_device.Read(rxbuf, RxLen);
        if (serial_device.available()) {

            RxLen = serial_device.read(rxbuf, sizeof(rxbuf));
            printf("size of rxbuf: %d\n",sizeof(rxbuf));
            /*收到新数据*/
            for (int i = 0; i < RxLen; i++) {
                    /*解包*/
                    printf("解第 %d 个包\n", i);
                    printf("Byte %d: 0x%02X\n", i, rxbuf[i]);
                    /*MavlinkV2出现错误包后，再次接收二个正常包后恢复正常解析，但第一个正常包将丢失，第二个可被正确解析*/
                    if (mavlink_parse_char(chan, rxbuf[i], msg, &status)) {
                        printf("第 %d 个包，解包成功\n",i);
                        /*解析包成功 处理数据*/
                        switch (msg->msgid) {
                            case MAVLINK_MSG_ID_CHS_MOTOR_INFO: {
                                mavlink_msg_chs_odom_info_decode(msg, &chassis_feedback);
                                /*Do someting with new message*/
                                /*...*/
                                printf("22222222222222222222222222222222222222222222222222222");
                                printf("vx:%f,vy:%f,vz:%f",chassis_feedback.vx,chassis_feedback.vy,chassis_feedback.vw);
                                break;  
                            }
                            /*其他数据处理*/
                            /*...*/
                        }
                    }
            }
            /* Sleep thread for 1ms if no data received */
            sleep(1);
        }
    }
    /*释放资源*/
    free((void *)msg);
    free((void *)rxbuf);
    return 1;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "test_rec");
    MyTask();
    // ros::spin();
    
    return 0;
}