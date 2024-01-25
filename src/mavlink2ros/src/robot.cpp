#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <thread>
#include <iostream>
#include "FishCom/mavlink.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>

class RobotController {
public:
    RobotController(const std::string& port, int baud_rate) : 
        serial_device(port, baud_rate, serial::Timeout::simpleTimeout(1000)),
        chassis_odom_pub_(nh.advertise<nav_msgs::Odometry>("odom", 50))
    {
        if (!serial_device.isOpen()) {
            std::cerr << "Failed to open serial port." << std::endl;
        }

        sub = nh.subscribe("cmd_vel", 1000, &RobotController::cmdVelCallback, this);
        receive_thread = std::thread(&RobotController::receiveData, this);

        // Regularly Msg
        manage_timer = nh.createTimer(ros::Duration(1.0), &RobotController::sendManageInfoRegularly, this);
        control_timer = nh.createTimer(ros::Duration(0.01), &RobotController::sendCtrlInfoRegularly, this);
        
        chassis_odom_pub_=nh.advertise<nav_msgs::Odometry>("odom",1);
        odom_.header.frame_id = "odom";
        odom_.child_frame_id = "base_link";
        odom_tf_.header.frame_id = "odom";
        odom_tf_.child_frame_id = "base_link";
    }

    ~RobotController() {
        if (receive_thread.joinable()) {
            receive_thread.join();
        }
    }

    void sendManageInfoRegularly(const ros::TimerEvent&) {
        sendManageInfo(1, 1, 0);
    }

    void sendCtrlInfoRegularly(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(twist_mutex); // 确保线程安全
        sendCtrlInfo(current_twist.linear.x, current_twist.linear.y, current_twist.angular.z);
    }

    void sendManageInfo(bool enable_chassis,bool enable_servos,bool reset_quaternion){
        mavlink_message_t *msg = (mavlink_message_t *)malloc(sizeof(mavlink_message_t));
        memset(msg, 0, sizeof(mavlink_message_t));

        uint16_t Txlen = 0;
        uint8_t *txbuf = (uint8_t *)malloc(MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN + 12);

        mavlink_msg_chs_manage_info_pack(
            CHS_SYSTEM_ID::CHS_ID_ORANGE,
            CHS_SYSTEM_ID::CHS_ID_ORANGE, 
            msg, enable_chassis, enable_servos, reset_quaternion);
        
        Txlen = mavlink_msg_to_send_buffer(txbuf, msg);

        serial_device.write(txbuf, Txlen);
    }

    void sendCtrlInfo(float vx, float vy, float vz){
        mavlink_message_t *msg = (mavlink_message_t *)malloc(sizeof(mavlink_message_t));
        memset(msg, 0, sizeof(mavlink_message_t));

        uint16_t Txlen = 0;
        uint8_t *txbuf = (uint8_t *)malloc(MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN + 12);

        mavlink_msg_chs_ctrl_info_pack(
            CHS_SYSTEM_ID::CHS_ID_ORANGE,
            CHS_SYSTEM_ID::CHS_ID_ORANGE, 
            msg, 
            vx, vy, vz);
        
        Txlen = mavlink_msg_to_send_buffer(txbuf, msg);

        serial_device.write(txbuf, Txlen);
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        current_twist = *msg;
    }

    // void publishOdometry(const mavlink_chs_odom_info_t& odom_info) {
    //     nav_msgs::Odometry odom_msg;
    //     geometry_msgs::TransformStamped transform_stamped;

    //     odom_msg.header.stamp = ros::Time::now();
    //     odom_msg.header.frame_id = "odom";
    //     odom_msg.child_frame_id = "base_link";

    //     odom_msg.pose.pose.position.x = 0;
    //     odom_msg.pose.pose.position.y = 0;
    //     odom_msg.pose.pose.position.z = 0;

    //     tf2::Quaternion quat;
    //     quat.setValue(odom_info.quaternion[0], odom_info.quaternion[1], odom_info.quaternion[2], odom_info.quaternion[3]);
    //     quat.normalize();
    //     odom_msg.pose.pose.orientation = tf2::toMsg(quat);

    //     odom_msg.twist.twist.linear.x = odom_info.vx;
    //     odom_msg.twist.twist.linear.y = odom_info.vy;
    //     odom_msg.twist.twist.angular.z = odom_info.vw;

    //     odom_pub.publish(odom_msg);

    //     transform_stamped.header.stamp = odom_msg.header.stamp;
    //     transform_stamped.header.frame_id = "odom";
    //     transform_stamped.child_frame_id = "base_link";
    //     transform_stamped.transform.translation.x = odom_msg.pose.pose.position.x;
    //     transform_stamped.transform.translation.y = odom_msg.pose.pose.position.y;
    //     transform_stamped.transform.translation.z = odom_msg.pose.pose.position.z;
    //     transform_stamped.transform.rotation = odom_msg.pose.pose.orientation;

    //     tf_broadcaster.sendTransform(transform_stamped);
    // }
    void receiveData() {
        while (ros::ok()) {
            if (serial_device.available()) {
                mavlink_message_t *msg = (mavlink_message_t *)malloc(sizeof(mavlink_message_t));
                memset(msg, 0, sizeof(mavlink_message_t));

                mavlink_status_t status;
                uint8_t rxbuf[512];
                size_t RxLen = serial_device.read(rxbuf, sizeof(rxbuf));

                mavlink_chs_odom_info_t odom_info;
                mavlink_chs_ctrl_info_t ctrl_info;
                mavlink_chs_motor_info_t motor_info;
                mavlink_chs_servos_info_t servos_info;
                mavlink_chs_manage_info_t manage_info;
                mavlink_chs_remoter_info_t remoter_info;

                for (size_t i = 0; i < RxLen; ++i) {
                    // printf("Byte %ld: 0x%02X\n", i, rxbuf[i]);
                    if (mavlink_parse_char(MAVLINK_COMM_0, rxbuf[i], msg, &status)) {
                        printf("第 %ld 个包，解包成功\n",i);
                        printf("%d",msg->msgid);
                        switch (msg->msgid) {
                            case MAVLINK_MSG_ID_CHS_CTRL_INFO: {
                                mavlink_msg_chs_ctrl_info_decode(msg, &ctrl_info);
                                printf("received ctrl info! \n");
                                printf("vx: %f,vy: %f,vz: %f \n",ctrl_info.vx,ctrl_info.vy,ctrl_info.vw);
                                break;
                            }
                            case MAVLINK_MSG_ID_CHS_ODOM_INFO: {
                                mavlink_msg_chs_odom_info_decode(msg, &odom_info);
                                printf("received odom info! \n");
                                printf("vx: %f,vy: %f,vz: %f \n",odom_info.vx,odom_info.vy,odom_info.vw);
                                printf("quaternion: ");
                                for(int i = 0; i < 4; i++) printf("%f ", odom_info.quaternion[i]);
                                printf("\n");

                                // publishOdometry(odom_info);
                                current_time = ros::Time::now();
                                double dt = (current_time - last_time).toSec();

                                // 使用从 MAVLink 消息接收到的速度来更新位置和姿态
                                double delta_x = (odom_info.vx * cos(th) - odom_info.vy * sin(th)) * dt;
                                double delta_y = (odom_info.vx * sin(th) + odom_info.vy * cos(th)) * dt;
                                double delta_th = odom_info.vw * dt;

                                x += delta_x;
                                y += delta_y;
                                th += delta_th;

                                // 更新里程计消息
                                odom_.header.stamp = current_time;
                                odom_.twist.twist.linear.x = odom_info.vx;
                                odom_.twist.twist.linear.y = odom_info.vy;
                                odom_.twist.twist.angular.z = odom_info.vw;

                                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
                                odom_.pose.pose.position.x = x;
                                odom_.pose.pose.position.y = y;
                                odom_.pose.pose.position.z = 0.0;
                                odom_.pose.pose.orientation = odom_quat;

                                // 发布里程计消息
                                chassis_odom_pub_.publish(odom_);

                                // 发布 TF 变换
                                odom_tf_.header.stamp = current_time;
                                odom_tf_.transform.translation.x = x;
                                odom_tf_.transform.translation.y = y;
                                odom_tf_.transform.translation.z = 0.0;
                                odom_tf_.transform.rotation = odom_quat;
                                tf_broadcaster_.sendTransform(odom_tf_);

                                last_time = current_time;
                                break;
                            }
                            case MAVLINK_MSG_ID_CHS_MOTOR_INFO: {
                                mavlink_msg_chs_motor_info_decode(msg, &motor_info);
                                printf("motor_info: \n");
                                for(int i = 0; i < 4; i++) printf("%d ", motor_info.motor[i]);
                                printf("\n");
                                break;
                            }
                            case MAVLINK_MSG_ID_CHS_SERVOS_INFO: {
                                mavlink_msg_chs_servos_info_decode(msg, &servos_info);
                                printf("received servos_info: \n");
                                for(int i = 0; i < 7; i++) printf("%d ", servos_info.servos[i]);
                                printf("\n");
                                break;
                            }
                            case MAVLINK_MSG_ID_CHS_MANAGE_INFO: {
                                mavlink_msg_chs_manage_info_decode(msg, &manage_info);
                                printf("received manage_info: \n");
                                printf("enable_chassis: %d, enable_servos: %d, reset_quaternion: %d\n",
                                        manage_info.enable_chassis, manage_info.enable_servos, manage_info.reset_quaternion);
                                break;
                            }
                            case MAVLINK_MSG_ID_CHS_REMOTER_INFO: {
                                mavlink_msg_chs_remoter_info_decode(msg, &remoter_info);
                                printf("received remoter_info: \n");
                                printf("switch_message: %d, channel_0: %d, channel_1: %d, channel_2: %d, channel_3: %d, wheel: %d\n",
                                        remoter_info.switch_messgae, remoter_info.channel_0, remoter_info.channel_1, 
                                        remoter_info.channel_2, remoter_info.channel_3, remoter_info.wheel);
                                break;
                            }
                        }
                    }
                }
            }
            usleep(1000);
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    serial::Serial serial_device;
    std::thread receive_thread;
    // ros::Publisher odom_pub;
    // tf::TransformBroadcaster tf_broadcaster;
    geometry_msgs::Twist current_twist; 

    ros::Timer manage_timer;
    ros::Timer control_timer;
    std::mutex twist_mutex;

    tf::TransformBroadcaster tf_broadcaster_;//! ros chassis odometry tf broadcaster
    ros::Publisher chassis_odom_pub_;//! ros odometry message publisher
    geometry_msgs::TransformStamped odom_tf_;//! ros chassis odometry tf
    nav_msgs::Odometry odom_;//! ros odometry message
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    ros::Time current_time = ros::Time::now(), last_time = ros::Time::now();
}; 

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_controller");
    RobotController controller("/dev/robomaster", 115200);
    ros::spin();
    return 0;
}
