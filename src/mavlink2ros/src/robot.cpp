#include"FishCom/mavlink.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <thread>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "serial_device.h"

class RobotController {
public:
    RobotController() : serial_device("/dev/robomaster", 115200) {
        if (!serial_device.Init()) {
            std::cerr << "Failed to initialize serial device." << std::endl;
            exit(-1);
        }
        sub = nh.subscribe("cmd_vel", 1000, &RobotController::cmdVelCallback, this);
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& twistMsg) {
        ros::Time current_time = ros::Time::now();

        mavlink_message_t *msg =
            (mavlink_message_t *)malloc(sizeof(mavlink_message_t));
        memset(msg, 0, sizeof(mavlink_message_t));
        uint16_t Txlen = 0;

        uint8_t *txbuf = (uint8_t *)malloc(MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN + 12);

        mavlink_msg_chs_ctrl_info_pack(
        CHS_SYSTEM_ID::CHS_ID_ORANGE,
        CHS_SYSTEM_ID::CHS_ID_ORANGE, 
        msg, 
        twistMsg->linear.x,twistMsg->linear.y, twistMsg->linear.z);

        Txlen = mavlink_msg_to_send_buffer(txbuf, msg);

        serial_device.Write(txbuf, Txlen);
        free((void *)txbuf);
        free((void *)msg);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    robomaster::SerialDevice serial_device;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "simple_robot");
    RobotController controller;
    ros::spin();
    return 0;
}