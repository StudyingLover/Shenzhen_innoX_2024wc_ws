#include <ros/ros.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include"geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include <bits/stdc++.h>

int main(int argc, char  *argv[]){
    ros::init(argc,argv,"static_tf");
    ros::NodeHandle nh;
    tf2_ros::StaticTransformBroadcaster pub ;


    int id = 101;
    double x = 0.3, y = 0.3;
    for(int i = 1; i <= 9; i ++){
        for(int j = 1; j <= 13; j ++){
            geometry_msgs::TransformStamped ts;
            ts.header.seq = 100;
            ts.header.stamp = ros::Time::now();
            ts.header.frame_id = "map";
            ts.child_frame_id = "tag_" + std::to_string(id);;
            ts.transform.translation.x = y;
            ts.transform.translation.y = x;
            ts.transform.translation.z = 0;

            tf2::Quaternion qtn;
            qtn.setRPY(0,0,0);
            ts.transform.rotation.x = qtn.getX();
            ts.transform.rotation.y = qtn.getY();
            ts.transform.rotation.z = qtn.getZ();
            ts.transform.rotation.w = qtn.getW();
        
            pub.sendTransform(ts);

            id ++;
            y += 0.6;
        }
        x += 0.6;
        y = 0.3;
        //test for git
    }

    ros::spin();
    return 0;
}
