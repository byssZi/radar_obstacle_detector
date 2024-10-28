#include "radar_obstacle_detector/radar_pointcloud_subscriber.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "radar_pointcloud_subscriber");

    // 创建 RadarPointCloudSubscriber 类实例
    RadarPointCloudSubscriber radar_subscriber;

    // 进入 ROS 循环
    ros::spin();

    return 0;
}

