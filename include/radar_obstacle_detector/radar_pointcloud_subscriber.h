#ifndef RADAR_POINTCLOUD_SUBSCRIBER_H
#define RADAR_POINTCLOUD_SUBSCRIBER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/io/pcd_io.h>
#include "point.h"
#include "dbscan.h"
#include "obstacletracker.h"
#include "cluster.h"
#include "kdtree.h"
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include "radar_obstacle_detector/ClusterList.h"
#include "radar_obstacle_detector/ObjectList.h"
class RadarPointCloudSubscriber
{
public:
    RadarPointCloudSubscriber();
    ~RadarPointCloudSubscriber();

    // 回调函数，处理接收到的点云数据
    void radarPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void radarClusterCallback(const radar_obstacle_detector::ClusterList::ConstPtr& cluster_msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber pointcloud_sub_;  // 点云订阅者
    ros::Subscriber cluster_sub_; //自定义消息订阅
    ros::Publisher object_pub_; //聚类结果发布
    ros::Publisher cluster_view_; //可视化话题发布
    ros::Publisher cluster_pub_; //聚类点云发布

    // DBSCAN 相关变量
    std::vector<dbscan::Point> points;
    int min_points, max_frames;
    double epsilon;
    std::deque<std::vector<dbscan::Point>> point_cloud_frames;
    std::vector<dbscan::Cluster> clusters;
    std::vector<dbscan::Cluster::BoundingBox> prev_boxes_, curr_boxes_;
    dbscan::ObstacleTracker tracker;
};

#endif  // RADAR_POINTCLOUD_SUBSCRIBER_H

