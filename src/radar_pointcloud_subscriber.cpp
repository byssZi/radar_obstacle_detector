#include "radar_obstacle_detector/radar_pointcloud_subscriber.h"

// 构造函数，初始化订阅者和 DBSCAN 参数
RadarPointCloudSubscriber::RadarPointCloudSubscriber()
    : min_points(6), epsilon(1) // 初始化 DBSCAN 参数
{
    ros::NodeHandle private_nh("~");
    // 订阅毫米波雷达的点云话题
    std::string radar_points_topic;
    std::string radar_cluster_topic;
    std::string visualize_radar_topic;
    std::string dbscan_bbox_topic;
    std::string cluster_output_topic;
    ROS_ASSERT(private_nh.getParam("radar_points_topic", radar_points_topic));
    ROS_ASSERT(private_nh.getParam("radar_cluster_topic", radar_cluster_topic));
    ROS_ASSERT(private_nh.getParam("visualize_radar_topic", visualize_radar_topic));
    ROS_ASSERT(private_nh.getParam("dbscan_bbox_topic", dbscan_bbox_topic));
    ROS_ASSERT(private_nh.getParam("cluster_output_topic", cluster_output_topic));
    pointcloud_sub_ = nh.subscribe(radar_points_topic, 1, &RadarPointCloudSubscriber::radarPointCloudCallback, this);
    cluster_sub_ = nh.subscribe(radar_cluster_topic, 1, &RadarPointCloudSubscriber::radarClusterCallback, this);
    cluster_view_ = nh.advertise<visualization_msgs::MarkerArray>(visualize_radar_topic, 1);
    object_pub_ = nh.advertise<radar_obstacle_detector::ObjectList>(dbscan_bbox_topic, 1);
    cluster_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cluster_output_topic, 1);

}

RadarPointCloudSubscriber::~RadarPointCloudSubscriber() {

}

// 回调函数，处理点云数据
void RadarPointCloudSubscriber::radarPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // 创建迭代器来解析每个字段
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_range(*cloud_msg, "Range");
    sensor_msgs::PointCloud2ConstIterator<float> iter_velocity(*cloud_msg, "Velocity");
    sensor_msgs::PointCloud2ConstIterator<float> iter_azimuth(*cloud_msg, "AzimuthAngle");
    sensor_msgs::PointCloud2ConstIterator<float> iter_elevation(*cloud_msg, "ElevationAngle");

    // 遍历点云中的每一个点
    points.clear();
    for (size_t i = 0; i < cloud_msg->width * cloud_msg->height; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_range, ++iter_velocity, ++iter_azimuth, ++iter_elevation) {
        // 提取每个字段的数据
        float x = *iter_x;
        float y = *iter_y;
        float z = *iter_z;
        float range = *iter_range;
        float velocity = *iter_velocity;
        float azimuth = *iter_azimuth;
        float elevation = *iter_elevation;
        points.push_back(dbscan::Point(x, y, z, velocity, azimuth, elevation, range));  // 转换为 dbscan::Point 类型并存入 points 向量

    }
        // 使用 DBSCAN 算法进行聚类
    clusters = kdtree::cluster_points(points, epsilon, min_points);

    visualization_msgs::MarkerArray marker_bboxs;
    radar_obstacle_detector::ObjectList dbscan_bboxs;
    dbscan_bboxs.header = cloud_msg->header;
        // 遍历每个聚类，计算边界框
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_pub(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < clusters.size(); ++i) {
        dbscan::Cluster::BoundingBox box = clusters[i].ConstructBoundingBox();
        curr_boxes_.emplace_back(box);
        for(auto point : clusters[i].points){
            pcl::PointXYZ cluster_point;
            cluster_point.x = point.x();
            cluster_point.y = point.y();
            cluster_point.z = point.z();
            cluster_pub->points.push_back(cluster_point);
        }
    }
    sensor_msgs::PointCloud2::Ptr obstacle_cloud(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*(cluster_pub), *obstacle_cloud);
    obstacle_cloud->header = cloud_msg->header;
    cluster_pub_.publish(obstacle_cloud);

    tracker.obstacleTracking(prev_boxes_, curr_boxes_, 1.0, 1.0);
    for (auto box : curr_boxes_) {
            // Check if the box dimensions and center are valid before proceeding
        radar_obstacle_detector::Object object_bbox;
        object_bbox.id = box.id;
        object_bbox.position.pose.position.x = box.position[0];
        object_bbox.position.pose.position.y = box.position[1];
        object_bbox.position.pose.position.z = box.position[2];
        object_bbox.position.pose.orientation.w = box.quaternion.w();
        object_bbox.position.pose.orientation.x = box.quaternion.x();
        object_bbox.position.pose.orientation.y = box.quaternion.y();
        object_bbox.position.pose.orientation.z = box.quaternion.z();
        object_bbox.length = box.dimension[0];
        object_bbox.width = box.dimension[1];
        object_bbox.height = box.dimension[2];
        Eigen::Vector3f obstacle_eulerAngle = box.quaternion.matrix().eulerAngles(2,1,0);
        object_bbox.orientation_angle = obstacle_eulerAngle[0];
        double magnitude, azimuth, elevation;
        magnitude = box.velocity[0];
        azimuth = box.velocity[1];
        elevation = box.velocity[2];
        object_bbox.relative_velocity.twist.linear.x = magnitude * std::cos(elevation) * std::cos(azimuth);
        object_bbox.relative_velocity.twist.linear.y = magnitude * std::cos(elevation) * std::sin(azimuth);
        object_bbox.relative_velocity.twist.linear.z = magnitude * std::sin(elevation);
        dbscan_bboxs.objects.push_back(object_bbox);

        visualization_msgs::Marker marker_bbox;
        marker_bbox.header = cloud_msg->header;
        marker_bbox.pose.position.x = box.position[0];
        marker_bbox.pose.position.y = box.position[1];
        marker_bbox.pose.position.z = box.position[2];
        marker_bbox.pose.orientation.w = box.quaternion.w();
        marker_bbox.pose.orientation.x = box.quaternion.x();
        marker_bbox.pose.orientation.y = box.quaternion.y();
        marker_bbox.pose.orientation.z = box.quaternion.z();
        marker_bbox.type = visualization_msgs::Marker::LINE_STRIP;
        marker_bbox.id = box.id;
        geometry_msgs::Point pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8;
        pos1.x = box.dimension[0] / 2;
        pos1.y = box.dimension[1] / 2;
        pos1.z = box.dimension[2] / 2;

        pos2.x = box.dimension[0] / 2;
        pos2.y = box.dimension[1] / 2;
        pos2.z = -box.dimension[2] / 2;

        pos3.x = box.dimension[0] / 2;
        pos3.y = -box.dimension[1] / 2;
        pos3.z = -box.dimension[2] / 2;

        pos4.x = box.dimension[0] / 2;
        pos4.y = -box.dimension[1] / 2;
        pos4.z = box.dimension[2] / 2;

        pos5.x = -box.dimension[0] / 2;
        pos5.y = -box.dimension[1] / 2;
        pos5.z = box.dimension[2] / 2;

        pos6.x = -box.dimension[0] / 2;
        pos6.y = -box.dimension[1] / 2;
        pos6.z = -box.dimension[2] / 2;

        pos7.x = -box.dimension[0] / 2;
        pos7.y = box.dimension[1] / 2;
        pos7.z = -box.dimension[2] / 2;

        pos8.x = -box.dimension[0] / 2;
        pos8.y = box.dimension[1] / 2;
        pos8.z = box.dimension[2] / 2;
        marker_bbox.points.push_back(pos1);
        marker_bbox.points.push_back(pos2);
        marker_bbox.points.push_back(pos3);
        marker_bbox.points.push_back(pos4);
        marker_bbox.points.push_back(pos5);
        marker_bbox.points.push_back(pos6);
        marker_bbox.points.push_back(pos7);
        marker_bbox.points.push_back(pos8);
        marker_bbox.points.push_back(pos1);
        marker_bbox.points.push_back(pos4);
        marker_bbox.points.push_back(pos3);
        marker_bbox.points.push_back(pos6);
        marker_bbox.points.push_back(pos5);
        marker_bbox.points.push_back(pos8);
        marker_bbox.points.push_back(pos7);
        marker_bbox.points.push_back(pos2);
        marker_bbox.color.r = 0.0;
        marker_bbox.color.g = 1.0;
        marker_bbox.color.b = 0.0;
        marker_bbox.scale.x = 0.1;
        marker_bbox.color.a = 1.0;
        marker_bbox.lifetime.fromSec(0.1);
        marker_bboxs.markers.push_back(marker_bbox);
    }
    object_pub_.publish(dbscan_bboxs);
    cluster_view_.publish(marker_bboxs);
    prev_boxes_.swap(curr_boxes_);
    curr_boxes_.clear();
}

void RadarPointCloudSubscriber::radarClusterCallback(const radar_obstacle_detector::ClusterList::ConstPtr& cluster_msg)
{
    points.clear();
    for (const auto& p : cluster_msg->clusters) {
        double x, y, z;
        x = p.position.pose.position.x;
        y = p.position.pose.position.y;
        z = p.position.pose.position.z;
        double magnitude, azimuth, elevation, vx, vy, vz;
        vx = p.relative_velocity.twist.linear.x;
        vy = p.relative_velocity.twist.linear.y;
        vz = 0;
        magnitude = std::sqrt(vx * vx + vy * vy + vz * vz);
        azimuth = std::atan2(vy, vx);
        elevation = std::asin(vz / magnitude);
        double rcs = p.rcs;
        points.push_back(dbscan::Point(x, y, z, magnitude, azimuth, elevation, rcs));  // 转换为 dbscan::Point 类型并存入 points 向量
    }
            // 使用 DBSCAN 算法进行聚类
    clusters = kdtree::cluster_points(points, epsilon, min_points);

    visualization_msgs::MarkerArray marker_bboxs;
    radar_obstacle_detector::ObjectList dbscan_bboxs;
    dbscan_bboxs.header = cluster_msg->header;
        // 遍历每个聚类，计算边界框
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_pub(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < clusters.size(); ++i) {
        dbscan::Cluster::BoundingBox box = clusters[i].ConstructBoundingBox();
        curr_boxes_.emplace_back(box);
        for(auto point : clusters[i].points){
            pcl::PointXYZ cluster_point;
            cluster_point.x = point.x();
            cluster_point.y = point.y();
            cluster_point.z = point.z();
            cluster_pub->points.push_back(cluster_point);
        }
    }
    sensor_msgs::PointCloud2::Ptr obstacle_cloud(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*(cluster_pub), *obstacle_cloud);
    obstacle_cloud->header = cluster_msg->header;
    cluster_pub_.publish(obstacle_cloud);

    tracker.obstacleTracking(prev_boxes_, curr_boxes_, 1.0, 1.0);
    for (auto box : curr_boxes_) {
        radar_obstacle_detector::Object object_bbox;
        object_bbox.id = box.id;
        object_bbox.position.pose.position.x = box.position[0];
        object_bbox.position.pose.position.y = box.position[1];
        object_bbox.position.pose.position.z = box.position[2];
        object_bbox.position.pose.orientation.w = box.quaternion.w();
        object_bbox.position.pose.orientation.x = box.quaternion.x();
        object_bbox.position.pose.orientation.y = box.quaternion.y();
        object_bbox.position.pose.orientation.z = box.quaternion.z();
        object_bbox.length = box.dimension[0];
        object_bbox.width = box.dimension[1];
        object_bbox.height = box.dimension[2];
        Eigen::Vector3f obstacle_eulerAngle = box.quaternion.matrix().eulerAngles(2,1,0);
        object_bbox.orientation_angle = obstacle_eulerAngle[0];
        double magnitude, azimuth, elevation;
        magnitude = box.velocity[0];
        azimuth = box.velocity[1];
        elevation = box.velocity[2];
        object_bbox.relative_velocity.twist.linear.x = magnitude * std::cos(elevation) * std::cos(azimuth);
        object_bbox.relative_velocity.twist.linear.y = magnitude * std::cos(elevation) * std::sin(azimuth);
        object_bbox.relative_velocity.twist.linear.z = magnitude * std::sin(elevation);
        dbscan_bboxs.objects.push_back(object_bbox);

        visualization_msgs::Marker marker_bbox;
        marker_bbox.header = cluster_msg->header;
        marker_bbox.pose.position.x = box.position[0];
        marker_bbox.pose.position.y = box.position[1];
        marker_bbox.pose.position.z = box.position[2];
        marker_bbox.pose.orientation.w = box.quaternion.w();
        marker_bbox.pose.orientation.x = box.quaternion.x();
        marker_bbox.pose.orientation.y = box.quaternion.y();
        marker_bbox.pose.orientation.z = box.quaternion.z();
        marker_bbox.type = visualization_msgs::Marker::LINE_STRIP;
        marker_bbox.id = box.id;
        geometry_msgs::Point pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8;
        pos1.x = box.dimension[0] / 2;
        pos1.y = box.dimension[1] / 2;
        pos1.z = box.dimension[2] / 2;

        pos2.x = box.dimension[0] / 2;
        pos2.y = box.dimension[1] / 2;
        pos2.z = -box.dimension[2] / 2;

        pos3.x = box.dimension[0] / 2;
        pos3.y = -box.dimension[1] / 2;
        pos3.z = -box.dimension[2] / 2;

        pos4.x = box.dimension[0] / 2;
        pos4.y = -box.dimension[1] / 2;
        pos4.z = box.dimension[2] / 2;

        pos5.x = -box.dimension[0] / 2;
        pos5.y = -box.dimension[1] / 2;
        pos5.z = box.dimension[2] / 2;

        pos6.x = -box.dimension[0] / 2;
        pos6.y = -box.dimension[1] / 2;
        pos6.z = -box.dimension[2] / 2;

        pos7.x = -box.dimension[0] / 2;
        pos7.y = box.dimension[1] / 2;
        pos7.z = -box.dimension[2] / 2;

        pos8.x = -box.dimension[0] / 2;
        pos8.y = box.dimension[1] / 2;
        pos8.z = box.dimension[2] / 2;
        marker_bbox.points.push_back(pos1);
        marker_bbox.points.push_back(pos2);
        marker_bbox.points.push_back(pos3);
        marker_bbox.points.push_back(pos4);
        marker_bbox.points.push_back(pos5);
        marker_bbox.points.push_back(pos6);
        marker_bbox.points.push_back(pos7);
        marker_bbox.points.push_back(pos8);
        marker_bbox.points.push_back(pos1);
        marker_bbox.points.push_back(pos4);
        marker_bbox.points.push_back(pos3);
        marker_bbox.points.push_back(pos6);
        marker_bbox.points.push_back(pos5);
        marker_bbox.points.push_back(pos8);
        marker_bbox.points.push_back(pos7);
        marker_bbox.points.push_back(pos2);
        marker_bbox.color.r = 0.0;
        marker_bbox.color.g = 1.0;
        marker_bbox.color.b = 0.0;
        marker_bbox.scale.x = 0.1;
        marker_bbox.color.a = 1.0;
        marker_bbox.lifetime.fromSec(0.1);
        marker_bboxs.markers.push_back(marker_bbox);
    }
    object_pub_.publish(dbscan_bboxs);
    cluster_view_.publish(marker_bboxs);
    prev_boxes_.swap(curr_boxes_);
    curr_boxes_.clear();
}


