#include "radar_obstacle_detector/cluster.h"

#include <algorithm>

namespace dbscan {

Cluster::Cluster(std::vector<Point> const& new_points) {
    id_ = current_id;
    points = new_points;

    double total_magnitude = 0;
    double total_azimuth = 0;
    double total_elevation = 0;
    double total_rcs = 0;
    for (auto point:new_points) {
        total_magnitude += point.magnitude();
        total_azimuth += point.azimuth();
        total_elevation += point.elevation();
        total_rcs += point.rcs();
    }
    int n = new_points.size();
    magnitude_ = total_magnitude/n;
    azimuth_ = total_azimuth/n;
    elevation_ = total_elevation/n;
    rcs_ = total_rcs/n;

    current_id = (current_id < SIZE_MAX) ? ++current_id : 0;
}

Cluster::BoundingBox Cluster::ConstructBoundingBox() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto point : points) {
        pcl::PointXYZ cloud_point;
        cloud_point.x = point.x();
        cloud_point.y = point.y();
        cloud_point.z = point.z();
        cluster->points.push_back(cloud_point);
    }

    // 如果点云少于3个点，记录警告并结束函数
    if (cluster->size() < 3) {
        return Cluster::BoundingBox();  // 返回默认或无效的BoundingBox
    }

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    const float box_height = max_pt.z - min_pt.z;
    const float box_z = (max_pt.z + min_pt.z) / 2;

    // Compute the cluster centroid
    Eigen::Vector4f pca_centroid;
    pcl::compute3DCentroid(*cluster, pca_centroid);

    // Squash the cluster to x-y plane with z = centroid z
    for (size_t i = 0; i < cluster->size(); ++i) {
        cluster->points[i].z = pca_centroid(2);
    }

    // Compute principal directions & Transform the original cloud to PCA coordinates
    pcl::PointCloud<pcl::PointXYZ>::Ptr pca_projected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cluster);
    pca.project(*cluster, *pca_projected_cloud);

    const auto eigen_vectors = pca.getEigenVectors();

    // Get the minimum and maximum points of the transformed cloud
    pcl::getMinMax3D(*pca_projected_cloud, min_pt, max_pt);
    const Eigen::Vector3f meanDiagonal = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf quaternion(eigen_vectors);
    const Eigen::Vector3f position = eigen_vectors * meanDiagonal + pca_centroid.head<3>();
    const Eigen::Vector3f dimension((max_pt.x - min_pt.x), (max_pt.y - min_pt.y), box_height);
    const Eigen::Vector3f velocity(magnitude_, azimuth_, elevation_);

    Cluster::BoundingBox new_box(id_, position, dimension, velocity, quaternion, rcs_);
    return new_box;
}


std::tuple<double, double, double> Cluster::Centroid() {
    double x, y, z = 0;
    for (auto point:points) {
        x += point.x();
        y += point.y();
        z += point.z();
    }
    int n = points.size();
    return {x/n, y/n, z/n};
}

double Cluster::IntraClusterDistance() {
    double total_distances = 0;
    int total_comparisons = 0;
    for (int i = 0; i < points.size(); i++) {
        for (int j = i + 1; j < points.size(); j++) {
            total_distances += points[i].EuclideanDistance(points[j]);
            total_comparisons++;
        }
    }
    double avg = total_distances/total_comparisons;
    return avg;
}

int Cluster::id() const {
    return id_;
}

double Cluster::magnitude() const {
    return magnitude_;
}

double Cluster::azimuth() const {
    return azimuth_;
}

double Cluster::elevation() const {
    return elevation_;
}

double Cluster::rcs() const {
    return rcs_;
}

int Cluster::current_id = 0;

}
