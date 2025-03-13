#pragma once
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include "point.h"
#include <Eigen/Geometry> 
#include <iostream>
#include <tuple>
#include <vector>

namespace dbscan {

class Cluster {
public:
    struct BoundingBox {
        int id;
        Eigen::Vector3f position;
        Eigen::Vector3f dimension;
        Eigen::Vector3f velocity;
        Eigen::Quaternionf quaternion;
        double rcs;
            // 带四元数的完整构造函数
        BoundingBox() {};
        BoundingBox(int id, Eigen::Vector3f position, Eigen::Vector3f dimension, Eigen::Vector3f velocity, Eigen::Quaternionf quaternion, double rcs)
            : id(id), position(position), dimension(dimension), velocity(velocity), quaternion(quaternion), rcs(rcs)
        {} 
    };

    Cluster(std::vector<Point> const& new_points);

    BoundingBox ConstructBoundingBox();

    std::tuple<double, double, double> Centroid();

    double IntraClusterDistance();

    int id() const;

    double magnitude() const;

    double azimuth() const;

    double elevation() const;

    double rcs() const;

    std::vector<Point> points;

    static int current_id;

private:
    int id_;
    double magnitude_;
    double azimuth_;
    double elevation_;
    double rcs_;
};
}
