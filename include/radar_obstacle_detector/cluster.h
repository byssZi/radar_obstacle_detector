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

class ObstacleTracker
{
 public:
  ObstacleTracker() {};
  virtual ~ObstacleTracker() {};

  // ****************** Tracking ***********************
  void obstacleTracking(const std::vector<Cluster::BoundingBox>& prev_boxes, std::vector<Cluster::BoundingBox>& curr_boxes, const float displacement_thresh, const float iou_thresh);

 private:

  // ****************** Tracking ***********************
  bool compareBoxes(const Cluster::BoundingBox& a, const Cluster::BoundingBox& b, const float displacement_thresh, const float iou_thresh);

  // Link nearby bounding boxes between the previous and previous frame
  std::vector<std::vector<int>> associateBoxes(const std::vector<Cluster::BoundingBox>& prev_boxes, const std::vector<Cluster::BoundingBox>& curr_boxes, const float displacement_thresh, const float iou_thresh);

  // Connection Matrix
  std::vector<std::vector<int>> connectionMatrix(const std::vector<std::vector<int>>& connection_pairs, std::vector<int>& left, std::vector<int>& right);

  // Helper function for Hungarian Algorithm
  bool hungarianFind(const int i, const std::vector<std::vector<int>>& connection_matrix, std::vector<bool>& right_connected, std::vector<int>& right_pair);

  // Customized Hungarian Algorithm
  std::vector<int> hungarian(const std::vector<std::vector<int>>& connection_matrix);

  // Helper function for searching the box index in boxes given an id
  int searchBoxIndex(const std::vector<Cluster::BoundingBox>& Boxes, const int id);
};

}
