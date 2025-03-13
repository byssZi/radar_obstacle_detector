#pragma once

#include <chrono>
#include <unordered_map>
#include <unordered_set>
#include "ukf.h"
#include "cluster.h"

namespace dbscan {

    class ObstacleTracker
    {
     public:
      ObstacleTracker() {};
      virtual ~ObstacleTracker() {};
    
      // ****************** Tracking ***********************
      void obstacleTracking(const std::vector<Cluster::BoundingBox>& prev_boxes, std::vector<Cluster::BoundingBox>& curr_boxes, const float displacement_thresh, const float iou_thresh);
    
     private:
      std::unordered_map<int, UKF> ukf_states; // 存储每个物体的 UKF 状态
      std::unordered_map<int, int> match_counts;
    
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
    
      // Helper function for checking if a point is inside a bounding box
      bool isPointInBoundingBox(const Eigen::Vector3f& point, const Cluster::BoundingBox& box);
    };

}