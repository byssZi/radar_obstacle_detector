#include "radar_obstacle_detector/obstacletracker.h"


namespace dbscan {

void ObstacleTracker::obstacleTracking(const std::vector<Cluster::BoundingBox>& prev_boxes, std::vector<Cluster::BoundingBox>& curr_boxes, const float displacement_thresh, const float iou_thresh)
{
  // Tracking (based on the change in size and displacement between frames)
  
  if (curr_boxes.empty() || prev_boxes.empty())
  {
    return;
  }
  else
  {
    // vectors containing the id of boxes in left and right sets
    std::vector<int> pre_ids;
    std::vector<int> cur_ids;
    std::vector<int> matches;

    // Associate Boxes that are similar in two frames
    auto connection_pairs = associateBoxes(prev_boxes, curr_boxes, displacement_thresh, iou_thresh);

    if (connection_pairs.empty()) return;

    // Construct the connection matrix for Hungarian Algorithm's use
    auto connection_matrix = connectionMatrix(connection_pairs, pre_ids, cur_ids);

    // Use Hungarian Algorithm to solve for max-matching
    matches = hungarian(connection_matrix);

    // Update the unmatched count for each UKF state
    std::unordered_set<int> matched_ids;
    for (int j = 0; j < matches.size(); ++j)
    {
      if (matches[j] < 0 || matches[j] >= pre_ids.size()) continue; // 确保索引在有效范围内

      // find the index of the previous box that the current box corresponds to
      const auto pre_id = pre_ids[matches[j]];
      const auto pre_index = searchBoxIndex(prev_boxes, pre_id);
      
      // find the index of the current box that needs to be changed
      const auto cur_id = cur_ids[j]; // right and matches has the same size
      const auto cur_index = searchBoxIndex(curr_boxes, cur_id);
      
      if (pre_index > -1 && cur_index > -1)
      { // 当前帧和上一帧的障碍物成功匹配上
        // change the id of the current box to the same as the previous box
        curr_boxes[cur_index].id = prev_boxes[pre_index].id;
        //std::cout<< "The obstacle with id " << prev_boxes[pre_index].id << " is successfully tracked." << std::endl;

        // UKF 预测和更新
        if (ukf_states.find(prev_boxes[pre_index].id) == ukf_states.end()) { //直到结尾没找到
          ukf_states[prev_boxes[pre_index].id] = UKF();//创建ukf
          match_counts[prev_boxes[pre_index].id] = 1;
        }
        else{
          match_counts[prev_boxes[pre_index].id]++;
        }
        double rho, phi, rho_dot;
        rho = sqrt(curr_boxes[cur_index].position[0] * curr_boxes[cur_index].position[0] + curr_boxes[cur_index].position[1] * curr_boxes[cur_index].position[1]);
        phi = atan2(curr_boxes[cur_index].position[1], curr_boxes[cur_index].position[0]);
        rho_dot = curr_boxes[cur_index].velocity[0];

        MeasurementPackage meas_package;
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        meas_package.timestamp_= std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
        meas_package.sensor_type_ = MeasurementPackage::RADAR; // 使用毫米波雷达进行初始化
        meas_package.raw_measurements_ = VectorXd(3);
        meas_package.raw_measurements_ << rho, phi, rho_dot;
        ukf_states[prev_boxes[pre_index].id].ProcessMeasurement(meas_package);


        double p_x = ukf_states[prev_boxes[pre_index].id].x_(0);
        double p_y = ukf_states[prev_boxes[pre_index].id].x_(1);
        Eigen::Vector3f ukf_position(p_x, p_y, curr_boxes[cur_index].position[2]);
        if(isPointInBoundingBox(ukf_position, curr_boxes[cur_index])){
          std::cout << "track target id is " << curr_boxes[cur_index].id 
          << ", detect position is (" << curr_boxes[cur_index].position[0] << ", " << curr_boxes[cur_index].position[1] 
          << "), track position is (" << p_x << ", " << p_y << ")" << std::endl;
          curr_boxes[cur_index].position[0] = p_x;
          curr_boxes[cur_index].position[1] = p_y;
        }
        matched_ids.insert(prev_boxes[pre_index].id);
      }
    }
    // 删除未匹配到的 UKF 状态
    for (auto it = ukf_states.begin(); it != ukf_states.end(); )
    {
      if (matched_ids.find(it->first) == matched_ids.end())
      {
        it = ukf_states.erase(it);
      }
      else
      {
        ++it;
      }
    }

    for (auto it = match_counts.begin(); it != match_counts.end(); )
    {
      if (matched_ids.find(it->first) == matched_ids.end())
      {
        it = match_counts.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }
}

bool ObstacleTracker::isPointInBoundingBox(const Eigen::Vector3f& point, const Cluster::BoundingBox& box) {
  // 将点转换到边界框的局部坐标系
  Eigen::Vector3f local_point = box.quaternion.inverse() * (point - box.position);

  // 检查点是否在边界框的范围内
  if (std::abs(local_point.x()) <= box.dimension.x() / 2 &&
      std::abs(local_point.y()) <= box.dimension.y() / 2 &&
      std::abs(local_point.z()) <= box.dimension.z() / 2) {
      return true;
  } else {
      return false;
  }
}

bool ObstacleTracker::compareBoxes(const Cluster::BoundingBox& a, const Cluster::BoundingBox& b, const float displacement_thresh, const float iou_thresh)
{
  // Percetage Displacements ranging between [0.0, +oo]
  const float dis = sqrt((a.position[0] - b.position[0]) * (a.position[0] - b.position[0]) + (a.position[1] - b.position[1]) * (a.position[1] - b.position[1]) + (a.position[2] - b.position[2]) * (a.position[2] - b.position[2]));

  const float a_max_dim = std::max(a.dimension[0], std::max(a.dimension[1], a.dimension[2]));
  const float b_max_dim = std::max(b.dimension[0], std::max(b.dimension[1], b.dimension[2]));
  const float ctr_dis = dis / std::min(a_max_dim, b_max_dim);

  // Dimension similiarity values between [0.0, 1.0]
  const float x_dim = abs(2 * (a.dimension[0] - b.dimension[0]) / (a.dimension[0] + b.dimension[0]));
  const float y_dim = abs(2 * (a.dimension[1] - b.dimension[1]) / (a.dimension[1] + b.dimension[1]));
  const float z_dim = abs(2 * (a.dimension[2] - b.dimension[2]) / (a.dimension[2] + b.dimension[2]));

  if (ctr_dis <= displacement_thresh && x_dim <= iou_thresh && y_dim <= iou_thresh && z_dim <= iou_thresh && isPointInBoundingBox(a.position, b))
  {
    return true;
  }
  else
  {
    return false;
  }
}

std::vector<std::vector<int>> ObstacleTracker::associateBoxes(const std::vector<Cluster::BoundingBox>& prev_boxes, const std::vector<Cluster::BoundingBox>& curr_boxes, const float displacement_thresh, const float iou_thresh)
{
  std::vector<std::vector<int>> connection_pairs;

  for (auto& prev_box : prev_boxes)
  {
    for (auto& curBox : curr_boxes)
    {
      // Add the indecies of a pair of similiar boxes to the matrix
      if (this->compareBoxes(curBox, prev_box, displacement_thresh, iou_thresh))
      {
        connection_pairs.push_back({prev_box.id, curBox.id});
      }
    }
  }

  //std::cout << "Associated " << connection_pairs.size() << " pairs of boxes." << std::endl;

  return connection_pairs;
}

std::vector<std::vector<int>> ObstacleTracker::connectionMatrix(const std::vector<std::vector<int>>& connection_pairs, std::vector<int>& left, std::vector<int>& right)
{
  // Hash the box ids in the connection_pairs to two vectors(sets), left and right
  for (auto& pair : connection_pairs)
  {
    bool left_found = false;
    for (auto i : left)
    {
      if (i == pair[0])
        left_found = true;
    }
    if (!left_found)
      left.push_back(pair[0]);

    bool right_found = false;
    for (auto j : right)
    {
      if (j == pair[1])
        right_found = true;
    }
    if (!right_found)
      right.push_back(pair[1]);
  }

  std::vector<std::vector<int>> connection_matrix(left.size(), std::vector<int>(right.size(), 0));

  for (auto& pair : connection_pairs)
  {
    int left_index = -1;
    for (int i = 0; i < left.size(); ++i)
    {
      if (pair[0] == left[i])
        left_index = i;
    }

    int right_index = -1;
    for (int i = 0; i < right.size(); ++i)
    {
      if (pair[1] == right[i])
        right_index = i;
    }

    if (left_index != -1 && right_index != -1)
      connection_matrix[left_index][right_index] = 1;
  }

  return connection_matrix;
}

bool ObstacleTracker::hungarianFind(const int i, const std::vector<std::vector<int>>& connection_matrix, std::vector<bool>& right_connected, std::vector<int>& right_pair)
{
  for (int j = 0; j < connection_matrix[0].size(); ++j)
  {
    if (connection_matrix[i][j] == 1 && right_connected[j] == false)
    {
      right_connected[j] = true;

      if (right_pair[j] == -1 || hungarianFind(right_pair[j], connection_matrix, right_connected, right_pair))
      {
        right_pair[j] = i;
        return true;
      }
    }
  }
  return false; // 添加返回值
}

std::vector<int> ObstacleTracker::hungarian(const std::vector<std::vector<int>>& connection_matrix)
{
  std::vector<bool> right_connected(connection_matrix[0].size(), false);
  std::vector<int> right_pair(connection_matrix[0].size(), -1);

  int count = 0;
  for (int i = 0; i < connection_matrix.size(); ++i)
  {
    if (hungarianFind(i, connection_matrix, right_connected, right_pair))
      count++;
  }

  std::cout << "For: " << right_pair.size() << " current frame bounding boxes, found: " << count << " matches in previous frame! " << std::endl;

  return right_pair;
}

int ObstacleTracker::searchBoxIndex(const std::vector<Cluster::BoundingBox>& boxes, const int id)
{
  for (int i = 0; i < boxes.size(); i++)
  {
    if (boxes[i].id == id)
      return i;
  }

  return -1; // 确保返回 -1 表示未找到
}

}
