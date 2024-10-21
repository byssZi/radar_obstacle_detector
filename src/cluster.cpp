#include "cluster.h"

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

    current_id++;
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

    for (int j = 0; j < matches.size(); ++j)
    {
      // find the index of the previous box that the current box corresponds to
      const auto pre_id = pre_ids[matches[j]];
      const auto pre_index = searchBoxIndex(prev_boxes, pre_id);
      
      // find the index of the current box that needs to be changed
      const auto cur_id = cur_ids[j]; // right and matches has the same size
      const auto cur_index = searchBoxIndex(curr_boxes, cur_id);
      
      if (pre_index > -1 && cur_index > -1)
      {
        // change the id of the current box to the same as the previous box
        curr_boxes[cur_index].id = prev_boxes[pre_index].id;
      }
    }
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
  const float x_dim = 2 * (a.dimension[0] - b.dimension[0]) / (a.dimension[0] + b.dimension[0]);
  const float y_dim = 2 * (a.dimension[1] - b.dimension[1]) / (a.dimension[1] + b.dimension[1]);
  const float z_dim = 2 * (a.dimension[2] - b.dimension[2]) / (a.dimension[2] + b.dimension[2]);

  if (ctr_dis <= displacement_thresh && x_dim <= iou_thresh && y_dim <= iou_thresh && z_dim <= iou_thresh)
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

  return -1;
}


}
