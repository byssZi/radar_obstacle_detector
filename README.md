***
# 说明
此工程为基于ros的毫米波雷达聚类跟踪工程，聚类采用dbscan算法，使用PCA计算boudingbox的朝向，通过hungarian算法实现匹配跟踪。</br>
工程改编自 https://github.com/alvanli/RadarClustering
***
# Step1
发布你的radar话题，格式如下之一即可</br>
## sensor_msgs::PoindCloud2
|参数名称|功能描述|备注|
|---|---|---|
|x|x坐标| - |
|y|y坐标| - |
|z|z坐标| - |
|Range|RCS反射强度| - |
|Velocity|速度大小| - |
|AzimuthAngle|速度水平方位角| - |
|ElevationAngle|速度垂直方位角| - |
## Cluster自定义消息
|参数名称|功能描述|备注|
|---|---|---|
|id|雷达点的id| - |
|position|雷达点的坐标| - |
|relative_velocity|雷达点的速度| - |
|rcs|RCS反射强度| - |
# Step2
***
```bash
catkin_make
source devel/setup.bash
```
***
# Step3
修改`launch\run.launch`文件
|参数名称|功能描述|备注|
|---|---|---|
|radar_points_topic|sensor_msgs::PoindCloud2格式的话题名称|与Cluster自定义消息话题发布其一即可|
|radar_cluster_topic|Cluster自定义消息的话题名称|与sensor_msgs::PoindCloud2话题消息发布其一即可|
|visualize_radar_topic|聚类boudingbox可视化话题| - |
|dbscan_bbox_topic|聚类boudingbox结果话题| - |
|cluster_output_topic|障碍物点云话题| - |
```bash
roslaunch radar_obstacle_detector run.launch
```
***