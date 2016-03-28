#ifndef SIX_POINT_TWO_EIGHT_POINT_CLOUD_UTILITIES_H
#define SIX_POINT_TWO_EIGHT_POINT_CLOUD_UTILITIES_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>

namespace six_point_two_eight {
  sensor_msgs::PointCloud2Ptr downsamplePointCloud2(sensor_msgs::PointCloud2ConstPtr points, double leaf_size);
  geometry_msgs::Point getCentroidOfPointCloud2(sensor_msgs::PointCloud2ConstPtr points);
  sensor_msgs::PointCloud2Ptr getSpherePointCloud2(sensor_msgs::PointCloud2ConstPtr points, const geometry_msgs::Point& centroid, double radius);
  sensor_msgs::PointCloud2Ptr loadPointCloud2File(const std::string& file_path);
  geometry_msgs::Transform registerPointCloud2(sensor_msgs::PointCloud2ConstPtr source_points, sensor_msgs::PointCloud2ConstPtr target_points, double max_correspondence_distance);
  sensor_msgs::PointCloud2Ptr removeFloorFromPointCloud2(sensor_msgs::PointCloud2ConstPtr points);
  sensor_msgs::PointCloud2Ptr removePointCloud2File(sensor_msgs::PointCloud2ConstPtr points);
  sensor_msgs::PointCloud2Ptr savePointCloud2File(sensor_msgs::PointCloud2ConstPtr points);
  sensor_msgs::PointCloud2Ptr showPointCloud2(sensor_msgs::PointCloud2ConstPtr points, const std::string& id, int rgb);
  void spinVisualizer();
  sensor_msgs::PointCloud2Ptr transformPointCloud2(sensor_msgs::PointCloud2ConstPtr points, const geometry_msgs::Transform& transform);
  sensor_msgs::PointCloud2Ptr transformPointCloud2(sensor_msgs::PointCloud2ConstPtr points, const std::string& target_frame, double z_adjust = 0.0);
}

#endif
