#pragma once

#include <nodelet_topic_tools/nodelet_throttle.h>
#include <sensor_msgs/PointCloud2.h>

namespace six_point_two_eight {
  using PointCloud2Throttle = nodelet_topic_tools::NodeletThrottle<sensor_msgs::PointCloud2>;
}
