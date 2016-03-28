#include <pluginlib/class_list_macros.h>

#include "six_point_two_eight/get_point_cloud_2_server.h"
#include "six_point_two_eight/make_target_models.h"
#include "six_point_two_eight/make_world_models.h"
#include "six_point_two_eight/move_base_server.h"
#include "six_point_two_eight/point_cloud_2_throttle.h"
#include "six_point_two_eight/register_models.h"

PLUGINLIB_EXPORT_CLASS(six_point_two_eight::GetPointCloud2Server, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(six_point_two_eight::MakeTargetModels, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(six_point_two_eight::MakeWorldModels, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(six_point_two_eight::MoveBaseServer, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(six_point_two_eight::PointCloud2Throttle, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(six_point_two_eight::RegisterModels, nodelet::Nodelet)
