#pragma once

#include <actionlib/server/simple_action_server.h>
#include <boost/utility/in_place_factory.hpp>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <six_point_two_eight/GetPointCloud2Action.h>

#include "six_point_two_eight/utilities.h"

namespace six_point_two_eight {
  class GetPointCloud2Server : public nodelet::Nodelet {
  private:
    ros::Subscriber points_subscriber_;
    boost::optional<actionlib::SimpleActionServer<six_point_two_eight::GetPointCloud2Action>> get_point_cloud_2_action_server_;
    
    auto getPointCloud2(const std::string& target_topic) {
      auto points_subscribing_time = ros::Time::now();
      points_subscriber_ = getNodeHandle().subscribe<sensor_msgs::PointCloud2>(
        target_topic, 1,
        [&, points_subscribing_time](const auto& message) {
          // 起動直後は動作が安定しないので、3秒間は何もしません。
          if (message->header.stamp < points_subscribing_time + ros::Duration(3.0)) {
            return;
          }
          
          get_point_cloud_2_action_server_->setSucceeded(createGetPointCloud2ResultMsg(message));
          points_subscriber_.shutdown();
        });
    }

  public:
    void onInit() {
      get_point_cloud_2_action_server_ = boost::in_place(getNodeHandle(), "get_point_cloud_2", false);
      get_point_cloud_2_action_server_->registerGoalCallback(
        [&]() {
          getPointCloud2(get_point_cloud_2_action_server_->acceptNewGoal()->target_topic);
        });
      get_point_cloud_2_action_server_->start();
    }
  };
}

