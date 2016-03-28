#pragma once

#include <actionlib/server/simple_action_server.h>
#include <boost/utility/in_place_factory.hpp>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>

#include "six_point_two_eight/utilities.h"

namespace six_point_two_eight {
  class MoveBaseServer : public nodelet::Nodelet {
  private:
    ros::Publisher velocity_publisher_;
    ros::Subscriber odom_subscriber_;

    boost::optional<actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>> move_base_action_server_;  // actionlib::SimpleActionServerの初期化はコンストラクタですが、Nodeletの初期化はonInit()です。この矛盾をBoostのIn-Place Factoryを使用します。
    
    auto tunedLinearX(double distance, double angle) const {
      if (distance < 0.05) {
        return 0.0;
      }
      if (std::fabs(angle) > M_PI / 180 * 45) {
        return 0.0;
      }
      
      return std::min(std::fabs(distance) * 0.5, 0.1);
    }
    
    auto tunedAngularZ(double angle) const {
      if (std::fabs(angle) < M_PI / 180 * 1) {
        return 0.0;
      }
      
      return std::min(std::fabs(angle) * 0.5, M_PI / 180 * 10) * (angle / std::fabs(angle));
    }

    auto turnTo(const geometry_msgs::Pose& goal_pose, nav_msgs::OdometryConstPtr odometry) {
      auto angular_z = this->tunedAngularZ(normalizeAngle(tf::getYaw(goal_pose.orientation) - tf::getYaw(odometry->pose.pose.orientation)));

      velocity_publisher_.publish(createTwistMsg(0, angular_z));

      return angular_z != 0.0;
    }

    auto turnTo(const geometry_msgs::Pose& goal_pose) {
      odom_subscriber_ = getNodeHandle().subscribe<nav_msgs::Odometry>(
        "odom", 1,
        [&, goal_pose](const auto& message) {
          if (this->turnTo(goal_pose, message)) {
            return;
          }

          move_base_action_server_->setSucceeded(move_base_msgs::MoveBaseResult());
          odom_subscriber_.shutdown();
        });
    }
    
    auto moveTo(const geometry_msgs::Pose& goal_pose, nav_msgs::OdometryConstPtr odometry) {
      auto difference = createPointMsg(goal_pose.position.x - odometry->pose.pose.position.x, goal_pose.position.y - odometry->pose.pose.position.y);
      
      auto distance = std::sqrt(std::pow(difference.x, 2) + std::pow(difference.y, 2));
      auto angle = normalizeAngle(std::atan2(difference.y, difference.x) - tf::getYaw(odometry->pose.pose.orientation));
      
      auto linear_x = this->tunedLinearX(distance, angle);
      auto angular_z = this->tunedAngularZ(angle);
      
      velocity_publisher_.publish(createTwistMsg(linear_x, angular_z));
      
      return linear_x != 0.0 || angular_z != 0.0;
    }

    auto moveTo(const geometry_msgs::Pose& goal_pose) {
      odom_subscriber_ = getNodeHandle().subscribe<nav_msgs::Odometry>(
        "odom", 1,
        [&, goal_pose](const auto& message) {
          if (this->moveTo(goal_pose, message)) {
            return;
          }

          this->turnTo(goal_pose);
        });
    }

  public:
    void onInit() {
      velocity_publisher_ = getNodeHandle().advertise<geometry_msgs::Twist>("velocity", 1);

      move_base_action_server_ = boost::in_place(getNodeHandle(), "move_base", false);  // boostのIn-Place Factoryを使用して、サーバーを初期化します。
      move_base_action_server_->registerGoalCallback(
        [&]() {
          auto goal = move_base_action_server_->acceptNewGoal();

          geometry_msgs::PoseStamped goal_pose_stamped;
          transform_listener_.transformPose("odom", goal->target_pose, goal_pose_stamped);

          moveTo(goal_pose_stamped.pose);
        });
      move_base_action_server_->start();
    }
  };
}
