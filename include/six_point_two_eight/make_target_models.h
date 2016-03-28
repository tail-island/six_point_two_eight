#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include "six_point_two_eight/point_cloud_utilities.h"
#include "six_point_two_eight/utilities.h"

namespace six_point_two_eight {
  class MakeTargetModels : public nodelet::Nodelet {
  private:
    static constexpr double radius = 1.25;
    static const int step_size = 16;

    ros::Subscriber odom_subscriber_;
    CallGetPointCloud2Action::TopicBinded get_point_cloud_2_;
    CallMoveBaseAction::TopicBinded move_base_;

  public:
    void onInit() {
      get_point_cloud_2_ = CallGetPointCloud2Action::bindTopic("get_point_cloud_2");
      move_base_ = CallMoveBaseAction::bindTopic("move_base");

      odom_subscriber_ = getNodeHandle().subscribe<nav_msgs::Odometry>(
        "odom", 1,
        [&](const auto& message) {
          auto position = message->pose.pose.position;
          auto yaw = tf::getYaw(message->pose.pose.orientation);

          auto target_position = createPointMsg(position.x + std::cos(yaw) * radius, position.y + std::sin(yaw) * radius);
          auto target_yaw = normalizeAngle(yaw + M_PI);  // 対象物からTurtleBotを見た場合のヨー。

          // 撮影しながら、撮影ポイントを回ります。
          for (auto i = 0; i < step_size; ++i) {
            ROS_INFO_STREAM("Step " << (i + 1) << "/" << step_size);

            while (ros::ok()) {
              try {
                savePointCloud2File(                                                               // 点群を保存します。
                  removeFloorFromPointCloud2(                                                      // 点群から床を除去します。
                    transformPointCloud2(                                                          // 追加。誤差補正での回転処理をやりやすくするために、原点が中心になるように補正します。
                      getSpherePointCloud2(                                                        // 精度が低いセンサーから遠くの点を削除するために、球状の点群に切り取ります。
                        transformPointCloud2(                                                      // オドメトリー座標系に変換して、点群同士を重ねあわせられるようにします。
                          sensor_msgs::PointCloud2ConstPtr(                                        // PointCloud2をPointCloud2ConstPtrに変換します。
                            new sensor_msgs::PointCloud2(
                              get_point_cloud_2_(createGetPointCloud2GoalMsg("points")).points)),  // alitionlibから点群を取得します。
                          "odom"),
                        target_position,
                        radius * 0.5),
                      createTransformMsg(-target_position.x, -target_position.y, 0))));  // 追加。
                
                break;
              
              } catch (const std::exception& ex) {
                ROS_WARN_STREAM(ex.what());
              }
            }

            target_yaw = normalizeAngle(target_yaw + M_PI * 2 / step_size);  // 対象物から見た、次の撮影ポイントのヨー。
            move_base_(
              createMoveBaseGoalMsg(
                "odom",
                ros::Time::now(),
                target_position.x + std::cos(target_yaw) * radius,
                target_position.y + std::sin(target_yaw) * radius,
                normalizeAngle(target_yaw + M_PI)));
          }

          ROS_INFO_STREAM("Finished!");
          odom_subscriber_.shutdown();
        });
    }
  };
}
