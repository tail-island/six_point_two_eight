#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include "six_point_two_eight/point_cloud_utilities.h"
#include "six_point_two_eight/utilities.h"

namespace six_point_two_eight {
  class MakeWorldModels : public nodelet::Nodelet {
  private:
    ros::Publisher velocity_publisher_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber points_subscriber_;
    ros::Timer timer_1_;
    ros::Timer timer_2_;

  public:
    void onInit() {
      velocity_publisher_ = getNodeHandle().advertise<geometry_msgs::Twist>("velocity", 1);

      // ラムダ式間で共有するステート。C++のラムダ式は寿命管理をしないので、スマート・ポインターで寿命管理します。
      struct State {
        geometry_msgs::Point position;
      };
      std::shared_ptr<State> state(new State());

      odom_subscriber_ = getNodeHandle().subscribe<nav_msgs::Odometry>(
        "odom", 1,
        [&, state](const auto& message) {
          state->position = message->pose.pose.position;
        });

      auto points_subscribing_time = ros::Time::now();
      points_subscriber_ = getNodeHandle().subscribe<sensor_msgs::PointCloud2>(
        "points", 10,
        [&, state, points_subscribing_time](const auto& message) {
          // 起動直後は動作が安定しないので、3秒間は何もしません。
          if (message->header.stamp < points_subscribing_time + ros::Duration(3.0)) {
            return;
          }
          
          try {
            savePointCloud2File(            // 点群を保存します。
              transformPointCloud2(         // 追加。誤差補正での回転処理をやりやすくするために、原点が中心になるように補正します。
                getSpherePointCloud2(       // 精度が低いセンサーから遠くの点を削除するために、球状の点群に切り取ります。
                  transformPointCloud2(     // オドメトリー座標系に変換して、点群同士を重ねあわせられるようにします。
                    downsamplePointCloud2(  // 点の数を減らして、処理の負荷を減らします。
                      message,
                      0.005),
                    "odom",
                    -0.05),
                  state->position,
                  1.5),
                createTransformMsg(-state->position.x, -state->position.y, 0)));
            
          } catch (const std::exception& ex) {
            ROS_WARN_STREAM(ex.what());
          }
        });

      timer_1_ = getNodeHandle().createTimer(
        ros::Duration(0.1),
        [&](const auto& message) {
          velocity_publisher_.publish(createTwistMsg(0.0, M_PI * 2 / 30));  // 30秒で1回転するペースで、回転させます。
        });
      
      timer_2_ = getNodeHandle().createTimer(
        ros::Duration(45.0),  // 計算上は30+3秒でよいのですけれど、それでは一周しないので余裕を持たせて45秒にします。
        [&](const auto& message) {
          ROS_INFO_STREAM("Finished!");
          
          points_subscriber_.shutdown();
          timer_1_.stop();
          timer_2_.stop();
        });
    }
  };
}
