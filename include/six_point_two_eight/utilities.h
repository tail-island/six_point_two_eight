#pragma once

#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Transform.h>  // 追加。
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <six_point_two_eight/GetPointCloud2Action.h>

namespace six_point_two_eight {
  // 例外。
  
  class SixPointTwoEightException : public std::exception {
  private:
    std::string what_;

  public:
    SixPointTwoEightException(const std::string& what)
      : what_(what)
    {
      ;
    }

    const char* what() const noexcept {
      return what_.c_str();
    }
  };
  
  // ユーティリティ。
  
  inline auto normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }
  
  // メッセージ生成ヘルパー。
  
  inline auto createTwistMsg(double linear_x, double angular_z) {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    
    return msg;
  }

  inline auto createPointMsg(double x, double y) {
    geometry_msgs::Point msg;
    msg.x = x;
    msg.y = y;

    return msg;
  }

  inline auto createVector3Msg(double x, double y) {
    geometry_msgs::Vector3 msg;
    msg.x = x;
    msg.y = y;

    return msg;
  }
  
  inline auto createQuaternionMsg(double yaw) {
    return tf::createQuaternionMsgFromYaw(yaw);
  }
  
  inline auto createPoseMsg(double x, double y, double yaw) {
    geometry_msgs::Pose msg;
    msg.position = createPointMsg(x, y);
    msg.orientation = createQuaternionMsg(yaw);

    return msg;
  }

  inline auto createTransformMsg(double x, double y, double yaw) {
    geometry_msgs::Transform msg;
    msg.translation = createVector3Msg(x, y);
    msg.rotation = createQuaternionMsg(yaw);

    return msg;
  }
  
  inline auto createPoseStampedMsg(const std::string& frame_id, const ros::Time& stamp, double x, double y, double yaw) {
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.pose = createPoseMsg(x, y, yaw);

    return msg;
  }

  inline auto createMoveBaseGoalMsg(const std::string& frame_id, const ros::Time& stamp, double x, double y, double yaw) {
    move_base_msgs::MoveBaseGoal msg;
    msg.target_pose = createPoseStampedMsg(frame_id, stamp, x, y, yaw);

    return msg;
  }

  inline auto createGetPointCloud2GoalMsg(const std::string& target_topic) {
    six_point_two_eight::GetPointCloud2Goal msg;
    msg.target_topic = target_topic;

    return msg;
  }

  inline auto createGetPointCloud2ResultMsg(sensor_msgs::PointCloud2ConstPtr points) {
    six_point_two_eight::GetPointCloud2Result msg;
    msg.points = *points;

    return msg;
  }

  // actionlib呼び出しヘルパー。

  template <typename Action>
  struct CallAction {
    using Goal        = typename Action::_action_goal_type::_goal_type;
    using Result      = typename Action::_action_result_type::_result_type;
    using TopicBinded = std::function<Result(const Goal&)>;

    static auto bindTopic(const std::string& topic) {
      return std::bind(CallAction<Action>(), topic, std::placeholders::_1);
    }

    auto exceptionWhat(const std::string& message, const std::string& topic) const {
      std::stringstream what;
      what << message << " topic = " << topic << ".";
      
      return what.str();
    }
    
    auto operator()(const std::string& topic, const Goal& goal) {
      actionlib::SimpleActionClient<Action> action_client(topic, true);
      if (!action_client.waitForServer(ros::Duration(60.0))) {
        throw SixPointTwoEightException(exceptionWhat("Timeout occured when waiting action server.", topic));
      }
      
      action_client.sendGoal(goal);
      if (!action_client.waitForResult(ros::Duration(60.0))) {
        throw SixPointTwoEightException(exceptionWhat("Timeout occured when waiting result.", topic));
      }
      
      if (action_client.getState() != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
        throw SixPointTwoEightException(exceptionWhat(action_client.getState().toString(), topic));
      }
      
      return *action_client.getResult();
    }
  };
  
  using CallMoveBaseAction = CallAction<move_base_msgs::MoveBaseAction>;
  using CallGetPointCloud2Action = CallAction<six_point_two_eight::GetPointCloud2Action>;

  // TFヘルパー。

  extern tf::TransformListener transform_listener_;

  inline auto transformMsgToTF(const geometry_msgs::Transform& transform) {
    tf::Transform converted_transform;
    tf::transformMsgToTF(transform, converted_transform);

    return converted_transform;
  }

  inline auto transformTFToMsg(const tf::Transform& transform) {
    geometry_msgs::Transform converted_transform;
    tf::transformTFToMsg(transform, converted_transform);

    return converted_transform;
  }
}
