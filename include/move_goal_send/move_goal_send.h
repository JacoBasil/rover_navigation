#ifndef MOVE_GOAL_SEND_H_
#define MOVE_GOAL_SEND_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class MoveGoalSend {
public:

  MoveGoalSend();
  void PoseStampedCallback(const geometry_msgs::PoseStamped& pose_stamped_msg);

  void Run();
private:
  ros::NodeHandle n_;

  std::string param_pose_stamped_subscriber_topic_name_;
  std::string param_move_base_name_;
  std::string param_move_base_frame_id_;
  float param_move_goal_send_rate_hz_;

  ros::Subscriber pose_stamped_subscriber_;
  ros::Publisher aruco_global_angles_publisher_;

  geometry_msgs::PoseStamped pose_local_msg_;
  geometry_msgs::PoseStamped pose_global_msg_;
  int is_msg_sended_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener* tf_listener_;
  
  double aruco_roll_global_;
  double aruco_pitch_global_;
  double aruco_yaw_global_;
};

#endif
