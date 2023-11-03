#include "move_goal_send/move_goal_send.h"


MoveGoalSend::MoveGoalSend() {
  ros::NodeHandle n_("~");

  n_.param<std::string>("pose_stamped_subscriber_topic_name", param_pose_stamped_subscriber_topic_name_, "/aruco_single/pose");
  n_.param<std::string>("move_base_name", param_move_base_name_, "/move_base");
  n_.param<std::string>("move_base_frame_id", param_move_base_frame_id_, "map");
  n_.param<float>("move_goal_send_rate_hz", param_move_goal_send_rate_hz_, 1.0);

  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
  pose_stamped_subscriber_ = n_.subscribe(param_pose_stamped_subscriber_topic_name_, 1, &MoveGoalSend::PoseStampedCallback, this);
  aruco_global_angles_publisher_ = n_.advertise<geometry_msgs::Vector3>("/aruco_global_angles", 10);
  pose_local_msg_ = geometry_msgs::PoseStamped();
  pose_global_msg_ = geometry_msgs::PoseStamped();
  is_msg_sended_ = false;
}

void MoveGoalSend::PoseStampedCallback(const geometry_msgs::PoseStamped& pose_stamped_msg) {
  tf2::Quaternion quat_tf;
  pose_local_msg_ = pose_stamped_msg;
  is_msg_sended_ = true;
  tf_buffer_.transform<geometry_msgs::PoseStamped>(pose_local_msg_, pose_global_msg_, param_move_base_frame_id_);
  // ROS_INFO("Aruco pose in global coordinates is:\n x,y,z = %.1f,%.1f,%.1f",
  //       pose_global_msg_.pose.position.x,
  //       pose_global_msg_.pose.position.y,
  //       pose_global_msg_.pose.position.z);

  tf2::convert(pose_global_msg_.pose.orientation, quat_tf);
  tf2::Matrix3x3(quat_tf).getRPY(aruco_roll_global_, aruco_pitch_global_, aruco_yaw_global_);

  ROS_INFO("Aruco angles in global coordinates is:\n x,y,z = %.1f,%.1f,%.1f",
      aruco_roll_global_,
      aruco_pitch_global_,
      aruco_yaw_global_);
}

void MoveGoalSend::Run() {
  MoveBaseClient move_base_action_client(param_move_base_name_, true);
  ros::Rate rate(param_move_goal_send_rate_hz_);
  geometry_msgs::Vector3 aruco_global_angles_msg;
  while ((ros::ok()) && (!move_base_action_client.waitForServer(ros::Duration(5.0)))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;

  while(ros::ok()) {
    if (is_msg_sended_ == false) {
      rate.sleep();
      ros::spinOnce();
      continue;
    }


    pose_global_msg_.pose.position.z = 0.0;
    pose_global_msg_.pose.orientation.x = 0.0;
    pose_global_msg_.pose.orientation.y = 0.0;
    goal.target_pose = pose_global_msg_;
    goal.target_pose.header.frame_id = param_move_base_frame_id_;
    goal.target_pose.header.stamp = ros::Time::now();

    move_base_action_client.sendGoal(goal);
    move_base_action_client.waitForResult();
    aruco_global_angles_msg.x = aruco_pitch_global_;
    aruco_global_angles_msg.y = aruco_roll_global_;
    aruco_global_angles_msg.z = aruco_yaw_global_;

    aruco_global_angles_publisher_.publish(aruco_global_angles_msg);

    if(move_base_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Success");
    }else {
      ROS_INFO("Fail");
    }

    rate.sleep();
    ros::spinOnce();
  }
}
