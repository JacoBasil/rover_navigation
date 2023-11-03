#include "move_goal_send/move_goal_send.h"

#define ROS_NODE_NAME_ "move_goal_send_node"

int main(int argc, char** argv) {
  ros::init(argc, argv, ROS_NODE_NAME_);

  MoveGoalSend move_goal_send;
  move_goal_send.Run();

  return -1;
}
