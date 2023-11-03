#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class aruco_goal {

private:
    /* data */
public:
    ros::NodeHandle nh;
    ros::Publisher move_base_goal_pub;

    aruco_goal();
};


aruco_goal::aruco_goal() {

}

int main(int argc, char** argv){
  //test-array
  //the first index - value of the ArUco marker
  //the second index - x in the coordinates
  //the third index - y in the coordinates
  int a[6][3]={{0,0,0},{1,1,1},{2,2,2},{3,3,3},{4,-3,-3},{5,0,0}};

  int marker=0;
  int breaknumber=10;//you can change this later
  //test-loop
  //the condition can be changed due to the actual circumstance
  while(marker!=breaknumber){
    printf("Please input the value of the landmark:\n");
    int temp=marker;//temp is used to save the former value
    //because we are using absolute, not relative, coordinates in this project
    scanf("%d",&marker);
    if(marker==breaknumber){
      break;
    }

    ros::init(argc, argv, "aruco_goal");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(10.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = a[marker][1] - a[temp][1];
    goal.target_pose.pose.position.y = a[marker][2] - a[temp][2];
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved %d meter in the direction of x and %d meter in the direction of y\n", a[marker][1],a[marker][2]);
    else
      ROS_INFO("The base failed to move for some reason\n");
  }
  ROS_INFO("Voila! This is the end!\n");
  return 0;
}