#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickup_goal;

  // set up the frame parameters
  pickup_goal.target_pose.header.frame_id = "map";
  pickup_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  //pickup_goal.target_pose.pose.position.x = 1.04;
  //pickup_goal.target_pose.pose.position.y = 0.56;
  pickup_goal.target_pose.pose.position.x = 1.97;
  pickup_goal.target_pose.pose.position.y = -2.02;
  pickup_goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup_goal");
  ac.sendGoal(pickup_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Reached pickup goal");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  //while(!ac.waitForServer(ros::Duration(5.0))){
    //ROS_INFO("Waiting while pickup....");
  //}

  ROS_INFO("Waiting while pickup....");
  ros::Duration(5.0).sleep();

  move_base_msgs::MoveBaseGoal drop_goal;

  drop_goal.target_pose.header.frame_id = "map";
  drop_goal.target_pose.header.stamp = ros::Time::now();

  drop_goal.target_pose.pose.position.x = -0.32;
  drop_goal.target_pose.pose.position.y = -4.07;
  drop_goal.target_pose.pose.orientation.w = 1;

  ROS_INFO("Sending drop_goal");
  ac.sendGoal(drop_goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Reached drop goal");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}
