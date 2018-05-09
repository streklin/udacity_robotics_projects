#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Empty.h>
#include <stdlib.h> 
#include <iostream>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool navigateTo(float x, float y, float z, float w) {
	
  
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;

   // Send the goal position and orientation for the robot to reach
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();


  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
  }
 
  return false;

}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects_with_markers");
  ros::NodeHandle n;
  
  ros::Publisher togglePickup = n.advertise<std_msgs::Empty>("toggle_pickup",1);
  ros::Publisher toggleDropOff = n.advertise<std_msgs::Empty>("toggle_dropoff",1);
  std_msgs::Empty myMsg;
  
  // need to wait before sending first message
  sleep(1);
  
  togglePickup.publish(myMsg);
  
  // navigate to pick up zone
  std::cout << "Sending Pick Up Goal." << std::endl;
  bool pickUpZoneReached = navigateTo(5.3, -4.3, 0.0, 1.0);
  
  if (!pickUpZoneReached) {
	  std::cout << "Failed to reach pickup zone." << std::endl;
	  return 0;
  } 
  
  std::cout << "Reach pickup zone, waiting 5 seconds." << std::endl;
  togglePickup.publish(myMsg);
  sleep(5);
  

  std::cout << "Sending Drop-off Goal" << std::endl;
  bool dropOffZoneReached = navigateTo(0.0, 0.0, 0.0, 1.0);
  if (dropOffZoneReached) {
	  std::cout << "Drop off zone reached." << std::endl;
	  // navigate to drop off zone
	  toggleDropOff.publish(myMsg);
  } else {
	  std::cout << "Drop-off Failed." << std::endl;
  }
  
  sleep(5);

  return 0;
}