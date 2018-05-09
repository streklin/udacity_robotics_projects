// ROS Libraries
#include "ros/ros.h"
#include "geometry_msgs/Twist.h" // Motor Commands
#include "sensor_msgs/LaserScan.h" // Laser Data
#include "tf/transform_listener.h" // tf Tree

// C++ Libraries
#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>

#define ROBOT_SPEED 0.15 // Keep a low speed for the best results
#define ANGULAR_SPEED 0.2

#define NAN_THRESHOLD 0.9
#define CRASHED_THRESHOLD 0.25
#define FOLLOW_THRESHOLD 0.5
#define WALL_THRESHOLD 2.0
#define DOOR_THRESHOLD 1.0

// ROS Publisher:Motor Commands, Subscriber:Laser Data, and Messages:Laser Messages & Motor Messages
ros::Publisher motor_command_publisher;
ros::Subscriber laser_subscriber;
sensor_msgs::LaserScan laser_msg;
geometry_msgs::Twist motor_command;

// Define the robot direction of movement
typedef enum _ROBOT_MOVEMENT {
    STOP = 0,
    FORWARD,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    GO_RIGHT,
    GO_LEFT

} ROBOT_MOVEMENT;

class WallFollower {
private:
	bool isFollowingWall;
	bool isCrashed;
	bool isDoor;
	float avgRightScan;
	float avgLeftScan;
	float rangeMin;
	float rangeMax;
	int nanCount;
	size_t rangeSize;
	std::vector<float> laserRanges;
	
	void updateStateFromLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
	void checkCrashed();
	void checkWallFollowing();
	void checkForDoor();
	void followWall();
	void handleDoor();
	void moveRobot(const ROBOT_MOVEMENT move_type);
	
public:
	WallFollower();
	
	void doNextAction(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
};


WallFollower::WallFollower() {
	this->isFollowingWall = false;
	this->isCrashed = false;
	this->isDoor = false;
	this->avgRightScan = 0;
	this->avgLeftScan = 0;
	this->rangeMin = 0;
	this->rangeMax = 0;
	this->nanCount = 0;
}

void WallFollower::updateStateFromLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
	ROS_INFO("updateStateFromLaserScan()");
	
	// Read and process laser scan values
    laser_msg = *scan_msg;
    std::vector<float> laser_ranges;
    laser_ranges = laser_msg.ranges;
    size_t range_size = laser_ranges.size();
    
	float left_side = 0.0, right_side = 0.0;
    float range_min = laser_msg.range_max, range_max = laser_msg.range_min;
    int nan_count = 0;
	
	int rightCount = 0;
	int leftCount = 0;
    
	for (size_t i = 0; i < range_size; i++) {
		
		if (std::isnan(laser_ranges[i])) {
            nan_count++;
		}
		
		if (laser_ranges[i] < range_min) {
            range_min = laser_ranges[i];
        }
		
		if (i < range_size / 4) {
            if (laser_ranges[i] > range_max) {
                range_max = laser_ranges[i];
            }
        }
		
		if (i > range_size / 2) {
            left_side += laser_ranges[i];
			leftCount++;
        } else {
            right_side += laser_ranges[i];
			rightCount++;
        }
	}

	this->avgLeftScan = left_side;
	this->avgRightScan = right_side;
	this->rangeMin = range_min;
	this->rangeMax = range_max;
	this->nanCount = nan_count;
	this->rangeSize = range_size;
	this->laserRanges = laser_ranges;
}

void WallFollower::checkCrashed() {
	ROS_INFO("checkCrashed()");
	// Check if the robot has crashed into a wall
    if (this->nanCount > (this->rangeSize * NAN_THRESHOLD) || this->laserRanges[this->rangeSize / 2] < CRASHED_THRESHOLD) {
        this->isCrashed = true;
    }
    else {
        this->isCrashed = false;
    }
}

void WallFollower::checkWallFollowing() {
	ROS_INFO("checkWallFollowing()");
	
	if (this->rangeMin <= FOLLOW_THRESHOLD && this->rangeMax <= WALL_THRESHOLD) {
		this->isFollowingWall = true;
	} else {
		this->isFollowingWall = false;
	}
}

void WallFollower::followWall() {
	ROS_INFO("followWall()");
	
	this->moveRobot(STOP);
	
	if (this->avgLeftScan >= this->avgRightScan) {
		this->moveRobot(TURN_RIGHT);
	} else {
		this->moveRobot(TURN_LEFT);
	}
}

void WallFollower::checkForDoor() {
	ROS_INFO("checkForDoor()");
	
	if (this->laserRanges[0] <= DOOR_THRESHOLD || this->rangeMax <= WALL_THRESHOLD) {
		this->isDoor = false;
		return;
	}
	
	
	this->isDoor = true;
	
}

void WallFollower::handleDoor() {
	
	ROS_INFO("handleDoor()");
	this->moveRobot(GO_RIGHT);
}

void WallFollower::doNextAction(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
	ROS_INFO("----------------------");
	ROS_INFO("doNextAction()");
	
	this->updateStateFromLaserScan(scan_msg);
	this->checkCrashed();
	
	if (this->isCrashed) {
		this->moveRobot(BACKWARD);
		return;
	}
	
	this->checkWallFollowing();
	if (this->isFollowingWall) {
		followWall();
		return;
	}
	
	this->checkForDoor();
	if (this->isDoor) {
		this->handleDoor();
		return;
	}
	

	this->moveRobot(FORWARD);

	
}

void WallFollower::moveRobot(const ROBOT_MOVEMENT move_type) {
	if (move_type == STOP) {
        ROS_INFO("[ROBOT] HALT! \n");

        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.0;
    }

    else if (move_type == FORWARD) {
        ROS_INFO("[ROBOT] FORWARD! \n");
        motor_command.angular.z = 0.0;
        motor_command.linear.x = ROBOT_SPEED;
    }

    else if (move_type == BACKWARD) {
        ROS_INFO("[ROBOT] BACKWARD \n");
        motor_command.linear.x = -ROBOT_SPEED;
        motor_command.angular.z = 0.0;
    }

    else if (move_type == TURN_LEFT) {
        ROS_INFO("[ROBOT] TURN_LEFT \n");
        motor_command.linear.x = 0.0;
        motor_command.angular.z = ANGULAR_SPEED;
    }

    else if (move_type == TURN_RIGHT) {
        ROS_INFO("[ROBOT] TURN_RIGHT \n");
        motor_command.linear.x = 0.0;
        motor_command.angular.z = -ANGULAR_SPEED;
    }
    else if (move_type == GO_RIGHT) {
        ROS_INFO("[ROBOT] GO_RIGHT \n");
        motor_command.linear.x = ROBOT_SPEED;
        motor_command.angular.z = -ANGULAR_SPEED;
    }
    else if (move_type == GO_LEFT) {
        ROS_INFO("[ROBOT] GO_LEFT \n");
        motor_command.linear.x = ROBOT_SPEED;
        motor_command.angular.z = ANGULAR_SPEED;
    }
    else {
        ROS_INFO("[ROBOT_MOVE] Move type wrong! \n");
    }

    //Publish motor commands to the robot and wait 10ms
    motor_command_publisher.publish(motor_command);
    usleep(10);
}

int main(int argc, char* argv[]) {
	
	// Initialize the Wall Follower
	WallFollower follower;
	
	// Initialize a ROS node
    ros::init(argc, argv, "wall_follower");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 100
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 100);

    // Subscribe to the /scan topic and call the laser_callback function
    laser_subscriber = n.subscribe("/scan", 1000, &WallFollower::doNextAction, &follower);

    // Enter an infinite loop where the laser_callback function will be called when new laser messages arrive
    ros::Duration time_between_ros_wakeups(0.001);
    while (ros::ok()) {
        ros::spinOnce();
        time_between_ros_wakeups.sleep();
    }

    return 0;
	
}