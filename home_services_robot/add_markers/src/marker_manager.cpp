#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Empty.h>
#include <stdlib.h> 
#include <iostream>

visualization_msgs::Marker pickupMarker;
visualization_msgs::Marker dropOffMarker;
ros::Publisher marker_pub;

bool isPickupMarkerOn = false;
bool isDropOffMarkerOn = false;

using namespace std;

visualization_msgs::Marker createMarker(string name, int id, float x, float y) {
	uint32_t shape = visualization_msgs::Marker::CUBE;
	
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	
	marker.ns = name;
	marker.id = id;
	
	marker.type = shape;
	
	marker.action = visualization_msgs::Marker::ADD;
	
	marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
	
	marker.lifetime = ros::Duration();
	
	return marker;
}


void togglePickup(const std_msgs::Empty& toggle_msg){
	
	cout << "Toggle Pickup Marker" << endl;
	
	isPickupMarkerOn = !isPickupMarkerOn;
	
	if (isPickupMarkerOn) {
		pickupMarker.action = visualization_msgs::Marker::ADD;
	} else {
		pickupMarker.action = visualization_msgs::Marker::DELETE;
	}
	
	
	marker_pub.publish(pickupMarker);	
}

void toggleDropOff(const std_msgs::Empty& toggle_msg){
	
	cout << "Toggle DropOff Marker" << endl;
	
	isDropOffMarkerOn = !isDropOffMarkerOn;
	
	if (isDropOffMarkerOn) {
		dropOffMarker.action = visualization_msgs::Marker::ADD;
	} else {
		dropOffMarker.action = visualization_msgs::Marker::DELETE;
	}
	
	
	marker_pub.publish(dropOffMarker);	
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "marker_manager");
	ros::NodeHandle n;
	ros::Rate r(10);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	
	// wait for rviz (or similar tool) to start up.
	while (marker_pub.getNumSubscribers() < 1)
    {
		if (!ros::ok())
		{
			return 0;
		}
		ROS_INFO("Please create a subscriber to the marker");
		sleep(1);
    }
	
	ROS_INFO("Marker Manager Started");
	
	pickupMarker = createMarker("Pick-up Zone",0, 5.3, -4.3);
	dropOffMarker = createMarker("Drop-off Zone", 1, 0.0, 0.0);
	
	ros::Subscriber pickupMarkerSubscriber = n.subscribe("toggle_pickup", 1, &togglePickup);
	ros::Subscriber dropOffMarkerSubscriber = n.subscribe("toggle_dropoff", 1, &toggleDropOff);
	
	while (ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}
}