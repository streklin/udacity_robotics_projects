#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <iostream>
#include <stdlib.h>

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
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
	
	marker.lifetime = ros::Duration();
	
	return marker;
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "add_markers");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);


	// wait for rviz (or similar tool) to start up.
	while (marker_pub.getNumSubscribers() < 1)
    {
		if (!ros::ok())
		{
			return 0;
		}
		ROS_WARN_ONCE("Please create a subscriber to the marker");
		sleep(1);
    }
	
	// publish marker at pickup zone
	cout << "Create pick-up marker" << endl;
	visualization_msgs::Marker pickupMarker = createMarker("Pick-up Zone",0, 5.3, -4.3);
	marker_pub.publish(pickupMarker);
	
	// wait five seconds
	cout << "Wait five seconds" << endl;
	sleep(5);
	
	// hide pickup marker
	cout << "Hide the marker" << endl;
	pickupMarker.action = visualization_msgs::Marker::DELETE;
	marker_pub.publish(pickupMarker);
	
	// wait five seconds
	cout << "Wait five seconds" << endl;
	sleep(5);
	
	// publish marker at drop-off zone
	cout << "Publish marker at drop off zone" << endl;
	visualization_msgs::Marker dropOffMarker = createMarker("Drop-off Zone", 1, 0.0, 0.0);
	marker_pub.publish(dropOffMarker);
	sleep(5);
	

}