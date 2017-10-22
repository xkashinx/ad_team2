#include "ros/ros.h"



int main(int argc, char **argv) {
	ros::init(argc, argv, "spinTest");
	ros::NodeHandle n;

	ros::Rate loop_rate(1);

	int count = 0;

	while(ros::ok()) {
		ROS_INFO("%i\n\n", count++);


		loop_rate.sleep();
	}
	
	return 0;
}