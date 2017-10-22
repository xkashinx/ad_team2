#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "control/drive_param.h"
ros::Publisher pub;
geometry_msgs::Twist msg;
void timerCallback(const ros::TimerEvent& event)
{
        pub.publish(msg);
}
void callback(const control::drive_param::ConstPtr& in_msg)
{
        msg.linear.x  =  in_msg->velocity/100*10;
        msg.angular.z = -in_msg->angle/100*30/180*3.14159265;
}
int main(int argc, char ** argv)
{
        ros::init(argc,argv,"talkerToSim");
        ROS_INFO("Talker to Simulator Start");
        ros::NodeHandle rosHandle;
        ros::Subscriber sub = rosHandle.subscribe("control/drive_parameters",10,callback);
        pub = rosHandle.advertise<geometry_msgs::Twist>("/catvehicle/cmd_vel",100);

        ros::Timer timer = rosHandle.createTimer(ros::Duration(0.05),timerCallback);

        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;

        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;

        ros::spin();
        return 0;
}

