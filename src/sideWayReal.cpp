#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <math.h>
#include <fstream>
#include "control/sideWay.h"
#include "control/angleDistanceError.h"
#include "control/pid_input.h"
#define dataSize (1081)
#define validSize (150) 
#define ratio (1.0)//sim(7.525) real(1.0)
typedef struct
{
	double x,y;
} Point;

std::vector<Point> rightWall,leftWall;
ros::Publisher pub,pubError,pub2Python;
double prevSpeed = 0.0;
double prevSpeedSum = 0.0;
int count = 0;
double avgSpeed = 0.0;
double MPSToCmd = 10.0; // sim(75.25) real:10%=1m/s -> (10.0) 
double maxMPS = 2.5;
double minMPS = 1;
double stopDistance = 1.5; // meters
double velThresh = 9.5;

Point convertCoord(double range,int index)
{
	double theta = (-135+index*0.25)*3.14159265358979323846264/180.0;
	Point ans;
	ans.x = range*cos(theta);
	ans.y = range*sin(theta);
	return ans;
}
inline double distanceOfPoints(Point A,Point B)
{
	return sqrt((A.x-B.x)*(A.x-B.x)+(A.y-B.y)*(A.y-B.y));
}

void linearFit(std::vector<Point> P,double &A,double &B)
{
	std::vector<Point>::iterator it;
	double Sxy=0,Sx=0,Sy=0,Sxx=0,N;
	N=P.size();
	A=0;
	B=0;
	for (it=P.begin();it!=P.end();it++)
	{
		Sxy += it->x*it->y;
		Sx  += it->x;
		Sy  += it->y;
		Sxx += it->x*it->x;
	}
	A = (Sxy-Sx*Sy/N)/(Sxx-Sx*Sx/N);
	B = Sy/N-A*Sx/N;
	
}
void calculateAngleDistance(double A,double B,double &ang, double &dist)
{
	ang = -atan(A);
	dist = cos(ang)*B;
	ang = ang*180/3.14159265;
}



void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	leftWall.clear();
	rightWall.clear();

	int outrange_count =0;
	Point lastPoint,currentPoint;

	for (int i=1;i<dataSize;i++)
	{	
		if (msg->ranges[i]>4*ratio && msg->ranges[i]<100 && rightWall.size()>0) outrange_count++;

		if (outrange_count>20) break;

		if (msg->ranges[i]<3*ratio)
		{
			currentPoint = convertCoord(msg->ranges[i],i);
			if (rightWall.size()>0 && distanceOfPoints(currentPoint,lastPoint)>0.3*ratio)
				break;
			rightWall.push_back(currentPoint);
			lastPoint = currentPoint;
		}
	}

	outrange_count=0;
	
	for (int i=dataSize-1;i>1;i--)
	{	
		if (msg->ranges[i]>4*ratio && msg->ranges[i]<100 && leftWall.size()>0) outrange_count++;
		
		if (outrange_count>20) break;

		if (msg->ranges[i]<3*ratio)
		{
			currentPoint = convertCoord(msg->ranges[i],i);
			if (leftWall.size()>0 && distanceOfPoints(currentPoint,lastPoint)>0.3*ratio)
				break;
			leftWall.push_back(currentPoint);
			lastPoint = currentPoint;
		}
	}

	control::sideWay side;
	control::angleDistanceError error;
	control::pid_input pid_error;


	linearFit(leftWall,side.LA,side.LB);
	linearFit(rightWall,side.RA,side.RB);
	double Lang,Rang,Ldist,Rdist;

	calculateAngleDistance(side.LA,side.LB,Lang,Ldist);
	calculateAngleDistance(side.RA,side.RB,Rang,Rdist);

	if (leftWall.size()>validSize && rightWall.size()>validSize)
	{
		error.ang  =  (Lang*leftWall.size()+Rang*rightWall.size())/(leftWall.size()+rightWall.size());
		error.dist = -(Ldist*leftWall.size()+Rdist*rightWall.size())/(leftWall.size()+rightWall.size());
	}
	
	if (leftWall.size()<=validSize && rightWall.size()>validSize)
	{	
		error.ang  =  Rang;
		error.dist =  -1*ratio+(-Rdist);
	}

	if (leftWall.size()>validSize && rightWall.size()<=validSize)
	{

		error.ang  =  Lang;
		error.dist =  (1*ratio-Ldist);
	}

	if (leftWall.size()<=validSize && rightWall.size()<=validSize)
	{

		error.ang  =  0;//invalid
		error.dist =  0;
	}

	/**
	** Team 2 START ----------------------------------------------
	**/
	int turn = 0;
	int indexJB = 0;
	int indexMax= -1;
	int indexJA = 1081;
	int foundJB = 0; // boolean to check if jump before max exists
	int foundJA = 0; // boolean to check if jump after max exists
	double prevRange = 0.0;
	double max_range = 0.0;
	double min_range = 100.0*ratio;
	int rightGap = -1;
	int leftGap = -1;
	int gapDiff = 123456;

	double maxRawAngle = -10000.0;
	double maxAngle = -10000.0;

	// find max_range
	for ( int i = 0; i < 1080; i++ ) {
		if (msg->ranges[i] > max_range) 
			max_range = msg->ranges[i];
		if (msg->ranges[i] < min_range)
			min_range = msg->ranges[i];
	}
		
	if (max_range > 8.0 * ratio) {
		turn = 0; // default no turn or stop the turn mode
		goto skip;
	}


	// if expecting turn
	for(int i = 0; i < 1080; i++) {
		// index of range_max found
		if(abs(msg->ranges[i] - max_range) < 0.0001 * ratio && abs(msg->ranges[i] - msg->ranges[i+1]) > 1.8*ratio && abs(msg->ranges[i] - msg->ranges[i+1]) < 2.8*ratio) {
			indexMax =  i;
			break;
		}		
 	}

 	maxRawAngle = indexMax * 270/1080.0 - 270/2.0;
 	maxAngle = maxRawAngle - error.ang;
 	if (maxAngle < 0)
 		turn = 1; // right
 	else
 		turn = -1; // left

 	skip:
	if (turn == 1) 
		error.dist-=0.4*ratio; // right turn, shift left --> error = pos-(mid+0.4m)
	else if (turn == -1) 
		error.dist+=0.4*ratio; // left turn, shift right --> error = pos-(mid-0.4m)
	
	pid_error.pid_error =(error.dist/ratio+error.ang/45*1.5)*100;
	double p_error = abs(pid_error.pid_error);
	
	double target = 200/(1+p_error)+20;

	// set target speed
	if (turn == 0) // in straight line
		target = std::max(minMPS*MPSToCmd, std::min(maxMPS*MPSToCmd/(0+0.2*abs(error.ang)) + 0, maxMPS*MPSToCmd) - 0.3*std::max(0.0, 9-msg->ranges[540]/ratio)*MPSToCmd);
	else // in corner
		target = minMPS * MPSToCmd;

	if (msg->ranges[540] < stopDistance*ratio)
		target = -velThresh+1;//target = (msg->ranges[540]/ratio)/1.5*MPSToCmd;

	double velError = target - prevSpeed;
	int sign = 1;
	if (velError < 0)
		sign = -1;
	double velChange = velError;
	int velSign = velChange > 0 ? 1: -1;
	if (abs(velChange)*50/MPSToCmd > 0.7)
		velChange = velSign * 0.7*MPSToCmd/50;
	prevSpeed += velChange;
	pid_error.pid_vel = prevSpeed;
	prevSpeedSum += prevSpeed/MPSToCmd;
	count++;
	avgSpeed = prevSpeedSum / count;
	double kph = 3.6*prevSpeed/MPSToCmd;
	ROS_INFO("\n\n\n\n\nerror.ang = %5.2lf(deg)\nerror.dist = %5.2lf(cm)\nmax_range = %5.2lf(m)\nfrontRange = %5.2lf\nindexMax = %i (half=540)\n-----------------\nkm/hour = %5.2lf(km/h)\navgSpeed = %5.2lf(m/s)\ntarget = %5.2lf(m/s)\nmax(0.0, 10-max_range/ratio) = %5.2lf(m/s)\nvelError = %5.2lf (m/s)\nvelChange = %5.2lf(m/s^2)\nprevSpeed = %5.2lf(m/s)\n-----------------\nmaxRawAngle = %5.2lf\nmaxAngle = %5.2lf\nTURN = %i (0:none 1:right -1:left)", error.ang, error.dist/ratio*100, max_range/ratio, msg->ranges[540]/ratio, indexMax, kph, avgSpeed, target/MPSToCmd, 0.3*std::max(0.0, 10-max_range/ratio), velError/MPSToCmd, velChange*50/MPSToCmd, prevSpeed/MPSToCmd, maxRawAngle, maxAngle, turn);

	/**
	* Team 2 End -------------------------------------------------------------
	**/


	pub.publish(side);
	pubError.publish(error);
	pub2Python.publish(pid_error);
}

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"side_way_controller");
	ROS_INFO("Side Way Finder Start");
	ros::NodeHandle rosHandle;
	//ros::Subscriber sub = rosHandle.subscribe("catvehicle/front_laser_points",100,callback);
	ros::Subscriber sub = rosHandle.subscribe("/scan",100,callback);
	pub = rosHandle.advertise<control::sideWay>("control/sideWay",100);
	pubError = rosHandle.advertise<control::angleDistanceError>("control/angleDistanceError",100);	
	pub2Python = rosHandle.advertise<control::pid_input>("control/error",100);

	ros::spin();
	return 0;
}
