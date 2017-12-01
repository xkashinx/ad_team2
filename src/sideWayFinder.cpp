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
#define ratio (7.525)//(7.525)
typedef struct
{
	double x,y;
} Point;

std::vector<Point> rightWall,leftWall;
ros::Publisher pub,pubError,pub2Python;
double prevSpeed = 0.0;
double prevMinRange = 100*ratio;
int turnStage = 0;

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
	
//	ROS_INFO("Xave:%lf Yave:%lf A:%lf B:%lf C:%lf",Xave,Yave,A,B,C);
}
void calculateAngleDistance(double A,double B,double &ang, double &dist)
{
	ang = -atan(A);
	dist = cos(ang)*B;
	ang = ang*180/3.14159265;
}



void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
//	ROS_INFO("%d",sizeof(msg->ranges));
	//ROS_INFO("Message received");
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


	//ROS_INFO("%ld %ld",leftWall.size(),rightWall.size());

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
		
	if (max_range > 7.5 * ratio) {
		turn = 0; // default no turn or stop the turn mode
		goto skip;
	}


	// if expecting turn
	for(int i = 0; i < 1080; i++) {
		// index of range_max found
		if(abs(msg->ranges[i] - max_range) < 0.0001 * ratio && abs(msg->ranges[i] - msg->ranges[i+1]) > 1.8*ratio) {
			indexMax =  i;
			break;
		}
		/**
		// if jump in rage 
		if (abs(msg->ranges[i] - prevRange) > 1.8*ratio) {
			// if index for max_range have not found yet
			if (indexMax == -1) {
				indexJB = i; // set index of closest jump before max_range
			} else { // if index of max_range already found, decide right or left turn
				rightGap = indexMax - indexJB;
				leftGap = indexJA - indexMax;
				gapDiff = leftGap - rightGap;
				if (leftGap != 0 && leftGap > rightGap )
					turn = 1; // right turn
				else
					turn = -1; // left turn
				break;
			}
		}	
		*/		
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

	//ROS_INFO("La:%0.5lf Ra:%0.5lf Ld:%0.5lf Rd:%0.5lf",Lang,Rang,Ldist,Rdist);
	//ROS_INFO("Ea:%0.5lf Ed:%0.5lf",error.ang,error.dist);
	
	//ROS_INFO("\n\n\nerror.ang = %5.2lf\nerror.dist = %5.2lf\nmax_range = %5.2lf\nindexJB = %i\nindexMax = %i\nindexJA = %i\n------------------\nTurn = %i\nrightGap = %i\nleftGap = %i\nleftGap - rightGap = %i", error.ang, error.dist, max_range/ratio, indexJB, indexMax, indexJA, turn, rightGap, leftGap, gapDiff);

	pid_error.pid_error =(error.dist/ratio+error.ang/45*1.5)*100;
	double p_error = abs(pid_error.pid_error);
	
	double target = 200/(1+p_error)+20;
	//double target = -70*log(p_error+0.5)+130;

	if (turn == 0)
		target = std::max(11*ratio, std::min(26*ratio/(0+abs(error.ang)) + 0, 22*ratio) - 2.5*std::max(0.0, 10-msg->ranges[540]/ratio)*ratio);
	else
		target = 11 * ratio;

	
	if (target > 22 * ratio)
		target = 22 * ratio;

	double velError = target - prevSpeed;
	int sign = 1;
	if (velError < 0)
		sign = -1;
	double velChange = velError;
	int velSign = velChange > 0 ? 1: -1;
	if (abs(velChange)*50/ratio > 7)
		velChange = velSign * 7*ratio/50;
	prevSpeed += velChange;
	pid_error.pid_vel = prevSpeed;
	ROS_INFO("\n\n\n\n\n\nerror.ang = %5.2lf(deg)\nerror.dist = %5.2lf(cm)\nmax_range = %5.2lf(m)\nindexMax = %i (half=540)\n-----------------\ntarget = %5.2lf(m/s)\nstd::max(0.0, 10-max_range/ratio) = %5.2lf(m/s)\nvelError = %5.2lf (m/s)\nvelChange = %5.2lf(m/s^2)\nprevSpeed = %5.2lf(m/s)\n-----------------\nmaxRawAngle = %5.2lf\nmaxAngle = %5.2lf\nTURN = %i (0:none 1:right -1:left)", error.ang, error.dist/ratio*100, max_range/ratio, indexMax, target/ratio, 2.5*std::max(0.0, 10-max_range/ratio), velError/ratio, velChange*50/ratio, prevSpeed/ratio, maxRawAngle, maxAngle, turn);

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
	ros::Subscriber sub = rosHandle.subscribe("catvehicle/front_laser_points",100,callback);
	//ros::Subscriber sub = rosHandle.subscribe("/scan",100,callback);
	pub = rosHandle.advertise<control::sideWay>("control/sideWay",100);
	pubError = rosHandle.advertise<control::angleDistanceError>("control/angleDistanceError",100);	
	pub2Python = rosHandle.advertise<control::pid_input>("control/error",100);

	ros::spin();
	return 0;
}
