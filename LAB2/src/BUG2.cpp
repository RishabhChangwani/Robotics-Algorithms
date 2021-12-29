
#include<ros/ros.h>
#include<iostream>
#include<std_msgs/String.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include<math.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include<bits/stdc++.h>
#include<cmath>
//#include <LinearMath/btMatrix3x3.h>
using namespace std;

float x_init, y_init, y, m, x,c, A,B,C, d, desired_theta, current_theta, x_current, y_current;
const float x_final = 4.5, y_final = 9.0;
int Goal_Seek = 1, counter = 0 ,Wall_Follow, Turn, first_time = 1, ctr = 0;
void odom_callback(const nav_msgs::Odometry odom)
{
	//cout<<"Checking pose \n ";
	ros::NodeHandle nh;
	ros::Rate Rate(10);
	ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	geometry_msgs::Quaternion msg;
	msg = odom.pose.pose.orientation;
	tf:: Quaternion q;
	tf::quaternionMsgToTF(msg, q);
	double roll, pitch, yaw;
    	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	current_theta = round(yaw*100)/100;
	//x_init = odom.pose.pose.position.x;
	//y_init = odom.pose.pose.position.y;
	//first_time = 0;
	x_current = odom.pose.pose.position.x;
	y_current = odom.pose.pose.position.y;
	m = (y_final - y_current)/(x_final - x_current);
	c = y - m*x;
	A = (y_final - y_current);
	B = (x_current - x_final);
	C = (x_final - x_current) * m;
	d = (unsigned(A*x_current + B*y_current + C)/sqrt(A*A + B*B));
	desired_theta = atan(m);
	desired_theta = round(desired_theta*100)/100;
	
}
void cmd_callback(const sensor_msgs::LaserScan msg)
{
	//cout<<"Distance of cuurent point from Goal line is : " <<d<<endl;
	ros::NodeHandle nh;
	ros::Rate Rate(10);
	ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	geometry_msgs::Twist cmd_vel;
	geometry_msgs::Twist ang_vel;
	geometry_msgs::Vector3 obj;
	geometry_msgs::Vector3 obj2;
	float front_sensor, left_sensor;
	
//Getting Front Sensor Readings
	for(int i = 170; i< 190; i++)
	{
		front_sensor = msg.ranges[i];
		//cout<<"Front Value is : "<<front_sensor<<endl;
	}
//Getting Left Side Readings
	for(int i = 290; i< 300; i++)
	{
		left_sensor = msg.ranges[i];
		//cout<<"\n Left Value is : "<<left_sensor<<endl;
	}
//Check if Goal_Seek is ON
	if(Goal_Seek == 1)
	{	
		//cout<<" \n Inside Goal Seek \n";
		if(current_theta == desired_theta and front_sensor > 0.5)
		{
			cout<<"\n I'm following Goal line \n";
			obj.x = 1;
		}
		else if( current_theta != desired_theta)
		{
			obj2.z = -0.05;
			cout<<"\n Current theta is : "<<current_theta;
			cout<<"\n Desired theta is : "<<desired_theta;
		}
		else if(front_sensor < 0.5)
		{
			cout<<"\n Object Detected, calling turn \n";
//Switch ON Turn and Switch OFF Goal_Seek
			Turn = 1;
			Goal_Seek = 0;
		}
	}
//Check if Turn is switched ON Goal_Seek
	if(Turn == 1 or counter == 1)
	{
		if(left_sensor < 0.5 or left_sensor > 1.5 or front_sensor < 1.5)
		{
			cout<<"\n Turning on the spot to move away from object \n";
			obj.x = 0;
			obj2.z = 1;
			counter = 1;
		}
		else
		{
			cout<<"\n Have moved away from wall, going to Wall Follow \n";
//Switch ON Wall_Follow and switch OFF Turn
			Wall_Follow = 1;
			Turn = 0;
		}
		
	}
//Check if Wall_FOllow is switched ON
	if(Wall_Follow == 1)
	{
		cout<<" \n Wall Follow \n";
		counter =0 ;
		ctr++;
		if(left_sensor < 1.5 and front_sensor>2)
		{
			cout<<"\n Moving parallel to the wall \n";
			obj.x = 1;
			obj2.z = 0;
		}
		else if(left_sensor < 0.7 or left_sensor > 1 or front_sensor < 1)
		{
			cout<<"\n Moving towards the wall \n";
			obj.x = 0.2;
			obj2.z = 1;
		}
		if(d < 0.05 and ctr> 70)
		{
			cout<< "\n Goal line Detected "<< d<<endl;
			obj.x = 0;
//Switch OFF Wall_Follow and switch OFF Goal_Seek
			Wall_Follow = 0;
			Goal_Seek = 1;
			ctr = 0;
		}
	}
	if(abs(x_current - x_final) < 0.5 and abs( y_current - y_final) < 1)
	{
		obj.x = 0;
		cout<<"\n Goal reached \n ";
	}		
		
		
	
	
	cmd_vel.linear = obj;
	cmd_vel.angular = obj2;
	pub_cmd_vel.publish(cmd_vel);
	}
	
	
	
	
int main(int argc, char** argv)
{
	ros::init(argc, argv, "bug2");
	ros::NodeHandle nh;
	ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::Subscriber sub = nh.subscribe("/odom", 10, odom_callback);
	ros::Subscriber sub1 = nh.subscribe("/base_scan",10, cmd_callback);
	
	ros::spin();
}

