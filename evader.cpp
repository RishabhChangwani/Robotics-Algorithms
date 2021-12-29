#include<ros/ros.h>
#include<std_msgs/String.h>
#include<sensor_msgs/LaserScan.h>
#include<ackermann_msgs/AckermannDriveStamped.h>
#include<ackermann_msgs/AckermannDrive.h>

void chatterCallBack(const sensor_msgs::LaserScan msg)
{
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	
	//creating the publisher object
	ros::Publisher pub1 =nh.advertise<ackermann_msgs::AckermannDriveStamped>("/evader_drive",10);
	
	//creating objects to publoish data to AckermannDriveStamped visa AckermannDrive
	ackermann_msgs::AckermannDrive obj;
	ackermann_msgs::AckermannDriveStamped obj1;
	
	float dist[131];
	obj.steering_angle = 0.0;
	
	//Calculating Distance from the side to detect a turn
	for(int i = 680, j =0; i<=810, j<= 130; i++,j++)
	{
		dist[j] = msg.ranges[i];
	}
	float *max = std::max_element(std::begin(dist), std::end(dist));
	
	if(*max>3.5)
	{	
		//Turn Detected
		obj.steering_angle = 0.4;
		obj.speed = 2.0;
		obj1.drive = obj;
		pub1.publish(obj1);
		ros::Duration(0.2).sleep();
		goto label1;
	}
	else
	{
		//Drive in a straight line
		label1: obj.steering_angle = 0.0;
		obj.speed= 2.0;
		obj1.drive = obj;
		pub1.publish(obj1);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "evader");
	ros::NodeHandle nh;
	ros::Publisher pub1 = nh.advertise<ackermann_msgs::AckermannDriveStamped>("evader_drive",1);
	ros::Subscriber sub = nh.subscribe("/scan", 10, chatterCallBack);
	
	ros::spin();
}
