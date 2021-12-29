#!/usr/bin/env python3

#from types import pointer
import numpy as np
#from numpy.lib.shapebase import column_stack
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
#from sensor_msgs.msg import PointCloud2
from laser_geometry import laser_geometry
#import laser geometry. laser geometry as lg
import sensor_msgs
import sensor_msgs.point_cloud2 as pc
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from random import randint
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
def callback_pointcloud(data):
	x= []
	y=[]
	array = []
	theta = []
	#index = []
	m = 0
	final_inliers = []
	points_to_rviz = []
	laser_proj=laser_geometry.LaserProjection()
	cloud=laser_proj.projectLaser(data)
	point_generator = pc.read_points(cloud)
	point_1=[]
	point_2=[]
	index = 0
	if not(all(i >= 3.0 for i in data.ranges)):
	# we can access a generator in a loop
		for point in point_generator:
			x.append(point[0]) #Storing all my x-coordinates
			y.append(point[1]) #storing all my y-coordinates
		
		#x=[round (num,4) for num in x]
		#y= [round(num,4) for num in y]
		#print(len(x))
		for l in range (3): #This loop number of lines, expecting maximum 3 lines
			x_points = []
			y_points = []
			final_x = []
			final_y= []
			final_inliers = []
			inliers_list = [] #creating an empty list of inliers
			inlier_counter = [] #creating an empty list to count number of inliers
			R1 = [] #store a copy of all random points
			R2 = [] #store a copy of all random points
			#print(len(y))
			#print(len(y))
			for k in range (20):
				r1 = randint(0, len(x)-1)
				r2 = randint(0, len(x)- 1)

				R1.append(r1)
				R2.append(r2)
				point_1 = []
				point_2 = []
				#index = 0
				if( r1 != r2):
					x1 = x[r1] #Getting a random x-coordinate
					x2 = x[r2] #Getting a random x-coordinate
					y1 = y[r1] ##Getting a random x-coordinate
					y2 = y[r2] #Getting a random y-coordinate
					if(x2 - x1 != 0): m = (y2 - y1)/(x2-x1) #Calculating slope of the line	
					c = y1 - m*x1;
					A = y2 - y1;
					B = x1 - x2;
					C = (x2 - x1)*c;
					list1 = [] #Creating a list to store all  the inliers for x1,y1 and x2,y2
					length = len(x)
					#print(len(x))
					
					# Taking distance of all data points from the line
					for i in range(length - 1):
						x0 = x[i]
						y0 = y[i]
						d = abs((A*x0 + B*y0 + C)/np.sqrt(A*A + B*B))
						if(d<0.001): #Setting a threshold distance
							#count++
							list1.append([x0,y0]) #Adding inliers to list1
					inliers_list.append(list1) #storing the list of inliers to another list (list<list>)
					inlier_counter.append(len(list1)) #Getting the number of inliers for x1,y1 and x2,y2
				
			index = inlier_counter.index(max(inlier_counter)) #Getting the index of the maximum inliers
			final_inliers = inliers_list[index] #Storing inliers of the index(maximum)
			
			for i in range(len(final_inliers) - 1):
				x_points.append(final_inliers[i][0]) #Storing all the x-coordinates of inlers
				#print(x_points)
				y_points.append(final_inliers[i][1]) #Storing all the y-coordintes of the inliers
				
			if(abs(m) < 1):
			
				final_x = x_points[x_points.index(max(x_points))]
				point_1.append(final_x)
				final_y = y_points[x_points.index(max(x_points))]
				point_1.append(final_y)
				final_x = x_points[x_points.index(min(x_points))]
				point_2.append(final_x)
				final_y = y_points[x_points.index(min(x_points))]
				point_2.append(final_y)
				array.append([point_1,point_2])
			elif(abs(m) > 1):
				final_x = x_points[y_points.index(max(y_points))]
				point_1.append(final_x)
				final_y = y_points[y_points.index(max(y_points))]
				point_1.append(final_y)
				final_x = x_points[y_points.index(min(y_points))]
				point_2.append(final_x)
				final_y = y_points[y_points.index(min(y_points))]
				point_2.append(final_y)
				array.append([point_1,point_2])
			
			#Removing all the inliers for the next line				
			x = [i for i in x if i not in x_points]
			y = [i for i in y if i not in y_points]
		
		#print(array)
		#Creating Markers for RVIZ
	else:

		array=[[[0,0],[0,0]] for _ in range(3)]
		
		# for a in array:
		# 	for b in a:
		# 		for c in b:
		# ###			c=0
	print(array)
#########################################################################################		
	marker = Marker()
	marker.header.frame_id ="base_link";
	marker.id = 1;
	#marker.header.stamp = ros::Time::now();
	marker.ns = "lines";
	marker.action = marker.ADD
	marker.pose.orientation.w = 0.01;
	marker.type = marker.LINE_LIST
	marker.scale.x = 0.1;
	marker.color.b = 1.0;
	marker.color.a = 1.0
	 
	point1 = Point() 
	point2 = Point()
	point3 = Point()
	point4 = Point()
	point5 = Point()
	point6 = Point()	
	
	#print((array))
	point1.x = array[0][0][0]
	point1.y = array [0][0][1]
	marker.points.append(point1)
	point2.x = array[0][1][0]
	point2.y = array [0][1][1]
	marker.points.append(point2)

	point3.x = array[1][0][0]
	point3.y = array[1][0][1]
	marker.points.append(point3)

	point4.x = array[1][1][0]
	point4.y = array[1][1][1]
	marker.points.append(point4)

	point5.x = array[2][0][0]
	point5.y = array[2][0][1]
	marker.points.append(point5)

	point6.x = array[2][1][0]
	point6.y = array[2][1][1]
	marker.points.append(point6)
		
	LinePoint_pub.publish(marker)
	marker.points = 0			
					
####################################################################################				
		
	
if __name__ == '__main__':
	rospy.init_node("RANSAC")
	LinePoint_pub = rospy.Publisher('visualization_marker',Marker,queue_size=100)
	rospy.Subscriber('/base_scan' ,LaserScan, callback_pointcloud)
	rospy.spin()
