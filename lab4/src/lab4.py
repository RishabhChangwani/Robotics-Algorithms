#!/usr/bin/env python3
import math
from heapq import *

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


orientation = None
position = None
# Map 
# Grid representation with 1s and 0s 
# 1 indicating an obstacle in that cell and 0 representing an empty cell.
map = np.array([
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1],
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
       [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
       [0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
       [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0,1],
       [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0,1],
       [0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,1],
       [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1],
       [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1],
       [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,1],
       [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,1],
       [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1],
       [0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,1],
       [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,1],
       [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,1],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1,1]])


# Euclidian Distance
def eucledian_distance(point1,point2):
    distance = math.sqrt(((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2))
    return distance


class main():
    # Get Parameters from the launch file.
    def get_parameters(self):
        goalx_param = rospy.search_param('goalx')
        goaly_param = rospy.search_param('goaly')
        goalx = rospy.get_param(goalx_param)
        goaly = rospy.get_param(goaly_param)
        goalx_int = int(goalx)
        goaly_int = int(goaly)
        return (goalx_int,goaly_int)

    def _path_planning(self,pathf,arguemts_for_path_planning):
        if (done == False):
            turn = Twist()
            path=arguemts_for_path_planning['astar_path']
            nextIndex=arguemts_for_path_planning['nextindex']
            pub = arguemts_for_path_planning['pub']
            translation=pathf.pose.pose.position
            rotation=pathf.pose.pose.orientation
            robot_orientation=tf.transformations.euler_from_quaternion([rotation.x,rotation.y,rotation.z,rotation.w])[2]
            robotpositionx=translation.x
            robotpositiony=translation.y
            if(round(abs(robotpositionx - goalx),1) <= 0.6 and round(abs(goaly  -robotpositiony),1) <= 0.8):
                arguemts_for_path_planning["done"]=True
                arguemts_for_path_planning["Rotation"]=False
                goal_reached()
            else:
                nextx=path[nextIndex][0]-9+0.9
                nexty=10.5-path[nextIndex][1]-1
                diff_x = nextx-robotpositionx
                diff_y = nexty-robotpositiony
                theta=math.atan2(diff_y,diff_x)
                print(theta)
                err = theta - robot_orientation
            if arguemts_for_path_planning['Rotation']:
                if math.fabs(err) > 0.1:
                    # rotate
                    if err < 0:
                        err += 6
                    elif err > 6:
                        err -= 6
                    if err > 6:
                        turn.angular.z = -0.07
                        pub.publish(turn)        #self._do_twist(angular=[0,0,-0.7]))
                    else:
                        turn.angular.z = 0.7
                        pub.publish(turn)
                else:
                    arguemts_for_path_planning["Rotation"]=False
            else:
                err=math.sqrt((diff_x)**2+(diff_y)**2)
                if err> 0.5:
                    turn.linear.x = 0.7
                    pub.publish(turn)
                else:
                    arguemts_for_path_planning["Rotation"]=True
                    if nextIndex+1<len(path):
                        arguemts_for_path_planning['nextindex']+=1
                    else:
                        arguemts_for_path_planning['done']=True


def astar(array, start, goal,Rotation):
    while (Rotation):
        list1,traversed,list2 = set(),{},[]
        end = eucledian_distance(start, goal)
        g,f = {start:0},{start: end}
        heappush(list2, (f[start], start))
        while list2:

            present = heappop(list2)[1]
            if present == goal:
                previous_path = []
                new_path = []
                while present in traversed:
                    previous_path.append(present)
                    present = traversed[present]
                previous_path.reverse()
                for point in previous_path:
                    y = math.floor(point[1])
                    x = math.floor(point[0])
                    new_path.append((x,y))
                return new_path

            list1.add(present)
            x,y = present
            list = []
            R = [0,1,-1]
            C = [0,1,-1]

            for u in R:
                for v in C:
                    new = (u,v)
                    if (new[0],new[1]) != (0,0):
                        list.append(new)

            for u, v in list:
                neighbouring_cells = x + int(u), y + int(v)
                updated_g = g[present] + eucledian_distance(present, neighbouring_cells)
                if(0 <= neighbouring_cells[0] < array.shape[1]):
                    C1 = True
                else:
                    C1 = False
                if(0 <= neighbouring_cells[1] < array.shape[0]):
                    C2 = True
                else:
                    C2 = False
                C3 = array[neighbouring_cells[1]][neighbouring_cells[0]]
                if C1:
                    if C2:
                        if C3:
                            continue
                elif (C1 == 0):
                    continue
                elif C1:
                    if (C2 == 0):
                        continue

                if neighbouring_cells in list1:
                    if updated_g >= g.get(neighbouring_cells, 0):
                        continue

                NC = [u[1]for u in list2]
                if  updated_g < g.get(neighbouring_cells, 0) or neighbouring_cells not in NC:
                    traversed[neighbouring_cells] = present
                    g[neighbouring_cells] = updated_g
                    f[neighbouring_cells] = updated_g + eucledian_distance(neighbouring_cells, goal)
                    heappush(list2, (f[neighbouring_cells], neighbouring_cells))

def goal_reached():
    done =  True
    Rotation = False
    print('Goal Reached')
    rospy.spin()
if __name__ == '__main__':
    rospy.init_node("robot", anonymous=False)
    robot=main()
    goalx,goaly=rospy.get_param("/goalx"),rospy.get_param("/goaly")
    start = (1,12)
    goal = (int(goalx+8.5),int(10-goaly))
    arguemts_for_path_planning = {}
    done = False
    Rotation = True
    nextindex = 0
    pub  = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
    afstar_path = {}
    astar_path = astar(map, start,goal,Rotation)
    arguemts_for_path_planning['done'] = done
    arguemts_for_path_planning['astar_path'] = astar_path
    arguemts_for_path_planning['nextindex'] = nextindex
    arguemts_for_path_planning['Rotation'] = Rotation
    arguemts_for_path_planning['pub'] = pub
    robot_pos_pub = rospy.Subscriber("/base_pose_ground_truth", Odometry,robot._path_planning,arguemts_for_path_planning)#,args)
    while not rospy.is_shutdown():
        if done:

            if rospy.has_param("/goalx") and rospy.has_param("/goaly"):
                    goalx,goaly=rospy.get_param("/goalx"),rospy.get_param("/goaly")
                    rospy.delete_param("/goalx")
                    rospy.delete_param("/goaly")
                    nstart = goal
                    goal = (round(goalx+9),round(10-goaly))
                    astar_path = astar(map, nstart, goal,Rotation)
                    Rotation = True
                    nextindex = 0
                    done = False
        rate = rospy.Rate(2)
        rospy.sleep(1)
