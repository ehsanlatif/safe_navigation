#!/usr/bin/env python


#Code is inspired by http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals (written in C++).
#TurtleBot must have minimal.launch & amcl_demo.launch running prior to starting this script.

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from network_analysis.msg import WirelessLink
import std_msgs.msg
import subprocess
import numpy as np

SIMILARITY_THRESHOLD = 0.1
SAFETY_OFFSET = 5    # number of pixels away from the wall the robot should remain


class PositionNode:
    def __init__(self, x, y, theta=0.0, parent=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.parent = parent
        # f(n) = h(n) + g(n)
        self.f = 0
        self.h = 0
        self.g = 0
        self.origin = (-10,-10)
        self.resolution = 0.05

    def euclidean_distance(self, goal):
        return math.sqrt(math.pow((goal.x-self.x),2) + math.pow((goal.y-self.y),2))

    def apply_move(self, move):
        theta_new = self.theta + move[1]
        x_new = self.x + math.cos(theta_new) * move[0]    # d.cos(theta)
        y_new = self.y + math.sin(theta_new) * move[0]  # d.sin(theta)
        return PositionNode(x_new, y_new, theta_new)
    def is_allowed(self,state,grid_map):
        was_error = False
        i, j = state[0],state[1]
        side = 7
        # int(math.floor((max(2, 3) / self.resolution) / 2))
        try:
            for s_i in range(i,i+side):
                for s_j in range(j, j+side):
                    cell = grid_map[s_i][s_j]
                    if cell == 100: # or cell == -1:
                        return False
        except IndexError as e:
            # rospy.loginfo("Indices are out of range")
            was_error = True
        return True and not was_error
    def is_move_valid(self, grid_map, move):
        goal = self.apply_move(move)
        # convert goal coordinates to pixel coordinates before checking this
        goal_pixel = self.world_to_pixel((goal.x, goal.y), self.origin,self.resolution)
        return self.is_allowed(goal_pixel,grid_map)
    def world_to_pixel(self,pos):
        pixel_points = [0,0]
        pixel_points[0] = int((pos[0] - self.origin[0]) / self.resolution)
        pixel_points[1] = int((pos[1] - self.origin[1]) / self.resolution)
        return pixel_points
    def set_origin_resolution(self,origin,resolution):
        self.origin = origin
        self.resolution = resolution
    
    def is_valid(self, grid_map):
        """
        Return true if the location on the map is valid, ie, in obstacle free zone
        """
        goal_pixel = self.world_to_pixel((self.x, self.y),self.origin, self.resolution)
        if grid_map[goal_pixel[0]][goal_pixel[1]] != -1:
            return True
        return False

    def is_similar(self, other):
        """
        Return true if other node is in similar position as current node
        """
        return self.euclidean_distance(other) <= SIMILARITY_THRESHOLD

class GoForwardAvoid():


    msg = WirelessLink()
    pos = PositionNode(0,0,0)
    scan_data = []
    map_grid=[]
    thr1 = 1

    def find_corridor_center(self):
        distance_right = min(self.scan_data.ranges[157:177])
        distance_left = min(self.scan_data.ranges[452:472])
        right_wall_start=self.pos.world_to_pixel((self.pos.x,self.pos.y-distance_right))
        left_wall_start=self.pos.world_to_pixel((self.pos.x,self.pos.y+distance_left))
        right_Wall_detected = True
        for i in range(right_wall_start[0]-2,right_wall_start[0]+2):
            for j in range(right_wall_start[1]-10,right_wall_start[1]+10):
                if self.map_grid[i][j] != 10:
                    right_Wall_detected  = False
        left_Wall_detected = True
        for i in range(left_wall_start[0]-2,left_wall_start[0]+2):
            for j in range(left_wall_start[1]-10,left_wall_start[1]+10):
                if self.map_grid[i][j] != 10:
                    left_Wall_detected  = False
        if right_Wall_detected and left_Wall_detected:
            wall_center_x= self.pos.x + 0.5
            wall_center_y = ((self.pos.y-distance_right)+(self.pos.y+distance_left))/2
            return (wall_center_x,wall_center_y)
        return (self.pos.x,self.pos.y)





    def mapCB(self,data):
        self.map_grid = np.array(data.data)
        # print(data.info)
        self.map_grid = self.map_grid.reshape(data.info.width,data.info.height,order='F')

    def metaCB(self,data):
        self.pos.set_origin_resolution(data.origin,data.resolution)


    def get_rssi_from_os(self,interface_name):
        self.msg.rssi = -256 # default rssi value when there is no link
        self.msg.lqi = 0 # default lqi value when there is no link
        self.msg.noise = -256 # default noise value when there is no link
        self.msg.iface = interface_name
        self.msg.status = False

        try:
            cmd ="cat /proc/net/wireless | grep " + interface_name
            cmd_output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0]#os.popen(cmd)
            # print(cmd_output)
            # cmd_output = f.read()
            ans = cmd_output.split()

            self.msg.rssi = int(float(ans[3]))
            self.msg.lqi = float(ans[2])
            self.msg.noise = int(ans[4])
            self.msg.iface = interface_name
            self.msg.status = True

        except:
            rospy.loginfo("The specified interface %s does not exist or is disconnected. Please check",interface_name)
            pass

    def odometryCb(self, msg):
       # print msg.pose.pose
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_,_,yaw)=euler_from_quaternion (orientation_list)
        self.pos=PositionNode(msg.pose.pose.position.x,msg.pose.pose.position.y,yaw)
        
    def scanCB(self,dt):
        self.scan_data=dt
        #print len(dt.ranges)

    def radians(self,degree):
        return (degree * math.pi / 180)

    def __init__(self):
        rospy.init_node('nav_test', anonymous=False) 
        rospy.Subscriber('odom',Odometry,self.odometryCb)
        rospy.Subscriber("/scan", LaserScan, self.scanCB) 
        rospy.Subscriber("/map", OccupancyGrid, self.mapCB)
        rospy.Subscriber("/map_metadata", MapMetaData, self.metaCB)



        #what to do if shut down (e.g. ctrl + C or failure)
        rospy.on_shutdown(self.shutdown)

    
        #tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("wait for the action server to come up")
        #allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))
        goal_acheived = True
        index = 0
        interfacename= "wlp2s0"
        while(1):
            self.get_rssi_from_os(interfacename)
            if self.msg > -70:
                next_goal = self.find_corridor_center()                 
                print next_goal
                #we'll send a goal to the robot to move 3 meters forward
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'base_link'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = next_goal[0]
                goal.target_pose.pose.position.y = next_goal[1]
                goal.target_pose.pose.orientation.w = 1 #go forward

               #start moving
                self.move_base.send_goal(goal)
                goal_acheived = False

                #allow TurtleBot up to 60 seconds to complete task
                success = self.move_base.wait_for_result(rospy.Duration(60)) 


                if not success:
                    self.move_base.cancel_goal()
                    rospy.loginfo("The base failed to move forward 0.5 meters for some reason")
                else:
                    # We made it!
                    state = self.move_base.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Hooray, the base moved 0.5 meters forward")
                        goal_acheived = True
                index+=1




    def shutdown(self):
        rospy.loginfo("Stop")


if __name__ == '__main__':
    try:
        GoForwardAvoid()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
