#!/usr/bin/env python

from vitarana_drone.msg import *
#from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import rospy
import time
import tf
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')  

       
        # This is the setpoint that will be published to the drone_command in the range from 1000 to 2000
        self.pub_cmd = [0.0, 0.0, 0.0, 0, 0, 0, 0, 0]
        
        # pid output values. [latitude, longitude, altitude]
        self.out= [0.0, 0.0, 0.0]
        self.setpoint_position_gps = [0.0, 0.0, 0.0]        #pseudo final destination
        self.changed_destination = [0.0, 0.0, 0.0]
        # setpoint of final destiantion
        self.final_destination = [19.0007046575, 71.9998955286, 22.1599967919]
        #self.final_destination = [19.000, 72.0000, 8.44]
        self.setpoint_position_gps[0] = self.final_destination[0]
        self.setpoint_position_gps[1] = self.final_destination[1]
        self.setpoint_position_gps[2] = self.final_destination[2]
        self.Kp = [10000000, 12000000, 700.1]
        self.Ki = [0.1, 0.1, 3]
        self.Kd = [720000000, 720000000, 8250]
        self.P = [0, 0, 0]
        self.I = [0, 0, 0]
        self.D = [0, 0, 0]
        self.min_values = [1000, 1000, 1000, 1000]
        self.max_values = [2000, 2000, 2000, 2000] 
        self.max_valuesI = [500, 500, 2000, 2000]
        # variables current values of gps
        self.current_pos_gps = [0.0, 0.0, 0.0]
        self.error= [0.0, 0.0, 0.0]
        self.prev_error= [0.0, 0.0, 0.0]
        
        self.obstacle = [0.0, 0.0, 0.0, 0.0, 0.0]
        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.075  # in seconds
        self.a = False
        self.f = 0
        self.flag = False
        self.t = 1
        self.z = 0
        self.b = False
        self.i = True
        self.h = True
        self.new_height = 22.1599967919
        self.g = 15                 #Distance from obstacle
        self.u = 0.0000005          #value by which it increases gps_coordinates of set_point
        self.v = 0.00000001         #value by which it decreases gps_coordinates of set_point
        # Publishing /edrone/drone_command
        self.cmd_pub = rospy.Publisher('/edrone/drone_command', edrone_cmd, queue_size=1)
    
        # Subscribing to /edrone/gps
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.Sensor_info_top)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.Sensor_info_bottom)
        rospy.Subscriber('/edrone/gripper_check', String, self.gripper_check)
        rospy.Subscriber('/edrone/gps_coordinates', prop_speed, self.change_dest)       #For getting coordinates from qr detector

        #services
        self.grip = rospy.ServiceProxy('/edrone/activate_gripper' , Gripper)

    def gps_callback(self, msg):                #callback for live location
        self.current_pos_gps[0] = msg.latitude
        self.current_pos_gps[1] = msg.longitude
        self.current_pos_gps[2] = msg.altitude

    def Sensor_info_top(self, msg):
        self.obstacle[0] = msg.ranges[4]
        self.obstacle[1] = msg.ranges[1]
        self.obstacle[2] = msg.ranges[2]
        self.obstacle[3] = msg.ranges[3]

    def Sensor_info_bottom(self, msg):
        self.obstacle[4] = msg.ranges[0]

    def IsSetPoint(self):           #function for checking if drone reached destination
        if abs(self.final_destination[0] - self.current_pos_gps[0]) < 0.0000015 and abs(self.final_destination[1] - self.current_pos_gps[1]) < 0.0000015 :
            return True
        else:
            return False

    def AlmostSetPoint(self):       #function for checking if drone almost reached destination
        if abs(self.final_destination[0] - self.current_pos_gps[0]) < 0.0001 and abs(self.final_destination[1] - self.current_pos_gps[1]) < 0.0001 :
            return True
        else:
            return False
       
    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude)

    def long_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude)

    def gripper_check(self, msg):   #function to check if drone is aligned
        if msg.data == 'True' or msg.data == 'true':
            self.flag = True
        else:
            self.flag = False
        
    def change_dest(self, msg):     #callback function for taking value of qr code
        self.changed_destination[0] = msg.prop1
        self.changed_destination[1] = msg.prop2
        self.changed_destination[2] = msg.prop3
        if self.changed_destination[1] != 0:
            self.b = True           #flag to determine if qr code was read
        
    def pick_box(self):
        if self.IsSetPoint() and self.current_pos_gps[2] - self.final_destination[2] < 0.3 and self.flag and self.a and self.b and self.h :
            s = self.grip(self.i)
            if s:                   #if drone picked/dropped the box
                for x in range(2):  #changing final destination's lattitue and longitude
                    self.final_destination[x] = self.changed_destination[x]
                    self.setpoint_position_gps[x] = self.changed_destination[x]
                self.new_height = self.changed_destination[2]           #saving the altitude in a new variable
                if self.i:          #changing value of i for shifting from picking to dropping
                    self.i = False
                else:
                    self.h = False  #for stopping the gripper service to be called multiple times
        
    
    def path_planner(self):

        for x in range(5):        #ignoring small variations in sensors
            if self.obstacle[x]<1:
                self.obstacle[x]=0
        
        p1 = 0
        p2 = 0
        
        #determining the direction to proceed(ie left or right if there is an obstacle in front)
        if self.error[0]>0:     
            p1 = -1
        elif self.error[0]<0:
            p1 = 1
        if self.error[1]>0:
            p2 = -1
        elif self.error[1]<0:
            p2 = 1

        # detecing obstacle. if its less than self.g value away, moving pseudo final destination accordingly so that the drone
        # avoids the obstacle and goes right or left, depending on which direction the final destination is (bug2 algorithm)

        if self.obstacle[3]<self.g:
            if abs(self.lat_to_x(self.error[0])) > self.obstacle[3] :
                self.setpoint_position_gps[1] = self.setpoint_position_gps[1] +  self.u*p1
                 
        elif self.AlmostSetPoint() and self.obstacle[1]>self.g:
                self.setpoint_position_gps[1] = self.final_destination[1]
        elif self.setpoint_position_gps[1] != self.final_destination[1] and self.obstacle[1]>self.g:
                self.setpoint_position_gps[1] = self.setpoint_position_gps[1] -  self.v*p1
                

        if self.obstacle[1]<self.g :
            if abs(self.lat_to_x(self.error[0])) > self.obstacle[1]  :
                self.setpoint_position_gps[1] = self.setpoint_position_gps[1] +  self.u*p1 
                
        elif self.AlmostSetPoint() and self.obstacle[3]>self.g:
                self.setpoint_position_gps[1] = self.final_destination[1]
        elif self.setpoint_position_gps[1] != self.final_destination[1] and self.obstacle[3]>self.g:
                self.setpoint_position_gps[1] = self.setpoint_position_gps[1] -  self.v*p1
                
        
        if self.obstacle[2]<5 :
            if abs(self.long_to_y(self.error[1])) > self.obstacle[2] :
                self.setpoint_position_gps[0] = self.setpoint_position_gps[0] +  self.u*p2
        elif self.AlmostSetPoint():
                self.setpoint_position_gps[0] =self.final_destination[0]
        elif self.setpoint_position_gps[0] != self.final_destination[0] : 
                self.setpoint_position_gps[0] = self.setpoint_position_gps[0] -  self.v*p2

        if self.obstacle[0]<5 :
            if abs(self.long_to_y(self.error[1])) > self.obstacle[0] :
                self.setpoint_position_gps[0] = self.setpoint_position_gps[0] +  self.u*p2 
        elif self.AlmostSetPoint():
                self.setpoint_position_gps[0] =self.final_destination[0]
        elif self.setpoint_position_gps[0] != self.final_destination[0] :
                self.setpoint_position_gps[0] = self.setpoint_position_gps[0] -  self.v*p2

        if self.setpoint_position_gps[2] - self.final_destination[2] < 3.5 and not self.IsSetPoint() : #and not self.b or self.setpoint_position_gps[2] - self.final_destination[2] < 5 and self.b and not self.IsSetPoint():
            self.setpoint_position_gps[2] = self.current_pos_gps[2] + 0.05
        elif self.IsSetPoint() and self.setpoint_position_gps[2] != self.new_height: #self.final_destination[2]  :
            self.final_destination[2] = self.new_height
            self.setpoint_position_gps[2] = self.current_pos_gps[2] - 0.025
            self.a = True

        
    def pid(self):

        for i in range(3):
            self.error[i] = self.setpoint_position_gps[i] - self.current_pos_gps[i]
        			
	    
        for i in range(3):                  #calculating I part of pid for each roll pitch throttle
            self.I[i] += self.Ki[i]* self.error[i]
        
        
        
        for x in range(3):                          #limiting I value
            if self.I[x] > self.max_valuesI[x]:
		        self.I[x] = self.max_valuesI[x]
            elif self.I[x] < self.max_valuesI[x]*-1:
	 	        self.I[x] = self.max_valuesI[x] * -1

        for i in range(3):                          #calculating P part of pid for each roll pitch throttle
            self.P[i]= self.Kp[i] * self.error[i]

        for i in range(3):                          #calculating D part of pid for each roll pitch throttle
            self.D[i]= self.Kd[i]* (self.error[i] - self.prev_error[i])

        for i in range(3):
	        self.out[i] = self.P[i]  + self.D[i] + self.I[i]        #calculating PID
        
        for i in range(3):
            self.prev_error[i] = self.error[i]                     #Storing error
        
        self.pub_cmd[0]= 1500 + self.out[0]
        self.pub_cmd[1]= 1500 + self.out[1] 
        self.pub_cmd[2]= 1500                       #Keeping the yaw value constant
        self.pub_cmd[3]= 1500 + self.out[2]                #Assigning throttle value
        
        
        for i in range(4):                          #Limiting the output
            if self.pub_cmd[i] > self.max_values[i] :
                self.pub_cmd[i] = self.max_values[i]
            elif self.pub_cmd[i] < self.min_values[i] :
                self.pub_cmd[i] = self.min_values[i]
        
        self.cmd_pub.publish(self.pub_cmd[0],self.pub_cmd[1],self.pub_cmd[2],self.pub_cmd[3],self.pub_cmd[4],self.pub_cmd[5],self.pub_cmd[6],self.pub_cmd[7])          #Publishing 
        
        #picking and dropping delivery
        """if self.IsSetPoint() and self.current_pos_gps[2] - self.final_destination[2] < 0.1 :
            if self.flag :
                y = False
                x = False
                self.flag = False
            if self.t%2 is 1:
                x = True
            y = self.pick_box(x)"""

        #self.pick_box()
    
    def flight(self):

        self.path_planner()     # changes pseudo final destination according to presence of obstacles and their positions
        self.pid()              # heads towards the pseudo final destination
        self.pick_box()         # if destination is reached, it picks or drops the box

        

if __name__ == '__main__':

    e_drone = Edrone()
    rospy.sleep(5)
    r = rospy.Rate(20)  
    while not rospy.is_shutdown():
        e_drone.flight()
        r.sleep()
        #e_drone.sample_time