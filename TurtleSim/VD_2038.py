#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist

def Revolve():
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('node_turtle_revolve', anonymous=True)
    V=Twist()   #V is Velocity
    rate = rospy.Rate(10) # 10hz
    V.linear.x=1
    V.linear.y=0
    V.linear.z=0
    V.angular.x=0
    V.angular.y=0
    V.angular.z=1

    
    while not rospy.is_shutdown():
        
        pub.publish(V)
        
       
        
        
if __name__ == '__main__':
    try:
        Revolve()
    except rospy.ROSInterruptException:
        pass
