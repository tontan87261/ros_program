#!/usr/bin/env python3

import os
import rospy
import numpy as np
import smbus2
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
from geometry_msgs.msg import Pose  

class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.range = 1
        self.right = 0
        self.left = 0
        self.pub = rospy.Publisher('left_wheel_rotation', Pose, queue_size = 1)
        self.pub2 = rospy.Publisher('left_wheel_travel', Pose, queue_size = 1)
        self.pub3 = rospy.Publisher('right_wheel', Pose, queue_size = 1)
        self.rwheel = rospy.Subscriber('/tera/right_wheel_encoder_node/tick', WheelEncoderStamped ,self.rightwheel)
        self.lwheel = rospy.Subscriber('/tera/left_wheel_encoder_node/tick', WheelEncoderStamped, self.leftwheel)

        self.ticks_left = 0
        self.prev_tick_left = self.ticks_left
        self.ticks_right = 0
        self.prev_tick_right = self.ticks_right
        self.rotation_wheel_left = 0
        self.rotation_wheel_right = 0
        self.delta_ticks_left = 0
        self.delta_ticks_right = 0
        #self.right = 0
        #self.left = 0
        self.timeL = 0
        self.baseline_wheel2wheel = 0.1 #Laius kahe ratta vahel, meetrites
        self.lastCall = 0

        self.position = Pose()
        self.right_pos = Pose()

        self.initial = 0
        self.leftinit = 0
        self.gotPose = False


    def rightwheel(self, data):
        self.right = data.data
        self.gotPose = True
        #rospy.loginfo("Parem ratas: %s", data.data)
    def leftwheel(self, data):
        self.left = data.data
        self.timeL = data.header.seq
        self.gotPose = True
        #rospy.loginfo("Vasak ratas: %s", data.data)
    
        

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(25) # 1Hz  
        while not rospy.is_shutdown():

#-------------------------------------- ODUMEETRIA ----------------------------------------------#
            
            if self.initial < 1 and self.gotPose:
                self.prev_tick_left = self.left
                self.prev_tick_right = self.right
                self.initial = self.initial + 1


            #print("prev tick left: ", self.prev_tick_left)

            #rospy.loginfo("Line follower: %s" % message)
            N_tot = 135 # total number of ticks per revolution
            alpha = 2 * np.pi / N_tot # wheel rotation per tick in radians
            #print(f"The angular resolution of our encoders is: {np.rad2deg(alpha)} degrees")
            self.ticks_right = self.right
            self.ticks_left = self.left
            self.delta_ticks_left = self.ticks_left-self.prev_tick_left # delta ticks of left wheel
            self.delta_ticks_right = self.ticks_right-self.prev_tick_right # delta ticks of right wheel
            self.rotation_wheel_left = alpha * self.delta_ticks_left # total rotation of left wheel
            self.rotation_wheel_right = alpha * self.delta_ticks_right # total rotation of right wheel
            #print(f"The left wheel rotated: {np.rad2deg(self.rotation_wheel_left)} degrees")
            #print(f"The right wheel rotated: {np.rad2deg(self.rotation_wheel_right)} degrees")
            R = 0.0345           # insert value measured by ruler, in *meters*
            d_left = R * self.rotation_wheel_left
            d_right = R * self.rotation_wheel_right
            #print(f"The left wheel travelled: {d_left} meters")
            #print(f"The right wheel travelled: {d_right} meters")
            # How much has the robot rotated?
            Delta_Theta = (d_right-d_left)/self.baseline_wheel2wheel # expressed in radians
            #print(f"The robot has rotated: {np.rad2deg(Delta_Theta)} degrees")
            #self.prev_tick_left = self.ticks_left
            #self.prev_tick_right = self.ticks_right
            #print("Left wheel time: ", self.timeL)
            #Delta T arvutamine


            self.delta_t = self.timeL - self.lastCall + 3
            #print("timeL: ", self.timeL)


            self.lastCall = self.timeL
            #print("DELTA T : ", self.delta_t)

            #Publishin saadud tulemused:
            self.position.position.x = np.rad2deg(self.rotation_wheel_left) #Vasaku ratta pöörlemine kraadides
            self.pub.publish(self.position)
            self.position.position.y = d_left       #Vasaku ratta läbitud vahemaa
            self.position.position.z = self.delta_t
            #print("deltaT: ", self.delta_t)
            self.pub2.publish(self.position)

            self.right_pos.position.x = np.rad2deg(self.rotation_wheel_right) #Parema ratta pöörlemine kraadides
            self.right_pos.position.y = d_right     #Parema ratta läbitud vahemaa
            #self.right_pos.position.z = True
            self.pub3.publish(self.right_pos)
            rate.sleep()

                
if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='Odomeetria')
    # run node
    node.run()
    # keep spinning
    
    rospy.spin()