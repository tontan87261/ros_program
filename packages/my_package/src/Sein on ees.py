#!/usr/bin/env python3

import os
import rospy
from time import sleep
import smbus2

from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, ColorRGBA

from smbus2 import SMBus

from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped


from sensor_msgs.msg import Range



speed = WheelsCmdStamped()


class MyPublisherNode(DTROS):




    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.range = 1
        self.right = 0
        self.left = 0
        self.avoiding = False

        self.pub = rospy.Publisher('bestestduckiebot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.tof = rospy.Subscriber('/bestestduckiebot/front_center_tof_driver_node/range', Range, self.callback)
        self.rwheel = rospy.Subscriber('/bestestduckiebot/right_wheel_encoder_node/tick', WheelEncoderStamped ,self.rightwheel)
        self.lwheel = rospy.Subscriber('/bestestduckiebot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.leftwheel)
        

        

    def on_shutdown(self):
                
        speed.vel_right = 0
        speed.vel_left = 0
        self.pub.publish(speed)

        rospy.on_shutdown()

    def callback(self, data):
        self.range = data.range
        
    
    def rightwheel(self, data):
        self.right = data.data
        

    def leftwheel(self, data):
        self.left = data.data
        

    def go_around(self):

        print("Going around!")
        sleep(1)
    
        right_start = self.right
        #print(":::::::::::::::: %s", right_start)
        parem = self.right - right_start
        print(parem)
        left_start = self.left
        vasak = self.left - left_start
        #print(":::::::::::::::: %s", left_start)
        print(vasak)
        while parem <= 60:
            print("Turning left:")
            print("vasak: ", vasak)
            print("parem: ", parem)
            speed.vel_right = float(0.07)
            speed.vel_left = float(0)
            self.pub.publish(speed)
            parem = self.right - right_start
            


        if parem > 60:
            speed.vel_right = float(0)
            speed.vel_left = float(0)
            self.pub.publish(speed)
            sleep(1)

            while vasak < 200:
                print("Driving forward:")
                print("vasak: ", vasak)
                print("parem: ", parem)
                speed.vel_right = float(0.5)
                speed.vel_left = float(0.5)
                self.pub.publish(speed)
                vasak = self.left - left_start
                print(vasak)
                
            if vasak >= 200:
                vasak = self.left - left_start
                print("Stopping:")
                print("vasak: ", vasak)
                print("parem: ", parem)
                speed.vel_right = float(0)
                speed.vel_left = float(0)
                self.pub.publish(speed)
                sleep(1)

                while vasak < 350:
                    print("Turning left:")
                    print("vasak: ", vasak)
                    print("parem: ", parem)
                    vasak = self.left - left_start
                    speed.vel_right = float(0)
                    speed.vel_left = float(0.07)
                    self.pub.publish(speed)

                if vasak >= 350:
                    print("Stopping:")
                    print("vasak: ", vasak)
                    print("parem: ", parem)
                    speed.vel_right = float(0)
                    speed.vel_left = float(0)
                    self.pub.publish(speed)
                    sleep(1)

            while vasak >= 350 and vasak < 600:
                print("Driving forward:")
                print("vasak: ", vasak)
                print("parem: ", parem)
                speed.vel_right = float(0.5)
                speed.vel_left = float(0.5)
                self.pub.publish(speed)
                vasak = self.left - left_start
                print(vasak)

            if vasak >= 600:
                print("Stopping:")
                print("vasak: ", vasak)
                print("parem: ", parem)
                speed.vel_right = float(0)
                speed.vel_left = float(0)
                self.pub.publish(speed)


        self.avoiding = False
        
    
        



        
    def run(self):
        

    # publish message every 1 second
        rate = rospy.Rate(30) # 1Hz
        
        
        bus = SMBus(1)
        
    
        while not rospy.is_shutdown() and not self.avoiding:
                
            

            try:
                temp = bus.read_byte_data(62, 17)
            except:

                print("ei saa andmeid lugeda")
                    
            message = str(temp)

            #rospy.loginfo("Line follower: %s" % message)
            

                
            
                    
            
            if self.range < 0.3:
                print("A wall appeared!")
                speed.vel_right = float(0)
                speed.vel_left = float(0)
                self.pub.publish(speed)
                self.avoiding = True
                self.go_around()
                if self.avoiding == False:
                    break
                
                
                
            else:
                speed.vel_right = float(0.2)
                speed.vel_left = float(0.2)

                self.pub.publish(speed)
            
            rate.sleep()
        
            #else:
            #speed.vel_left = float(0)
            #speed.vel_right = float(0)
            #self.pub.publish(speed)




if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep spinning

    rospy.spin()