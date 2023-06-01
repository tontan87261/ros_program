#!/usr/bin/env python3


import rospy
from smbus2 import SMBus
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

class Line_follower(DTROS):
    def __init__(self, node_name):
        super(Line_follower, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.pub = rospy.Publisher('line_follower', String, queue_size=1)
        self.theta_ref = 0
        self.bus = SMBus(1)


       
    def run (self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
        # publish message every 1 second
            try:
                temp = self.bus.read_byte_data(62, 17)
                self.theta_ref = bin(temp)[2:].zfill(8)
            except:
                print("Line follower ei saa infot")

            self.pub.publish(self.theta_ref)

            rate.sleep()
    
if __name__ == '__main__':
    #Teeb nodi
    node = Line_follower(node_name='line_follower')
    #Jooksutamis node
    node.run()
    #Jääb kordama
    rospy.spin()
