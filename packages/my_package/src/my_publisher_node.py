#!/usr/bin/env python3

#-------------------------------------- IMPORTIMISED ----------------------------------------------#
import rospy
import numpy as np
import time
from time import sleep
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose
import PID_controller
import Line_follower_output

import board
import adafruit_mpu6050



#-------------------------------------- KOODI PEAMINE FUNKTSIONAALSUS ----------------------------------------------#

speed = WheelsCmdStamped()
avoiding = False #Kas robot on mööda sõitmas takistusest




class MyPublisherNode(DTROS): #Klass

    def __init__(self, node_name):
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.bus = SMBus(1)
        


        self.start_vel = 0.25  #Roboti algne kiirus
        self.range = 1
        self.right = 0
        self.left = 0
        self.timeL = 0
        self.timeR = 0

        #Info vastu võtmine ja saatmine
        self.sub2 = rospy.Subscriber('left_wheel_travel', Pose, self.left_odom)
        self.sub3 = rospy.Subscriber('right_wheel', Pose, self.right_odom)
        self.sub4 = rospy.Subscriber('line_follower', String, self.line_follower)
    

        self.pub = rospy.Publisher('bestestduckiebot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.tof = rospy.Subscriber('/bestestduckiebot/front_center_tof_driver_node/range', Range, self.callback)
        self.rwheel = rospy.Subscriber('/bestestduckiebot/right_wheel_encoder_node/tick', WheelEncoderStamped ,self.rightwheel)
        self.lwheel = rospy.Subscriber('/bestestduckiebot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.leftwheel)

        
        self.ticks_left = 0
        self.prev_tick_left = 0
        self.ticks_right = 0
        self.prev_tick_right = 0
        self.rotation_wheel_left = 0
        self.rotation_wheel_right = 0
        self.delta_ticks_left = 0
        self.delta_ticks_right = 0
        self.baseline_wheel2wheel = 0.1 #Laius kahe ratta vahel, meetrites
        self.x_curr = 0
        self.y_curr = 0
        self.theta_curr = 0
        self.prev_int = 0
        self.prev_e = 0
        self.theta_ref = 0
    
        self.In_al = 0
        self.avoiding = False


        self.deltat = 0
        self.lastCall = 1

        self.position = 0
        self.lastturn = 0
        self.loops = 0
        self.short = 0

        self.prev_info = 0o00011000 #Jätab meelde eelmise joone lugeri info
        self.option = 0
        self.errorlist = []

        self.array = [-14,-12,-10,-6,6,10,12,14]

        #Joone järgimis anduri võimalikud väärtused mida on vaja sõitmiseks joonel
        self.rightvalues = [[0, 1, 1, 1, 1, 1, 1, 1], [0, 0, 1, 1, 1, 1, 1, 1], [0, 0, 0, 1, 1, 1, 1, 1], [0, 0, 0, 0, 1, 1, 1, 1], [0, 0, 0, 0, 0, 1, 1, 1], [0, 0, 0, 0, 0, 0, 1, 1], [0, 0, 0, 0, 0, 0, 0, 1]]
        self.leftvalues = [[1, 1, 1, 1, 1, 1, 1, 0], [1, 1, 1, 1, 1, 1, 0, 0], [1, 1, 1, 1, 1, 0, 0, 0], [1, 1, 1, 1, 0, 0, 0, 0], [1, 1, 1, 0, 0, 0, 0, 0], [1, 1, 0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0, 0, 0]]
        self.shortroute = [[1, 0, 0, 1, 0, 0, 0, 0], [1, 1, 0, 1, 1, 0, 0, 0], [1, 0, 1, 1, 0, 0, 0, 0], [0, 1, 1, 0, 1, 1, 0, 0]]
        self.turningpoint = [ [0, 0, 1, 1, 1, 1, 0, 0], [0, 1, 1, 1, 1, 0, 0, 0], [0, 0, 0, 1, 1, 1, 1, 0]]


        
        #Roboti sulgemis errori eemaldamine
    def on_shutdown(self):
        rospy.on_shutdown(self.shutdown)
    
        #Roboti sulgemisel saadetakse need käsud
    def shutdown(self):
        speed.vel_right = 0
        speed.vel_left = 0
        self.pub.publish(speed)

    def left_odom(self, data):
        self.input = data.position
        self.deltat = data.position.z
        #print("Vasaku ratta pöörlemine kraadides: ", self.input)
        
    
    def right_odom(self, data):
        self.odom_r = data.position.x       #Parema ratta pöörlemine kraadides
        #print("Parema ratta pöörlemine kraadides: ", self.odom_r)

    def line_follower(self, data):
        self.theta_ref = data.data
        #print("Eraldi failist line follower: ", self.theta_ref)    
        
    def callback(self, data):
        self.range = data.range
        #print("Kaugus: ", self.range)
        #rospy.loginfo("Kuulen: %s", data.range)
    def rightwheel(self, data):
        self.right = data.data
        #rospy.loginfo("Parem ratas: %s", data.data)
        #self.timeR = data.header.seq
    def leftwheel(self, data):
        self.left = data.data
        self.timeL = data.header.seq
        #rospy.loginfo("Vasak ratas: %s", data.header.seq)

    #def time_leftwheel(self, data):
        
    #def time_rightwheel(self, data):
        

#-------------------------------------- MÖÖDA SÕITMINE ----------------------------------------------#


    #Funktsioon mis seina tuvastusel hakkab seinast ümberpõiget sooritama ja otsib peale manööverdamist joone üles ning jätkab sõitu
    def go_around(self):
        print("Going around!")
        sleep(0.5) #Peale igat käsku on paus, parandab roboti liikumist
        right_start = self.right
        #print(":::::::::::::::: %s", right_start)
        parem = self.right - right_start
        print(parem)

        left_start = self.left
        vasak = self.left - left_start
        #print(":::::::::::::::: %s", left_start)
        print(vasak)

        while vasak <= 50:
            #print("Turning left:")
            #print("vasak: ", vasak)
            #print("parem: ", parem)
            speed.vel_right = 0.0
            speed.vel_left = 0.2
            self.pub.publish(speed)
            vasak = self.left - left_start
            #parem = self.right - right_start
        print("Paremale")
        print("Vasak ", vasak)

        if vasak > 50:
            speed.vel_right = 0.0
            speed.vel_left = 0.0
            self.pub.publish(speed)
            sleep(0.5)
            while vasak < 270:
                #print("Driving forward:")
                #print("vasak: ", vasak)
                #print("parem: ", parem)
                speed.vel_right = 0.5
                speed.vel_left = 0.5
                self.pub.publish(speed)
                vasak = self.left - left_start
                #vasak = self.left - left_start
                #print(parem)
            print("Otse")
            print("Vasak ", vasak)

            if vasak >= 270:
                vasak = self.left - left_start
                #vasak = self.left - left_start
                print("Stopping:")
                #print("vasak: ", vasak)
                #print("parem: ", parem)
                speed.vel_right = 0.0
                speed.vel_left = 0.0
                self.pub.publish(speed)
                sleep(0.5)
                print("Parem ", parem)
                while parem < 310:
                    #print("Turning left:")
                    #print("vasak: ", vasak)
                    #print("parem: ", parem)
                    parem = self.right - right_start
                    #vasak = self.left - left_start
                    speed.vel_right = 0.2
                    speed.vel_left = 0.0
                    self.pub.publish(speed)
                print("Parem ", parem)
                if parem >= 310:
                    #print("Stopping:")
                    #print("vasak: ", vasak)
                    #print("parem: ", parem)
                    speed.vel_right = 0.0
                    speed.vel_left = 0.0
                    self.pub.publish(speed)
                    sleep(0.5)
                print("Parem ", parem)
            while parem >= 310 and parem < 450:
                #print("Driving forward:")
                #print("vasak: ", vasak)
                #print("parem: ", parem)
                speed.vel_right = 0.4
                speed.vel_left = 0.4
                self.pub.publish(speed)
                parem = self.right - right_start
                #vasak = self.left - left_start
                #print(vasak)

            print("Paremale: ", parem)
            if parem >= 500:
                #print("Stopping:")
                #print("vasak: ", vasak)
                #print("parem: ", parem)
                speed.vel_right = 0.25
                speed.vel_left = 0.25
                self.pub.publish(speed)
                print("Parem ", parem)
        self.avoiding = False



#-------------------------------------- 90 KRAADISED KURVID / LÜHEM RADA VALIK ----------------------------------------------#

        #Äkiline parem põõrde funktsioon sooritab 90 kraadiseid põõrdeid
    def sharp_right(self):
        #print("starting to turn right")
        speed.vel_right = 0.0
        speed.vel_left = 0.2
        self.pub.publish(speed)
        while sum(self.errorlist) == 0 or self.errorlist in self.rightvalues:
            #print("Sharp right!")
            
            self.errorlist = list(map(int, self.theta_ref))            
            
            speed.vel_right = 0.0
            speed.vel_left = 0.2
            self.pub.publish(speed)
            #print("Right: ", self.theta_ref)
            self.lastturn = 1

        #Äkiline vasak põõrde funktsioon sooritab 90 kraadiseid põõrdeid
    def sharp_left(self):
        #print("starting to turn left")
        speed.vel_right = 0.2
        speed.vel_left = 0.0
        self.pub.publish(speed)

        while sum(self.errorlist) == 0 or self.errorlist in self.leftvalues:
            #print("Sharp left!")
            
            self.errorlist = list(map(int, self.theta_ref))            
            
            speed.vel_right = 0.2
            speed.vel_left = 0.0
            self.pub.publish(speed)
            #print("Left: ", self.theta_ref)
            self.lastturn = 2

        #Lühema raja valimine, valib lühema raja tuvastatdes joone ääres olevat ruutu mille järgi teab kumal pool on õige valik ning keerab õigesse suunda
    def short_path(self):
        print("Taking the short path!")
        while self.errorlist[0] == 1 or self.errorlist[1] == 1 or self.errorlist[2] == 1 and self.errorlist[5] == 0:
            
            self.errorlist = list(map(int, self.theta_ref))
            if self.errorlist[2] == 1 and self.errorlist[5] == 1:
                while self.errorlist[0] == 0 and self.errorlist[1] == 0:

                    while self.errorlist[0] == 1 or self.errorlist[1] == 1 or self.errorlist[2] == 1:

                    
                        speed.vel_right = 0.2
                        speed.vel_left = 0.0
                        self.pub.publish(speed)
        self.short = 0


#-------------------------------------- RUN FUNKTSIOON ----------------------------------------------#
    def run(self):
#Väljastab info iga sekund|| niipalju kordi
#                         \/
        rate = rospy.Rate(20) # 1Hz
        range1 = 0
        range2 = 0
        range3 = 0
        
        while not rospy.is_shutdown():

            #Delta T arvutamine
            self.timeR = time.time() 
            self.delta_t = self.timeR - self.lastCall
            #print("Delta t: ", self.delta_t)
            #self.delta_t = self.timeL - self.lastCall + 1
            #self.lastCall = self.timeL
            #print("DELTA T : ", self.delta_t)
            
            #Ümber takistuse
            range3 = range2
            range2 = range1
            range1 = self.range
            average = (range1 + range2 +range3) / 3
            
            if average < 0.28 and average > 0:
                print("A wall appeared!")
                speed.vel_right = 0.0
                speed.vel_left = 0.0
                self.pub.publish(speed)
                self.avoiding = True
                self.go_around()
                if self.avoiding == False:
                    pass
            
            
            #self.theta_ref = Line_follower_output.line_follower()
            #print(self.theta)
            
            #Joonelugeri info lugemine
            temp = self.bus.read_byte_data(62, 17)
            self.theta_ref = bin(temp)[2:].zfill(8)

            self.line_values = []
            for i, value in enumerate(str(self.theta_ref)):
                if value =='1':
                    self.line_values.append(self.array[i])
            '''
            emty_array = []
            for i, ele in enumerate(str(self.theta_ref)):
                if ele == "1":
                    emty_array.append(self.array[i + 1])
            '''

            if len(self.line_values) != 0:
                #self.position = sum(self.line_values) / len(self.line_values)
                self.position = sum(self.line_values)/len(self.line_values)

                self.PID, error = PID_controller.PID(self.position, self.delta_t, self.prev_e)
                self.prev_e = error
                #print("PID: ", self.PID)
                
                
                #print(self.theta_ref)
                #print(self.line_values)
                #self.errorlist = list(map(int, self.theta_ref))
                #print("errorlist = ", self.errorlist)

            else:
                self.PID = 0
            

            #D = (Error - previous error) / delta t

            if self.errorlist in self.rightvalues:  #Äkiline parem põõre
                #print("func start right")
                self.sharp_right()
               
            if self.errorlist in self.leftvalues:    #Äkiline vasak põõre
                #print("func start left")
                self.sharp_left()
                
            if self.line_values == [] and self.lastturn == 1:    #Äkiline parem põõre           self.position < 2 
                speed.vel_right = 0.0           
                speed.vel_left = 0.3
                self.pub.publish(speed)

            if self.line_values == [] and self.lastturn == 2:   #Äkiline vasak põõre     self.position > 7 
                speed.vel_right = 0.3
                speed.vel_left = 0.0
                self.pub.publish(speed)
            
            if len(self.line_values) != 0 and self.theta_ref is not self.rightvalues and self.theta_ref is not self.leftvalues:         #Lühikese raja läbimine

                #if self.errorlist in self.shortroute:
                #    self.short = 1
                
                #if self.short == 1 and self.errorlist[1] == 1  or self.errorlist[0] == 1 or self.errorlist in self.turningpoint:
                #    self.short_path()
         
                
                speed.vel_right = self.start_vel - self.PID
                speed.vel_left = self.start_vel + self.PID
                #print(speed.vel_right)
                #print(speed.vel_left)
                #print("PID: ", self.PID)
                self.pub.publish(speed)
                self.loops = self.loops + 1
                if self.loops > 50:
                    self.loops = 0
                    self.lastturn = 0
            self.lastCall = time.time()
            rate.sleep()
        
if __name__ == '__main__':
    #Teeb nodi
    node = MyPublisherNode(node_name='my_publisher_node')
    #Jooksutamis node
    node.run()
    #Jääb kordama
    rospy.spin()