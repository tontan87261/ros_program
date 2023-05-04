#!/usr/bin/env python3
import rospy
from smbus2 import SMBus
from duckietown.dtros import DTROS, NodeType
from geometry_msgs.msg import Pose


#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

print("Imu fail")

class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.bus = SMBus(1)
        self.Device_Address = 0x68   # MPU6050 device address
      
        #i2c = board.I2C()
        #self.mpu = adafruit_mpu6050.MPU6050(i2c)

        #write to sample rate register
        self.bus.write_byte_data(self.Device_Address, SMPLRT_DIV, 7)
        
        #Write to power management register
        self.bus.write_byte_data(self.Device_Address, PWR_MGMT_1, 1)
        
        #Write to Configuration register
        self.bus.write_byte_data(self.Device_Address, CONFIG, 0)
        
        #Write to Gyro configuration register
        self.bus.write_byte_data(self.Device_Address, GYRO_CONFIG, 24)
        
        #Write to interrupt enable register
        self.bus.write_byte_data(self.Device_Address, INT_ENABLE, 1)


        self.pub = rospy.Publisher('IMU_acc', Pose, queue_size = 0)
        self.pub2 = rospy.Publisher('IMU_gyro', Pose, queue_size= 0)

        self.position = Pose()
        self.position2 = Pose()

    def read_raw_data(self, addr):
        #Accelero and Gyro value are 16-bit
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
            value = value - 65536
            return value
        

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(20) # 1Hz  

        while not rospy.is_shutdown():
            #Read Accelerometer raw value
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)
            if acc_x == None:
                acc_x = 0
            if acc_y == None:
                acc_y = 0
            if acc_z == None:
                acc_z = 0
            
            #Read Gyroscope raw value
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)

            if gyro_x == None:
                gyro_x = 0
            if gyro_y == None:
                gyro_y = 0
            if gyro_z == None:
                gyro_z = 0
            
            #Full scale range +/- 250 degree/C as per sensitivity scale factor
            Ax = acc_x/16384.0
            Ay = acc_y/16384.0
            Az = acc_z/16384.0

            self.position.position.x = Ax
            self.position.position.y = Ay
            self.position.position.z = Az

            self.pub.publish(self.position)

            Gx = gyro_x/131.0
            Gy = gyro_y/131.0
            Gz = gyro_z/131.0

            self.position2.position.x = Gx
            self.position2.position.y = Gy
            self.position2.position.z = Gz

            self.pub.publish(self.position2)
            
            
            #print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)     

            self.pub.publish()
            rate.sleep()

                
if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='IMU')
    # run node
    node.run()
    # keep spinning
    
    rospy.spin()