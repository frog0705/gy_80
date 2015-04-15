#!/usr/bin/env python

import threading
import sys
import serial
import time
import string
import math

import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

IMU_FRAME = 'imu_link'
grad2rad = 3.141592654/180.0

#def process_msg(msg):
    

class  SerialDataGateway(object):

    def __init__(self):
        self.imu_pub = rospy.Publisher(rospy.get_name()+'/imu/data_raw', Imu, queue_size = 0) 
        self.mag_pub = rospy.Publisher(rospy.get_name()+'/imu/mag', Vector3Stamped, queue_size = 0)      
        self.port = rospy.get_param("~port", "/dev/ttyACM0")
        self.baud = int(rospy.get_param("~baud", "57600"))
        rospy.loginfo('Publishing IMU data on topics:\n'+rospy.get_name()+'/imu/data_raw\n' + rospy.get_name()+'/imu/mag\n')
        self.imuMsg = Imu()
        self.imuMsg.orientation_covariance = [999999 , 0 , 0, 0, 9999999, 0, 0, 0, 999999]
        self.imuMsg.angular_velocity_covariance = [9999, 0 , 0, 0 , 99999, 0, 0 , 0 , 0.02]
        self.imuMsg.linear_acceleration_covariance = [0.2 , 0 , 0, 0 , 0.2, 0, 0 , 0 , 0.2]

        self.magMsg = Vector3Stamped()
        self._KeepRunning = False


    def Start(self):
        self._Serial = serial.Serial(self.port, self.baud, timeout=1)
        self._Serial.flushInput()
        self._Serial.flushOutput()
        #arduino auto-reset when open serial port

        self._KeepRunning = True
        self._ReceiverThread = threading.Thread(target=self.Process)
        self._ReceiverThread.setDaemon(True)
        self._ReceiverThread.start()

    def Stop(self):
        rospy.loginfo("Stopping serial gateway")
        self._KeepRunning = False
        time.sleep(.1)
        self._Serial.close()

    def Write(self, data):
	info = "Writing to serial port: %s" %data
	rospy.loginfo(info)
	self._Serial.write(data)

    def Process(self):
        while self._KeepRunning:
            line = self._Serial.readline()
            print line
            words = string.split(line,",")
            #print words
            if len(words) == 11:
                try:   


                    '''
                    # a_scale=0.004, m_scale=0.00092, g_scale=0.00875
                    a_z=(g/257)*(v+43)
                    a_y=(g/260)*(v+4)
                    a_x=(g/254)*(v+2)
                    sh_g=9.975
                    '''
                    self.imuMsg.linear_acceleration.x = (float(words[0]) + 2)*0.038563
                    self.imuMsg.linear_acceleration.y = (float(words[1]) + 4)*0.037673
                    self.imuMsg.linear_acceleration.z = (float(words[2]) + 43)*0.038113
        

                    self.magMsg.vector.x = float(words[3])*0.001
                    self.magMsg.vector.y = float(words[4])*0.001
                    self.magMsg.vector.z = float(words[5])*0.001

                    self.imuMsg.angular_velocity.x = float(words[6])*0.00875*grad2rad
                    self.imuMsg.angular_velocity.y = float(words[7])*0.00875*grad2rad
                    self.imuMsg.angular_velocity.z = float(words[8])*0.00875*grad2rad

                except Exception as e:
                    print 'value error', e
        
            self.imuMsg.orientation.x = 0
            self.imuMsg.orientation.y = 0
            self.imuMsg.orientation.z = 0
            self.imuMsg.orientation.w = 1
            self.imuMsg.header.stamp= rospy.Time.now()
            self.imuMsg.header.frame_id = IMU_FRAME
            self.imu_pub.publish(self.imuMsg)

            self.magMsg.header.stamp= rospy.Time.now()
            self.magMsg.header.frame_id = IMU_FRAME
            self.mag_pub.publish(self.magMsg)
   
    
if __name__ == '__main__': 
    rospy.init_node('gy_80') 
    dataReceiver = SerialDataGateway()
    dataReceiver.Start()

    raw_input("Hit <Enter> to end.")
    dataReceiver.Stop()
