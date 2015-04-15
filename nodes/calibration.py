#!/usr/bin/env python

import serial
import string
import math
import time
import sys


if __name__ == '__main__': 
    ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)

    acc_x_max = 0.0
    acc_x_min = 0.0
    acc_y_max = 0.0
    acc_y_min = 0.0
    acc_z_max = 0.0
    acc_z_min = 0.0

    mag_x_max = 0.0
    mag_x_min = 0.0
    mag_y_max = 0.0
    mag_y_min = 0.0
    mag_z_max = 0.0
    mag_z_min = 0.0
    
    gyro_x_offset = 0.0
    gyro_y_offset = 0.0
    gyro_z_offset = 0.0

    while True:
        line = ser.readline()
        print line
        words = string.split(line,",")
        #print words
        if len(words) == 11:
            try:   

                # 0.004, 0.00092, 0.00875
                acc_x = float(words[0])
                acc_x_max = acc_x if acc_x > acc_x_max else acc_x_max
                acc_x_min = acc_x if acc_x < acc_x_min else acc_x_min

                acc_y = float(words[1])
                acc_y_max = acc_y if acc_y > acc_y_max else acc_y_max
                acc_y_min = acc_y if acc_y < acc_y_min else acc_y_min

                acc_z = float(words[2])
                acc_z_max = acc_z if acc_z > acc_z_max else acc_z_max
                acc_z_min = acc_z if acc_z < acc_z_min else acc_z_min
        
                mag_x = float(words[3])
                mag_x_max = mag_x if mag_x > mag_x_max else mag_x_max
                mag_x_min = mag_x if mag_x < mag_x_min else mag_x_min

                mag_y = float(words[4])
                mag_y_max = mag_y if mag_y > mag_y_max else mag_y_max
                mag_y_min = mag_y if mag_y < mag_y_min else mag_y_min

                mag_z = float(words[5])
                mag_z_max = mag_z if mag_z > mag_z_max else mag_z_max
                mag_z_min = mag_z if mag_z < mag_z_min else mag_z_min

                gyro_x_offset = float(words[6])
                gyro_y_offset = float(words[7])
                gyro_z_offset = float(words[8])

            except Exception as e:
                print 'value error', e

    print "accel x,y,z (min/max) = %f/%f %f/%f %f/%f" % (acc_x_min, acc_x_max, acc_y_min, acc_y_max, acc_z_min, acc_z_max)  
    print "mag x,y,z (min/max) = %f/%f %f/%f %f/%f" % (mag_x_min, mag_x_max, mag_y_min, mag_y_max, mag_z_min, mag_z_max)      
    print "gyro x,y,z offset = %f %f %f" % (gyro_x_offset, gyro_y_offset, gyro_z_offset)    

    ser.close()        
    

