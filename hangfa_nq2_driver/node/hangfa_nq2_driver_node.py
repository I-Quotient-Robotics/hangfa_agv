#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import time
import math
import rospy
import numpy as np

import roslib
from geometry_msgs.msg import Twist

# CRC constant
CRC_TA = np.array([
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
], dtype=np.uint16)

UART_MSG_CMD                = 54
UART_MSG_PACKAGE_ENDCHAR    = 0x0D
UART_MSG_PACKAGE_STARTCHAR  = 0xAA

MOTOR_DRIVER_DEVICE_TYPE    = 0x20  # default is 0x40
MOTOR_DRIVER_ADDRESS        = 0x20  # default0x01 # broadcast adress

class Q2ROS_Driver(object):
    def __init__(self):
        rospy.init_node('hangfa_nq2_driver_node', anonymous=True)    
        rospy.Subscriber('cmd_vel', Twist, self.callback)

        self.timeout = rospy.get_param("~timeout", 0.5)
        self.baud = int(rospy.get_param("~baud", 115200))
        self.port = rospy.get_param("~driver_port", "/dev/ttyUSB0")
        #self.base_frame = rospy.get_param("~base_frame", 'base_link')
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        except:
            print("open %s erro!!"%(self.port))
            exit(-1)

        print("open %s ,baud:%i"%(self.port, self.baud))

    def setSpeed(self, parameter_list):
        pass

    def callback(self, data):
        #rospy.loginfo('x:%s,y:%s,z:%s,u:%s,v:%s,w:%s', data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y,  data.angular.z)
        if(data.linear.x == 0):
            if(data.linear.y > 0):
                angle = 0
                speed = abs(data.linear.y)
            elif(data.linear.y < 0):
                angle = -180
                speed = abs(data.linear.y)
        if(data.linear.y == 0):
            if(data.linear.x > 0):
                angle = -90
                speed = abs(data.linear.x)
            elif(data.linear.x < 0):
                angle = 90
                speed = abs(data.linear.x)
        if(data.linear.x == 0 and data.linear.y == 0): 
            angle = 0
            speed = 0

        if(data.linear.x > 0 and data.linear.y > 0):
            _th = math.atan2(abs(data.linear.y), abs(data.linear.x))*180.0/math.pi
            angle = -90+_th
            speed = math.sqrt(data.linear.x*data.linear.x + data.linear.y*data.linear.y)
        elif(data.linear.x > 0 and data.linear.y < 0):
            _th = math.atan2(abs(data.linear.y), abs(data.linear.x))*180.0/math.pi
            angle = -_th-90
            speed = math.sqrt(data.linear.x*data.linear.x + data.linear.y*data.linear.y)
        elif(data.linear.x < 0 and data.linear.y > 0):
            _th = math.atan2(abs(data.linear.y), abs(data.linear.x))*180.0/math.pi
            angle = 90-_th
            speed = math.sqrt(data.linear.x*data.linear.x + data.linear.y*data.linear.y)
        elif(data.linear.x < 0 and data.linear.y < 0):
            _th = math.atan2(abs(data.linear.y), abs(data.linear.x))*180.0/math.pi
            angle = 90+_th
            speed = math.sqrt(data.linear.x*data.linear.x + data.linear.y*data.linear.y) 

        w = data.angular.z*180.0/math.pi

        self.sendMotorSpeeds(angle,speed,w);

    def start(self):
        self.sendMotorSpeeds(0,0,0);
        rospy.spin()

    def bin_str(self, input_data):
        data = input_data
        if type(data) == str:
            data = ord(input_data)

        binary_string = "0b"+"{0:08b}".format(data)
        if (len(binary_string) > 10):
            print("ERROR IN CONVERSION")  # TODO: handle via exception
            return -1
        else:
            return binary_string

    def int16_to_int8(self, n):
        mask = (1 << 8) - 1
        foo = [(n >> k) & mask for k in range(0, 16, 8)]
        return foo

    def float_array_to_int8(self, speeds):
        int_array = []
        for speed in speeds:
            #split = int16_to_int8(int(round(speed * 10000)))
            split = self.int16_to_int8(int(round(speed)))
            int_array.append(split[0])
            int_array.append(split[1])
        return int_array

    def getCRCValue(self, values, length):
        ptr = 0
        crc = 0  # uint16
        da = 0  # uint8

        while length != 0:
            da = np.uint8(np.uint16(crc) >> 8)
            crc = np.uint16(crc << 8)
            crc = np.uint16(crc) ^ np.uint16(
                CRC_TA[np.uint8(da) ^ np.uint8(values[ptr])])
            ptr += 1
            length -= 1

        return crc

    def sendMotorSpeeds(self, direction, linearVel, angularVel):
        message = []  # array of binary strings

        speeds = self.float_array_to_int8(
            [direction*100, linearVel*10000, angularVel*100])

        header = [UART_MSG_PACKAGE_STARTCHAR, MOTOR_DRIVER_DEVICE_TYPE,
                MOTOR_DRIVER_ADDRESS, UART_MSG_CMD, len(speeds)]

        ## send header
        for data in header:
            message.append(self.bin_str(data))

        ## send all of speeds array in correct format
        for speed in speeds:
            message.append(self.bin_str(speed))
            #print bin_str(speed)
        ## calculate and append CRC values
        crc = self.getCRCValue(header+speeds, len(header+speeds))
        split_crc = self.int16_to_int8(crc)

        message.append(self.bin_str(split_crc[0]))
        message.append(self.bin_str(split_crc[1]))

        ## terminating character
        message.append(self.bin_str(UART_MSG_PACKAGE_ENDCHAR))

        ## convert back to bytes (TODO: might be unnecessary)
        byte_message = []
        hex_message = []

        for binary_string in message:
            data = int(binary_string, 2)
            byte_message.append(data)
            hex_message.append(hex(data))

        # print binary representation here:
        # print "SENDING MESSAGE"
        # print "Speeds: " + str([direction*100, linearVel*10000, angularVel*100])
        # print ' '.join('{:02x}'.format(x) for x in byte_message)
        self.ser.write(bytearray(byte_message))
        self.ser.flush()

    def sendStop(self):
        self.sendMotorSpeeds(0.0, 0.0, 0.0)

if __name__ == '__main__':
    Q2 = Q2ROS_Driver()
    Q2.start()