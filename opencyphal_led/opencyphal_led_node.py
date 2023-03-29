#!/usr/bin/env python3
import os
import sys
import copy
import re
import importlib
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from canfd_msgs.msg import OpenCyphalMessage
import cv2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile

if cv2.__version__ < "4.0.0":
    raise ImportError("Requires opencv >= 4.0, "
                      "but found {:s}".format(cv2.__version__))

class OpenCyphalLEDNode(Node):

    def __init__(self):

        super().__init__("opencyphal_led_node")

        image_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='SPI LED image topic.')

        cyphal_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Cyphal transmit topic.')

        joy_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Joy topic.')

        max_leds_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Max number of LEDs to drive.')

        led_brightness_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='LED Brightness [0,31].')
        

        self.declare_parameter("led_image_topic", "led_image", 
            image_topic_descriptor)

        self.declare_parameter("cyphal_topic", "CyphalTransmitFrame", 
            cyphal_topic_descriptor)
        
        self.declare_parameter("joy_topic", "/joy", 
            joy_topic_descriptor)

        self.declare_parameter("max_leds", 64, 
            max_leds_descriptor)

        self.declare_parameter("led_brightness", 10, 
            led_brightness_descriptor)


        self.joyTopic = self.get_parameter("joy_topic").value

        self.cyphalTopic = self.get_parameter("cyphal_topic").value

        self.imageTopic = self.get_parameter("led_image_topic").value

        self.maxLEDs = int(self.get_parameter("max_leds").value)

        self.Brightness = int(self.get_parameter("led_brightness").value)

        if self.Brightness > 31:
            self.Brightness = 31
        elif self.Brightness < 0:
            self.Brightness = 0

        if self.maxLEDs > 1000:
            self.maxLEDs = 1000
        elif self.maxLEDs < 1:
            self.maxLEDs = 1

        self.previousRGBHex=np.array([], dtype=np.uint32)

        self.pubMaxHz=500

        self.AllowRepeats = True

        self.bridge = CvBridge()
        
        self.InitTime = int(round(self.get_clock().now().nanoseconds/1000.0))
        
        self.CounterCyphalMsg = 0

        self.PubCyphal = self.create_publisher(OpenCyphalMessage, '{:s}'.format(self.cyphalTopic), 0)

        self.joySub = self.create_subscription(Joy, 
            '{:s}'.format(self.joyTopic), 
            self.joyCallback, 0)

        self.imageSub = self.create_subscription(Image, 
            '{:s}'.format(self.imageTopic), 
            self.imageCallback, 
            qos_profile_sensor_data)

    
    def imageCallback(self, data):
        passedImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        if passedImage.shape[0]*passedImage.shape[1] <= self.maxLEDs:
            self.BGR2RGBHex(passedImage)
        else:
            print('Image pixel size passed: {:d}, Max allowed: {:d}'.format(passedImage.shape[0]*passedImage.shape[1], self.maxLEDs))
        return

    def joyCallback(self, msg):
        hsvimg = np.zeros([self.maxLEDs,1,3], dtype=np.uint8)
        hsvimg[:,:,0] = np.ones([self.maxLEDs,1], dtype=np.uint8)*int(min((msg.axes[2]+1)*128, 255))
        hsvimg[:,:,1] = np.ones([self.maxLEDs,1], dtype=np.uint8)*int(min((msg.axes[5]+1)*128, 255))
        hsvimg[:,:,2] = np.ones([self.maxLEDs,1], dtype=np.uint8)*255
        img = cv2.cvtColor(hsvimg, cv2.COLOR_HSV2BGR)
        self.BGR2RGBHex(img)
        return

    def BGR2RGBHex(self, image):
        RGBHex=np.array([0], dtype=np.uint32)
        for y in range(image.shape[0]):
            for x in range(image.shape[1]):
                RGBHex=np.append(RGBHex, np.uint32((image[y][image.shape[1]-x-1][0] << 16) + (image[y][image.shape[1]-x-1][1] << 8) + image[y][image.shape[1]-x-1][2]))
        
        if not np.array_equal(self.previousRGBHex,RGBHex) or self.AllowRepeats:
            self.previousRGBHex=RGBHex
            NumberLeds = len(RGBHex)
            NumberGroups = int(np.ceil(NumberLeds/10.0))
            for OffsetGroup in range(NumberGroups):
                LedValArray=RGBHex[OffsetGroup*10:np.min([(OffsetGroup+1)*10,NumberLeds])]
                msg = OpenCyphalMessage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.priority = int(4)
                msg.is_annonymous = False
                msg.subject_id = int(501)
                msg.source_node_id = int(96)
                msg.data = self.ConvertDataSPILED(OffsetGroup, NumberLeds, self.Brightness, LedValArray)
                msg.crc= int(224+(self.CounterCyphalMsg%32))
                time.sleep(1.0/self.pubMaxHz)
                self.PubCyphal.publish(msg)
                self.CounterCyphalMsg += 1
        return
            


    def ConvertDataSPILED(self, OffsetGroup, NumberLeds, Brightness, LedValArray):
        DataArray=np.array([], dtype=np.uint8)

        TimeSinceInit = int(round(self.get_clock().now().nanoseconds/1000.0))-self.InitTime
        for i in range(8):
            DataArray = np.append(DataArray,
                    [np.uint8((TimeSinceInit >> i*8) & 255)], 
                    axis=0)
        if len(LedValArray) <= 10:
            for Led in range(len(LedValArray)):
                useBrightness = (Brightness & 0x1F) + 0xE0
                DataArray = np.append(DataArray, [
                            np.uint8((LedValArray[Led] >> 16) & 0xFF), #RED
                            np.uint8((LedValArray[Led] >> 8) & 0xFF), #GREEN
                            np.uint8(LedValArray[Led] & 0xFF), #BLUE
                            np.uint8(useBrightness)
                            ], axis=0)
            if len(LedValArray) < 10: 
                for NoVal in range(10-len(LedValArray)):
                    DataArray = np.append(DataArray, [
                            np.uint8(0),np.uint8(0),np.uint8(0),np.uint8(0)], axis=0)

        else:
            print("LedValArray too large, max is 10")
        
        DataArray = np.append(DataArray,
                            [np.uint8(NumberLeds & 255),
                             np.uint8(NumberLeds >> 8)], axis=0)
        DataArray = np.append(DataArray,[np.uint8(OffsetGroup & 255)], axis=0)
        
        while len(DataArray) < 63:
            DataArray = np.append(DataArray, 
                               [np.uint8(0)], axis=0)
        return DataArray

def main(args=None):
    rclpy.init(args=args)
    OCLN = OpenCyphalLEDNode()
    rclpy.spin(OCLN)
    OCLN.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

