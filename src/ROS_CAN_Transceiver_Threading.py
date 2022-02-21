#! /usr/bin/env python3
# -*- coding: utf-8 -*-

#Software designed by Andrey Smirnov, Moscow State Unversity, 2022

import threading

import os
import can
import time
import numpy as np
import rospy
from std_msgs.msg import UInt16MultiArray


os.system('sudo ifconfig can0 down')
os.system('sudo ip link set can0 up type can bitrate 1000000   dbitrate 8000000 restart-ms 1000 berr-reporting on fd on')

def Subscriber_callback(TxBuffer_ROS):
    print ("Message Sent!")
    msg = can.Message(arbitration_id=TxBuffer_ROS.data[0], data=TxBuffer_ROS.data[1:9:], extended_id=False)
    can0.send(msg)
    return


def Subscriber(can0):
    print( "Subscriber Started!")
    sub = rospy.Subscriber('CAN_Tx_Buffer', UInt16MultiArray, Subscriber_callback)
    rospy.spin()
    
def Publisher(can0):
    print( "Publisher Started!")
    pub = rospy.Publisher('/CAN_Rx_Buffer', UInt16MultiArray, queue_size=1)
    while True:
        RxBuffer_ROS = UInt16MultiArray()
        RxBuffer = can0.recv()
    #    print("Received!")
        RxBuffer_ROS.data.append(RxBuffer.arbitration_id)
        for byte in RxBuffer.data:
            RxBuffer_ROS.data.append(byte)
     #   print(RxBuffer.data )
     #   print(RxBuffer_ROS )
        pub.publish (RxBuffer_ROS)

    



if __name__ == "__main__":
    try:
        rospy.init_node('CAN_Transceiver')
        can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan_ctypes')# socketcan_native
        threading.Thread(target=Subscriber, args=(can0,)).start()
        threading.Thread(target=Publisher, args=(can0,)).start()
    except KeyboardInterrupt:
        os.system('sudo ifconfig can0 down')
        exit()
