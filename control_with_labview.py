#!/usr/bin/env python

from __future__ import print_function
import threading
import time
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
global task
task ="stop"
import sys, select, termios, tty
from vision import get_obj2w
import socket
import random
PORT = 8089
HOST = '192.168.0.2'  # The server's hostname or IP address
#HOST = '127.0.0.1'
from datetime import datetime
from ctypes import *
SIZE_DATA_TCP_MAX  = 200
class Data(Union):
    _fields_ = [("byte", c_ubyte * SIZE_DATA_TCP_MAX),("int7Arr", c_int * 7),("float63dArr", c_float * 63)]

write_buffer = (c_char* 1024)()  
def tcp_init():
    global client, send_data
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = (HOST, PORT)
    client.connect(server_address)
    send_data = Data()    
def tcp_com():

    #'''
    global client
    global send_data,task
    start = datetime.now()
    try:
        task_data = get_obj2w()
        print("task_data",task_data)
        send_data.float63dArr[6]=task_data[0,0]
        send_data.float63dArr[7]=task_data[0,1]
        send_data.float63dArr[8]=task_data[0,2]
    except:
        pass
    #send_data.int7Arr[0]=random.randrange(1,100)
    send_data.float63dArr[0]=1.7
    memmove( write_buffer, send_data.byte,1024)
    client.sendall(write_buffer)
    end = datetime.now()
    labview_control = client.recv(5)
    b = bytearray(labview_control)
    print(labview_control)
    print(task)
    print(b[0],type(b[0]))
    if b[0]== 48:
        print(b[0],type(b[0]))
        task = "stop"
    elif b[1]== 49:
        print("googogogo")
        task = "go_forward"
    elif b[2]== 49:
        task = "go_back"
    elif b[3]== 49:
        task = "turn_left"
    elif b[4]== 49:
        task = "turn_right"



class ControlWheelchair():
    def __init__(self):
        rospy.init_node('Controlwheelchair',anonymous=False)
        rospy.loginfo("now wheelchair start")
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.rate = rospy.Rate(10)
        self.move_cmd = Twist()
        #move_cmd.linear.x = 0.3
        #move_cmd.angular.z = 0
        global task
        self.task = task
        print("Ros Control",self.task)
        while not rospy.is_shutdown():
            #self.rate.sleep(10)
            self.task = task
            print("self task",self.task)
            rospy.sleep(0.1)
            print("move_cmd",self.move_cmd)
            if self.task == "go_forward":
                print("lllllllllllllllllllllllllllllllllllllll")
                self.move_cmd = Twist()
                self.move_cmd.linear.x = 0.1
                self.cmd_vel.publish(self.move_cmd)
            elif self.task =="go_back":
                self.move_cmd = Twist()
                self.move_cmd.linear.x = -0.1
                self.cmd_vel.publish(self.move_cmd)
            elif self.task =="turn_left":
                self.move_cmd = Twist()
                self.move_cmd.angular.z = 0.1
                self.cmd_vel.publish(self.move_cmd)
            elif self.task =="turn_right":
                self.move_cmd = Twist()
                self.move_cmd.angular.z = -0.1
                self.cmd_vel.publish(self.move_cmd)
            elif self.task =="stop":
                print("hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh")
                self.move_cmd = Twist()
                self.cmd_vel.publish(self.move_cmd)
    def shutdown(self):
        rospy.loginfo("Stopping wheelchair")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
def tcp_com_while():
    while True:
        try:
            tcp_com()
        except:
            pass
        time.sleep(0.2)
        
        
def control():
    ControlWheelchair()
if __name__=="__main__":
    #try:
    

    tcp_init()
    t_tcp_com = threading.Thread(target=tcp_com_while)
    t_tcp_com.start()
    ControlWheelchair()
    #except:

    #    rospy.loginfo("End of this trip for wheelchair")
