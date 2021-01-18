#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty
from vision import get_obj2w
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
        self.task = "go_goal"
        while not rospy.is_shutdown():
            if self.task == "go_goal":
                print("go goal")
                self.go_goal()
            elif self.task =="go_home":
                print("go home")
                self.go_home()
            elif self.task =="stop":
                print("stop")
                self.stop()
            elif self.task =="exit":
                print("exit")
                self.cmd_vel.publish(Twist())
                rospy.sleep(1)
                break
    def go_goal(self):
        self.move_cmd.linear.x = 0.2
        for i in range(71):
            self.cmd_vel.publish(self.move_cmd)
            self.rate.sleep()
        self.cmd_vel.publish(Twist())
        self.task = "stop"
    def stop(self):
        self.move_cmd.linear.x = 0
        for i in  range(60):
            self.cmd_vel.publish(self.move_cmd)
            self.rate.sleep()
        for i in  range(60):
            print(get_obj2w())
            self.rate.sleep()
        self.task = "go_home"
    def go_home(self):
        self.move_cmd.linear.x = -0.2
        for i in range(69):
            self.cmd_vel.publish(self.move_cmd)
            self.rate.sleep()
        self.cmd_vel.publish(Twist())
        self.task = "exit"
        self.shutdown()
            
    def shutdown(self):
        rospy.loginfo("Stopping wheelchair")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
if __name__=="__main__":
    #try:
    ControlWheelchair()
    #except:

    #    rospy.loginfo("End of this trip for wheelchair")
