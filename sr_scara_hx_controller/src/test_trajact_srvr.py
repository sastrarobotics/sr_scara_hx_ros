#!/usr/bin/env python
import roslib; roslib.load_manifest('sr_scara_hx_controller')
import time, sys, threading, math
import copy
import datetime
import socket, select
import struct
import traceback, code
import optparse
import SocketServer
from BeautifulSoup import BeautifulSoup

import herkulex
from sensor_msgs.msg import JointState


num_joints = 4
herkulex.connect("/dev/ttyUSB1",115200)
herkulex.torque_on(1)
herkulex.torque_on(2)

import rospy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def joint_state_publisher():
    pub = rospy.Publisher('joint_states', JointState,queue_size = 5)
    #rospy.init_node('sr_jnt_stt')
    
    r = rospy.Rate(300)
    while not rospy.is_shutdown():
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = num_joints * [0.0]
        msg.velocity = num_joints * [0.0]
        #msg.effort = num_joints * [0.0]
        msg.name =  ['joint1', 'joint2', 'joint3', 'gripper_con']
        msg.position[0] = math.radians(herkulex.get_servo_angle(1))
        msg.position[1] = math.radians(herkulex.get_servo_angle(2))
        msg.position[2] =0.0
        msg.position[3] =0.0
        pub.publish(msg)
        r.sleep()


class SR_TrajectoryFollower:
	def __init__(self):
		self.server = actionlib.SimpleActionServer("scara_arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction, self.execute, auto_start=False)

	def execute(self, goal):
		print "I am executing and I got the goal:"#, goal.trajectory.points
                joint_names = goal.trajectory.joint_names
                trajectory_points = goal.trajectory.points
                num_points = len(trajectory_points)
                end_time = trajectory_points[-1].time_from_start.to_sec()
                pnt_times = [pnt.time_from_start.to_sec() for pnt in trajectory_points]
                pnt_positions =[pn.positions for pn in trajectory_points]
                #print joint_names , num_points, end_time , pnt_times, pnt_positions
                #print pnt_positions[0]
                #start_point = JointTrajectoryPoint()
                #start_point.positions = self._get_current_position(joint_names)
                for pnts in range(num_points):
                    #print pnt_positions[pnts]
                    #print " joint1 ", math.degrees(pnt_positions[pnts][1]),"joint2",math.degrees(pnt_positions[pnts][2])
                    herkulex.set_servo_angle(1,math.degrees(pnt_positions[pnts][1]),100,0x08)
                    herkulex.set_servo_angle(2,math.degrees(pnt_positions[pnts][2]),100,0x08)
                self.server.set_succeeded()

	def start(self):
		self.server.start()
        print "The action server for this driver has been started"

def main():
	rospy.init_node('sr_scara_action_server')
	action_server = SR_TrajectoryFollower()
	action_server.start()
        joint_state_publisher()
	rospy.spin()

if __name__ == '__main__': 
	main()
