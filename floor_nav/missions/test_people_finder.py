#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('floor_nav')
import rospy
import random
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.1)
tc = TaskClient(server_node,default_period)
rospy.loginfo("Mission connected to server: " + server_node)

scale=2.0
vel=0.5

tc.WaitForAuto()
while True:
	
	w4roi = tc.WaitForFace(foreground=False)
	tc.addCondition(ConditionIsCompleted("face detector",tc,w4roi))
	try:
		tc.Wander(front_sector=True,max_linear_speed=0.5)
	except TaskConditionException, e:
		rospy.loginfo("Path following interrupted on condition: %s" % \
                " or ".join([str(c) for c in e.conditions]))
		

	#if not rospy.core.is_shutdown():
	#	tc.SetManual()
	tc.StareAtFace(max_angular_velocity=0.6)
	tc.Wait(duration=3.)
	tc.SetHeading(target=random.choice([-pi/2, pi/2]),relative=True, max_angular_velocity=0.6,k_theta=0.8)
	rospy.loginfo("REPEAT")

rospy.loginfo("Mission completed")
