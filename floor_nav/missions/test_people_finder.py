#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('floor_nav')
import rospy
from math import *
from math import pi
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.2)
tc = TaskClient(server_node,default_period)
rospy.loginfo("Mission connected to server: " + server_node)

scale=2.0
vel=0.5


#
#
#tc.WaitForAuto()
while True:
	
	w4roi = tc.WaitForFace(foreground=False)
	tc.addCondition(ConditionIsCompleted("face detector",tc,w4roi))
	try:
		tc.Wander(front_sector=True,max_linear_speed=0.5)
		tc.clearConditions()
	except TaskConditionException, e:
		pass
		rospy.logerr("Exception caught: " + str(e))
		

	#if not rospy.core.is_shutdown():
	#	tc.SetManual()
	tc.Wait(duration=1.)
	tc.SetHeading(target=pi)
	rospy.loginfo("CONTINUEE")

rospy.loginfo("Mission completed")
