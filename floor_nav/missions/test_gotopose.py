#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('floor_nav')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.05)
tc = TaskClient(server_node,default_period)
rospy.loginfo("Mission connected to server: " + server_node)

ang_goal=pi/2
goal=1
vel=0.3
dist=0.2
ang=0.2


tc.WaitForAuto()
try:
    tc.GoToPose(goal_x=goal,goal_y=goal,goal_theta=ang_goal,max_velocity=vel,dist_threshold=dist,angle_threshold=ang,smart_mode=True, relative=True)
    tc.Wait(duration=1.0)
    
except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()

rospy.loginfo("Mission completed")
