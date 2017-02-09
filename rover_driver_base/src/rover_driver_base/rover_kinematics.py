#!/usr/bin/env python
import roslib; roslib.load_manifest('rover_driver_base')
import rospy
from geometry_msgs.msg import Twist
import numpy
from numpy.linalg import pinv
from math import atan2, hypot, pi, cos, sin

prefix=["FL","FR","CL","CR","RL","RR"]

class RoverMotors:
	def __init__(self):
		self.steering={}
		self.drive={}
		for k in prefix:
			self.steering[k]=0.0
			self.drive[k]=0.0
	def copy(self,value):
		for k in prefix:
			self.steering[k]=value.steering[k]
			self.drive[k]=value.drive[k]

class DriveConfiguration:
	def __init__(self,radius,x,y,z):
		self.x = x
		self.y = y
		self.z = z
		self.radius = radius

class RoverKinematics:
	def __init__(self):
		self.X = numpy.asmatrix(numpy.zeros((3,1)))
		self.motor_state = RoverMotors()
		self.first_run = True

	def twist_to_motors(self, twist, drive_cfg, skidsteer=False):
		motors = RoverMotors()
		if skidsteer:
			for k in drive_cfg.keys():
				# Insert here the steering and velocity of 
				# each wheel in skid-steer mode
				motors.steering[k] = 0
				motors.drive[k] = (twist.linear.x - twist.angular.z*drive_cfg[k].y)/drive_cfg[k].radius
		else:
			for k in drive_cfg.keys():
				# Insert here the steering and velocity of 
				# each wheel in rolling-without-slipping mode
				vwx=twist.linear.x - twist.angular.z*drive_cfg[k].y
				vwy=twist.linear.y + twist.angular.z*drive_cfg[k].x
				motors.steering[k] = atan2(vwy,vwx)
				motors.drive[k] = hypot(vwx,vwy)/drive_cfg[k].radius
				if(motors.steering[k]>pi/2):
					motors.steering[k] -= pi
					motors.drive[k] = -motors.drive[k]
				if(motors.steering[k]<=-pi/2):
					motors.steering[k] += pi
					motors.drive[k] = -motors.drive[k]		
		return motors
		
	

	def integrate_odometry(self, motor_state, drive_cfg):
		# The first time, we need to initialise the state
		if self.first_run:
			self.motor_state.copy(motor_state)
			self.first_run = False
			return self.X
		# Insert here your odometry code
		def mod(x,y):
			return (x+y/2)%y -y/2	
		A = [[0 for x in range(3)] for y in range(12)]
		B = [[0 for x in range(1)] for y in range(12)]
		h=0
		for k in drive_cfg.keys():
			A[2*h][0]=1
			A[2*h][2]=-drive_cfg[k].y
			A[2*h+1][1]=1
			A[2*h+1][2]=drive_cfg[k].x
			s=mod(motor_state.drive[k]- self.motor_state.drive[k],2*pi)*drive_cfg[k].radius
			beta=(motor_state.steering[k]+self.motor_state.steering[k])/2
			B[2*h][0]=s*cos(beta)
			B[2*h+1][0]=s*sin(beta)
			h+=1
		A=numpy.matrix(A)
		B=numpy.matrix(B)
		dX=pinv(A)*B
		#dX = numpy.dot(pinv(A), B)
		self.X[0,0] += dX[0,0]*cos(self.X[2,0]) - dX[1,0]*sin(self.X[2,0])
		self.X[1,0] += dX[0,0]*sin(self.X[2,0]) + dX[1,0]*cos(self.X[2,0])
		self.X[2,0] += dX[2,0]
		print "X = "+ str(self.X)
		self.motor_state.copy(motor_state)
		return self.X



