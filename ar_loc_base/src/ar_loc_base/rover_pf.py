import roslib; roslib.load_manifest('ar_loc_base')
import rospy
from numpy import *
from numpy.linalg import pinv, inv
from math import pi, sin, cos
from geometry_msgs.msg import *
import tf
import bisect
import threading

from rover_kinematics import *

class RoverPF(RoverKinematics):
	def __init__(self, initial_pose, initial_uncertainty):
		RoverKinematics.__init__(self)
		self.initial_uncertainty = initial_uncertainty
		self.lock = threading.Lock()
		self.X = mat(vstack(initial_pose))
		# Initialisation of the particle cloud around the initial position
		self.N = 500
		self.particles = [self.X + self.drawNoise(initial_uncertainty) for i in range(0,self.N)]
		self.pa_pub = rospy.Publisher("~particles",PoseArray,queue_size=1)

	def getRotation(self, theta):
		R = mat(zeros((2,2)))
		R[0,0] = cos(theta); R[0,1] = -sin(theta)
		R[1,0] = sin(theta); R[1,1] = cos(theta)
		return R
	
	# Draw a vector uniformly around [0,0,0], scaled by norm
	def drawNoise(self, norm):
		if type(norm)==list:
			return mat(vstack(norm)*(2*random.rand(3,1)-vstack([1,1,1])))
		else:
			return mat(multiply(norm,((2*random.rand(3,1)-vstack([1,1,1])))))
	

	def predict(self, motor_state, drive_cfg, encoder_precision):
		self.lock.acquire()
		# The first time, we need to initialise the state
		if self.first_run:
			self.motor_state.copy(motor_state)
			self.first_run = False
			self.lock.release()
			return 
		# Prepare odometry matrices (check rover_odo.py for usage)
		iW = self.prepare_inversion_matrix(drive_cfg)
		S = self.prepare_displacement_matrix(self.motor_state,motor_state,drive_cfg)
		self.motor_state.copy(motor_state)

		# Apply the particle filter prediction step here
		# TODO
		dX = iW*S
		#think the encoder_precision like a dS (displacement) vector in odometry, it will cause a deltaX,deltaY,deltaTheta in the robot frame
		var=iW*mat(vstack([encoder_precision] * len(S)))	
		
		#now apply the main displacement dX to each particles superposing the variation caused by the encoder precision
		new_part=self.particles
		i=0
		deltaPart=mat(vstack([.0] *3))
		for part in self.particles:
			theta = part[2,0]
			noise=self.drawNoise(var)
			deltaPart[0,0] = (dX[0,0]+ noise[0,0])*cos(theta) - (dX[1,0]+ noise[1,0])*sin(theta)
			deltaPart[1,0] = (dX[0,0]+ noise[0,0])*sin(theta) + (dX[1,0]+ noise[1,0])*cos(theta)
			deltaPart[2,0] = dX[2,0] + noise[2,0]
			new_part[i] = (part + deltaPart)
			i+=1			  
		
		self.particles = new_part		
		self.lock.release()


	def update_ar(self, Z, L, Uncertainty):
		self.lock.acquire()
		print "Update: L="+str(L.T)
		# Implement particle filter update using landmarks here
		# Note: the function bisect.bisect_left could be useful to implement
		# the resampling process efficiently
		# Z is a 2x1 vector of the measurement in the rover reference frame.
		# L is the position of the observed landmark in the world frame as a 2x1 vector. 
		# Uncertainty is the uncertainty on the measurement process, given as a direction-less radius.
		# TODO
		
		weights=mat(vstack([.0] * len(self.particles)))
		new_part=self.particles
		i=0
		currPart=mat(vstack([.0] *3))
		for part in self.particles:
			# calculating the supposed position of the landemark starting from the current particle position
			rotZ= getRotation(part[2,0])*Z
			currPart[0,0] = part[0,0] + rotZ[0,0]
			currPart[1,0] = part[1,0] + rotZ[1,0]
			#distance between the particle and the landmark
			dist=hypot(currPart[0,0]-L[0,0],currPart[1,0]-L[1,0])
			#form a weights vector, the nearer the particle is to the robot the bigger is the weight
			weights[i,0]=1/(sqrt(2*pi)*Uncertainty)*exp(-pow(dist/Uncertainty,2)/2) + 0.01  
			i+=1
	     
	    # normalizing the distribution 
		norm_fact = sum(weights)
		pdf = [weight/norm_fact for weight in weights]
		# resampling thanks to the cdf
		cdf = cumsum(pdf)
		samples = random.rand(len(self.particles))
		indexes=[]
		for sample in samples:
			index = bisect.bisect_left(cdf,sample)
			indexes.append(index)
		
		indexes=array(indexes)
		self.particles = [self.particles[i] for i in indexes]
		self.lock.release()

	def update_compass(self, angle, Uncertainty):
		self.lock.acquire()
		print "Update: C="+str(angle)
		# Implement particle filter update using landmarks here
		# Note: the function bisect.bisect_left could be useful to implement
		# the resampling process efficiently
		# TODO

		# self.particles = ...
		
		self.lock.release()

	def updateMean(self):
		X = mat(zeros((3,1)))
		for x in self.particles:
			X += x
		self.X = X / len(self.particles)
		
		return self.X

	def publish(self, pose_pub, target_frame, stamp):
		# Only compute the mean for plotting
		self.updateMean()
		pose = PoseStamped()
		pose.header.frame_id = target_frame
		pose.header.stamp = stamp
		pose.pose.position.x = self.X[0,0]
		pose.pose.position.y = self.X[1,0]
		pose.pose.position.z = 0.0
		Q = tf.transformations.quaternion_from_euler(0, 0, self.X[2,0])
		pose.pose.orientation.x = Q[0]
		pose.pose.orientation.y = Q[1]
		pose.pose.orientation.z = Q[2]
		pose.pose.orientation.w = Q[3]
		pose_pub.publish(pose)

		pa = PoseArray()
		pa.header = pose.header
		for p in self.particles:
			po = Pose()
			po.position.x = p[0,0]
			po.position.y = p[1,0]
			q = tf.transformations.quaternion_from_euler(0, 0, p[2,0])
			po.orientation = Quaternion(*q)
			pa.poses.append(po)
		self.pa_pub.publish(pa)

	def broadcast(self,br, target_frame, stamp):
		br.sendTransform((self.X[0,0], self.X[1,0], 0),
					 tf.transformations.quaternion_from_euler(0, 0, self.X[2,0]),
					 stamp, "/%s/ground"%self.name, target_frame)
		

