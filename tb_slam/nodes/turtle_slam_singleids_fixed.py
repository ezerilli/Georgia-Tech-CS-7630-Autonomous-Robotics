#!/usr/bin/env python
import roslib
roslib.load_manifest('ar_slam_base')

import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, PoseStamped
from tf.transformations import euler_from_matrix, decompose_matrix, quaternion_from_euler
import message_filters
import threading
import numpy
from numpy import mat,vstack,diag, zeros, eye
from numpy.linalg import inv
from math import atan2, hypot, pi, cos, sin, fmod, sqrt

from ar_track_alvar.msg import AlvarMarkers

def norm_angle(x):
    return fmod(x+pi,2*pi)-pi


class BubbleSLAM:
    def __init__(self):
        rospy.init_node('bubble_slam')
        rospy.loginfo("Starting bubble rob slam")
        self.ignore_id = rospy.get_param("~ignore_id",False)
        self.target_frame = rospy.get_param("~target_frame","/map")
        self.body_frame = rospy.get_param("~body_frame","/base_link")
        self.odom_frame = rospy.get_param("~odom_frame","/odom")
        self.ar_precision = rospy.get_param("~ar_precision",0.5)
        self.position_uncertainty = rospy.get_param("~position_uncertainty",0.01)
        self.angular_uncertainty = rospy.get_param("~angular_uncertainty",0.01)
        self.initial_x = rospy.get_param("~initial_x",0.0)
        self.initial_y = rospy.get_param("~initial_y",0.0)
        self.initial_theta = rospy.get_param("~initial_theta",0.0)
        self.lastt = (0.0, 0.0, 0.0);
	self.lastr = 0.0;
        # instantiate the right filter based on launch parameters
        initial_pose = [self.initial_x, self.initial_y, self.initial_theta]
        initial_uncertainty = [0.01, 0.01, 0.01]

        self.lock = threading.Lock()
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_cb)
        self.X = mat(vstack(initial_pose))
        self.P = mat(diag(initial_uncertainty))
        self.idx = {}
        self.pose_pub = rospy.Publisher("~pose",PoseStamped,queue_size=1)
        self.marker_pub = rospy.Publisher("~landmarks",MarkerArray,queue_size=1)

        rospy.sleep(1.0);
        now = rospy.Time.now()
        lasttf = rospy.Time(0)
        self.listener.waitForTransform(self.odom_frame,self.body_frame, now, rospy.Duration(5.0))
        (trans,rot) = self.listener.lookupTransform(self.odom_frame,self.body_frame, lasttf)

    def predict(self, dt, dr):
        theta = self.X[2,0]
        self.X[0] = self.X[0] + dt[0];
        self.X[1] = self.X[1] + dt[1];
        self.X[2] = self.X[2] + dr;
        Jx = mat([[1, 0, -sin(theta)*dt[0]-cos(theta)*dt[1]],
                  [0, 1,  cos(theta)*dt[0]-sin(theta)*dt[1]],
                  [0, 0,  1]])
        Qs = mat(diag([self.position_uncertainty**2,self.position_uncertainty**2,self.angular_uncertainty**2]))
        P = self.P[0:3,0:3]
        self.P[0:3,0:3] = Jx * P * Jx.T + Qs 
        return (self.X,self.P)

    def getRotation(self, theta):
        R = mat(zeros((2,2)))
        R[0,0] = cos(theta); R[0,1] = -sin(theta)
        R[1,0] = sin(theta); R[1,1] = cos(theta)
        return R


    def update_ar(self, Z, id, uncertainty):
        # Z is a dictionary of id->vstack([x,y])
        (n,_) = self.X.shape
        R = mat(diag([uncertainty,uncertainty]))
        theta = self.X[2,0]
        Rtheta = self.getRotation(theta)
        Rmtheta = self.getRotation(-theta)
        H = mat(zeros((0, n)))
        if id in self.idx.keys():
            l = self.idx[id]
            H = mat(zeros((2,n)))
            H[0:2,0:2] = -Rmtheta
            H[0:2,2] = mat(vstack([-(self.X[l+0,0]-self.X[0,0])*sin(theta) + (self.X[l+1,0]-self.X[1,0])*cos(theta), \
                                   -(self.X[l+0,0]-self.X[0,0])*cos(theta) - (self.X[l+1,0]-self.X[1,0])*sin(theta)]))
            H[0:2,l:l+2] = Rmtheta
            Zpred = Rmtheta * (self.X[l:l+2,0] - self.X[0:2,0])
            S = H * self.P * H.T + R
            K = self.P * H.T * inv(S)
            zdiff = Z-Zpred;
            zdiff = abs(zdiff[0]) + abs(zdiff[1]);
            if zdiff <= 5:
                 self.X = self.X + K * (Z - Zpred)
                 self.P = (mat(eye(n)) - K * H) * self.P
            else :
                 print 'id: ' + repr(id) + ' error is too large.'
        else:
            self.idx[id] = n
            self.X = numpy.concatenate((self.X, self.X[0:2,0]+(Rtheta*Z)))
            Pnew = mat(diag([uncertainty]*(n+2)))
            Pnew[0:n,0:n] = self.P
            self.P = Pnew
        return (self.X,self.P)

    def ar_cb(self, markers):
        for m in markers.markers:
            if m.id > 32:
                continue
            lasttf = rospy.Time(0) #m.header.stamp
            self.listener.waitForTransform(self.body_frame, m.header.frame_id, lasttf, rospy.Duration(1.0))
            m_pose = PointStamped()
            m_pose.header = m.header
            m_pose.point = m.pose.pose.position
            m_pose = self.listener.transformPoint(self.body_frame, m_pose)
            Z = vstack([m_pose.point.x, m_pose.point.y])
            self.lock.acquire()
            if self.ignore_id:
                self.update_ar(Z, 0, self.ar_precision)
            else:
                self.update_ar(Z, m.id, self.ar_precision)
            self.lock.release()


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            lasttf = now  #rospy.Time(0) 
            self.listener.waitForTransform(self.odom_frame,self.body_frame, now, rospy.Duration(1.0))
            (trans,rot) = self.listener.lookupTransform(self.odom_frame,self.body_frame, lasttf)
            new_odom = mat(self.listener.fromTranslationRotation(trans,rot))
            euler = tf.transformations.euler_from_quaternion(rot);
            deltar = euler[2]-self.lastr;
            deltat = map(lambda x, y: x-y, trans, self.lastt)
            self.lastt = trans;
            self.lastr = euler[2];
            self.lock.acquire()
            self.predict(deltat, deltar);
            theta = self.X[2,0]
            pose_mat = mat([[cos(theta), -sin(theta), 0, self.X[0,0]], 
                          [sin(theta),  cos(theta), 0, self.X[1,0]],
                          [         0,           0, 1, 0],
                          [         0,           0, 0, 1],
                          ]);
            correction_mat = pose_mat * inv(new_odom);
            self.lock.release()
            scale, shear, angles, trans, persp = decompose_matrix(correction_mat)
            self.broadcaster.sendTransform(trans,
                    quaternion_from_euler(*angles),now, self.odom_frame, self.target_frame)
            self.publish(now)
            rate.sleep()

    def publish(self, timestamp):
        pose = PoseStamped()
        pose.header.frame_id = self.target_frame
        pose.header.stamp = timestamp
        pose.pose.position.x = self.X[0,0]
        pose.pose.position.y = self.X[1,0]
        pose.pose.position.z = 0.0
        Q = quaternion_from_euler(0, 0, self.X[2,0])
        pose.pose.orientation.x = Q[0]
        pose.pose.orientation.y = Q[1]
        pose.pose.orientation.z = Q[2]
        pose.pose.orientation.w = Q[3]
        self.pose_pub.publish(pose)
        ma = MarkerArray()
        marker = Marker()
        marker.header = pose.header
        marker.ns = "kf_uncertainty"
        marker.id = 5000
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.pose.position.z = -0.1
        marker.scale.x = 3*sqrt(self.P[0,0])
        marker.scale.y = 3*sqrt(self.P[1,1]);
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        ma.markers.append(marker)
        for id in self.idx.iterkeys():
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = self.target_frame
            marker.ns = "landmark_kf"
            marker.id = id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            l = self.idx[id]
            marker.pose.position.x = self.X[l,0]
            marker.pose.position.y = self.X[l+1,0]
            marker.pose.position.z = -0.1
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.scale.x = 0.2;#3*sqrt(self.P[l,l])
            marker.scale.y = 0.2;#3*sqrt(self.P[l+1,l+1]);
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.lifetime.secs=3.0;
            ma.markers.append(marker)
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = self.target_frame
            marker.ns = "landmark_kf"
            marker.id = 1000+id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = self.X[l+0,0]
            marker.pose.position.y = self.X[l+1,0]
            marker.pose.position.z = 1.0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.text = str(id)
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.2
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.lifetime.secs=3.0;
            ma.markers.append(marker)
        self.marker_pub.publish(ma)



if __name__=="__main__":
    demo = BubbleSLAM()
    demo.run()
