#!/usr/bin/env python
import roslib; roslib.load_manifest('wpa_cli')
import rospy
import tf

from wpa_cli.msg import Scan,Network

log_dir = "/tmp"
reference_frame = "/map"
base_link = "/base_link"
def callback(scan):
    global log_dir, listener
    t = scan.header.stamp.to_sec()
    listener.waitForTransform(reference_frame,base_link, scan.header.stamp, rospy.Duration(1.0))
    ((x,y,z),rot) = listener.lookupTransform(reference_frame,base_link,scan.header.stamp)
    for n in scan.networks:
        f = open(log_dir+"/"+n.ssid+"-"+n.bssid+".csv","a")
        f.write("%e,%e,%d\n"%(x,y,n.level))
        f.close()

rospy.init_node('wpa_log')
log_dir = rospy.get_param("~log_directory","/tmp")
reference_frame = rospy.get_param("~reference_frame","/map")
base_link = rospy.get_param("~base_link","/base_link")
listener = tf.TransformListener()
rospy.sleep(1.0)
pub = rospy.Subscriber('~scan',Scan,callback,queue_size=1)

rospy.spin()



