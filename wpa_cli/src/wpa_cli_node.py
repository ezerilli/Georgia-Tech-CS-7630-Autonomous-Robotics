#!/usr/bin/env python
import roslib; roslib.load_manifest('wpa_cli')
import rospy
import subprocess
import re

from wpa_cli.msg import Scan,Network

rospy.init_node('wpa_cli')
pub = rospy.Publisher('~scan',Scan,latch=True,queue_size=1)
scan_period = rospy.get_param("~scan_period",10.0)
update_period = rospy.get_param("~update_period",5.0)
ignore = rospy.get_param("~ignore_ssid","")
ignore = ignore.split(",")
rospy.loginfo("Ignore SSID: " + str(ignore))

def scan_equals(s1,s2):
    if s1.interface != s2.interface:
        return False
    if len(s1.networks) != len(s2.networks):
        return False
    for n1,n2 in zip(s1.networks,s2.networks):
        if n1.ssid != n2.ssid:
            return False
        if n1.bssid != n2.bssid:
            return False
        if n1.level != n2.level:
            return False
        if n1.frequency != n2.frequency:
            return False
    return True

last_scan = Scan()
last_scan_time = rospy.Time.now()
subprocess.call(["wpa_cli", "scan"])

while not rospy.is_shutdown():
    scan = Scan()
    scan.header.stamp = rospy.Time.now()
    if (scan.header.stamp - last_scan_time).to_sec() > scan_period:
        last_scan_time = scan.header.stamp
        subprocess.call(["wpa_cli", "scan"])
    scan_results = subprocess.check_output(["wpa_cli", "scan_results"]).split("\n")
    for l in scan_results:
        m=re.search("Selected interface '([^']*)'",l)
        if m:
            scan.interface = m.group(1)
            continue
        m=re.search("bssid.*",l)
        if m:
            continue
        col = l.split()
        if len(col)<5:
            continue
        n = Network()
        n.ssid = col[4]
        if n.ssid in ignore:
            continue
        n.bssid = col[0]
        n.frequency = int(col[1])
        n.level = int(col[2])
        n.flags = col[3]
        scan.networks.append(n)
    if not scan_equals(scan,last_scan):
        pub.publish(scan)
        last_scan = scan
        rospy.loginfo("Detected %d networks" % len(scan.networks))
    rospy.sleep(update_period)



