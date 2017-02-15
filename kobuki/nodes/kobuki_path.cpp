#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

class KobukiPath {
    protected:
        ros::Subscriber odomSub;
        ros::Publisher pathPub;

        ros::NodeHandle nh;

		nav_msgs::Path path_msg;

   
		
        void odom_callback(const nav_msgs::OdometryPtr msg) {
			geometry_msgs:: PoseStamped p;
			p.pose=msg->pose.pose;
			p.header=msg->header;
			
			path_msg.header = p.header;
            path_msg.poses.push_back(p);
            pathPub.publish(path_msg);
        }

    public:
        KobukiPath() : nh("~") {
            odomSub = nh.subscribe("odom",1,&KobukiPath::odom_callback,this);
            pathPub = nh.advertise<nav_msgs::Path>("path",1);
        }
};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"kobuki_path");

    KobukiPath kp;

    ros::spin();
    // TODO: implement a security layer
}


