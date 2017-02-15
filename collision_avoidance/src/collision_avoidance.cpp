
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>       /* atan2 */




class CollisionAvoidance {
    protected:
        ros::Subscriber scanSub;
        ros::Subscriber velSub;
        ros::Publisher velPub;

        ros::NodeHandle nh;

        // This might be useful
        double radius_max, radius_min, alpha_max;
		geometry_msgs::Twist vel_des;

        pcl::PointCloud<pcl::PointXYZ> lastpc;

        void velocity_filter(const geometry_msgs::TwistConstPtr msg) {
            vel_des = *msg;
        }

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::fromROSMsg(*msg, lastpc);
            geometry_msgs::Twist filtered = findClosestAcceptableVelocity(vel_des);
            velPub.publish(filtered);
        }

        geometry_msgs::Twist findClosestAcceptableVelocity(const geometry_msgs::Twist & desired) {
             geometry_msgs::Twist res = desired;
             // TODO: modify desired using the laser point cloud
			 unsigned int n = lastpc.size();
			 float x,y,radius,current_radius,alpha;
			 radius = 100.0;
             //ROS_INFO("New point cloud: %d points",n);
             for (unsigned int i=0;i<n;i++) {
				 x = lastpc[i].x;
                 y = lastpc[i].y;
				 current_radius=sqrt(pow(x,2)+pow(y,2));
                 alpha = atan2(x,y)*180/M_PI;
					
                 if (current_radius<radius && current_radius!=0.0 && alpha>90.0-alpha_max && alpha<90.0+alpha_max)
					 radius=current_radius;
             }
             
             if(radius<radius_max && radius>radius_min){
				res.linear.x = (radius-radius_min)/(radius_max-radius_min)*desired.linear.x;
				//ROS_INFO("Minimum radius is %.3f", radius);
			 } else if (radius<=radius_min && radius>0.0) {
				//ROS_INFO("Too close %.3f %.3f %.3f", x,y,radius);
				res.linear.x=0.0;
			 }
			 
			 return res;
        }

    public:
        CollisionAvoidance() : nh("~"),radius_max(1.0),radius_min(0.3),alpha_max(30.0) {
            scanSub = nh.subscribe("scans",1,&CollisionAvoidance::pc_callback,this);
            velSub = nh.subscribe("cmd_vel",1,&CollisionAvoidance::velocity_filter,this);
            velPub = nh.advertise<geometry_msgs::Twist>("output_vel",1);
            nh.param("radius_max",radius_max,1.0);
            nh.param("radius_min",radius_min,0.3);
            nh.param("alpha_max",alpha_max,30.0);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"collision_avoidance");

    CollisionAvoidance ca;

    ros::spin();
    // TODO: implement a security layer
}


