

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <laser_geometry/laser_geometry.h>


class LaserscanToPC2 {
    protected:
        ros::Subscriber laserSub;
        ros::Publisher scanPub;

        ros::NodeHandle nh;
        std::string base_frame;

        laser_geometry::LaserProjection projector_;
        tf::TransformListener listener_;

        void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
        {
            if(!listener_.waitForTransform(
                        scan_in->header.frame_id,
                        base_frame,
                        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                        ros::Duration(1.0))){
                return;
            }

            sensor_msgs::PointCloud2 cloud2;
            projector_.transformLaserScanToPointCloud(base_frame,*scan_in,
                    cloud2,listener_);
            scanPub.publish(cloud2);
        }


    public:
        LaserscanToPC2() : nh("~"), base_frame("/base_link") {
            laserSub = nh.subscribe("laser",1,&LaserscanToPC2::scanCallback,this);
            scanPub = nh.advertise<sensor_msgs::PointCloud2>("scan",1);
            nh.param("base_frame",base_frame,base_frame);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"ls_to_pc2");

    LaserscanToPC2 ls2pc;

    ros::spin();
    // TODO: implement a security layer
}


