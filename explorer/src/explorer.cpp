
#include <ros/ros.h>
//#include <treasure_hunt/SigObsMap.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>



class Explorer {
    protected:

        ros::Publisher target_pub_;
		ros::Subscriber tot_map_sub_;
        ros::NodeHandle nh;
        tf::TransformListener listener_;
        
        std::string frame_id_;
        std::string base_link_;
        
        cv::Mat_<uint8_t> tg_;



		void tot_map_callback(const treasure_hunt::SigObsMap  & msg) {
				frame_id_ = msg->header.frame_id;
				tg_= msg->msg.map;
				
				tf::StampedTransform transform;
				listener_.lookupTransform(frame_id_,base_link_, ros::Time(0), transform);
				
				cv::Point3i current_point;
				double yaw = 0;
				if (debug) {
					current_point = og_center_;
				} else {
					yaw = tf::getYaw(transform.getRotation());
					current_point = cv::Point3i(transform.getOrigin().x() / info_.resolution, transform.getOrigin().y() / info_.resolution, (unsigned int)(round(yaw / (M_PI/4))) % 8)
						+ og_center_;
				}
			
			
			
			
			target_pub_.publish(goal);
			
		}


             
        

    public:
        Explorer() : nh("~"),radius_max(1.0) {
			nh_.param("base_frame",base_link_,std::string("/body"));
            
            tot_map_sub_ = nh_.subscribe("sig_grid",1,&Explorer::tot_map_callback,this);
            target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal",1);

        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"explorer");

    Explorer ex;

    ros::spin();
}


