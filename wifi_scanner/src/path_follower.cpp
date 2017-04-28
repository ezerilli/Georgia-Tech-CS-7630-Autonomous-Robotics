
#include <vector>
#include <string>
#include <map>
#include <list>


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <occgrid_planner_base/Trajectory.h>
#include <occgrid_planner_base/TrajectoryElement.h>


class PathFollower {
    protected:
        ros::NodeHandle nh_;
        tf::TransformListener listener_;
        tf::TransformBroadcaster broadcaster_;
        ros::Subscriber traj_sub_;
        ros::Publisher twist_pub_;
        ros::Publisher pose2d_pub_;
        ros::Publisher target_pub_;
        ros::Publisher reached_pub_;
        ros::Subscriber target_sub_;
        double look_ahead_;
        double Kx_,Ky_,Ktheta_;
        double max_rot_speed_;
        double max_velocity_;
        double max_y_error_;
        std::string frame_id_, base_frame_;
        geometry_msgs::PoseStamped goal;
        ros::Timer timer;
        std_msgs::Header reached;

        typedef std::map<double, occgrid_planner_base::TrajectoryElement> Trajectory;

        Trajectory traj_;

        void traj_cb(const occgrid_planner_base::TrajectoryConstPtr & msg) {
			traj_.clear();
            frame_id_ = msg->header.frame_id;
            for (unsigned int i=0;i<msg->Ts.size();i++) {
                traj_.insert(Trajectory::value_type(msg->Ts[i].header.stamp.toSec(), msg->Ts[i]));
            }
            ROS_INFO("Trajectory received");
        }
        
        void target_callback(const geometry_msgs::PoseStampedConstPtr & msg) {
             goal=*msg;
             timer.start();
		}
		
		void replanner_callback(const ros::TimerEvent& event){
			ROS_ERROR("Replanning");
			target_pub_.publish(goal);
		}

        geometry_msgs::Pose2D computeError(const ros::Time & now, const occgrid_planner_base::TrajectoryElement & te) {
            tf::StampedTransform transform;
            listener_.waitForTransform(base_frame_,frame_id_,now,ros::Duration(1.0));
            geometry_msgs::PoseStamped pose,error;
            pose.header.stamp = now;
            pose.header.frame_id = frame_id_;
            pose.pose  = te.pose;
            listener_.transformPose(base_frame_,pose,error);
            geometry_msgs::Pose2D result;
            result.x = error.pose.position.x;
            result.y = error.pose.position.y;
            result.theta = tf::getYaw(error.pose.orientation);
            //printf("Current error: %+6.2f %+6.2f %+6.2f\n",result.x,result.y,result.theta*180./M_PI);
            return result;
        }
        
    public:
        PathFollower(): nh_("~"){
            nh_.param("base_link",base_frame_,std::string("/base_link"));
            nh_.param("look_ahead",look_ahead_,1.0);
            nh_.param("Kx",Kx_,1.0);
            nh_.param("Ky",Ky_,1.0);
            nh_.param("Ktheta",Ktheta_,1.0);
            nh_.param("max_rot_speed",max_rot_speed_,1.0);
            nh_.param("max_velocity",max_velocity_,1.0);
            nh_.param("max_y_error",max_y_error_,1.0);
            traj_sub_ = nh_.subscribe<occgrid_planner_base::Trajectory>("traj",1,&PathFollower::traj_cb,this);
            twist_pub_ = nh_.advertise<geometry_msgs::Twist>("twistCommand",1);
            pose2d_pub_ = nh_.advertise<geometry_msgs::Pose2D>("error",1);
            target_sub_ = nh_.subscribe("goal",1,&PathFollower::target_callback,this);
            target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal",1);
            reached_pub_ = nh_.advertise<std_msgs::Header>("goal_reached",1);
            
            
        };
            
        inline double sat(double x, double max_x) {
            if (x < -max_x) return -max_x;
            if (x > +max_x) return +max_x;
            return x;
        }
        
        void run() {
            ros::Rate rate(20);
            ros::Time delay(0);
            timer = nh_.createTimer(ros::Duration(5.0), &PathFollower::replanner_callback,this);
            timer.stop();
            while (ros::ok()) {
                ros::spinOnce();
                if (traj_.size() > 0) {
                    bool final = false;
                    ros::Time now = ros::Time::now();
                    Trajectory::const_iterator it = traj_.lower_bound(now.toSec() + look_ahead_ - delay.toSec());
                    if (it == traj_.end()) {
                        // let's keep the final position
                        it --;
                        final = true;
                    }
                    tf::Transform transform;
                    tf::poseMsgToTF(it->second.pose,transform);
                    broadcaster_.sendTransform(tf::StampedTransform(transform, now, frame_id_, "/carrot"));

                    geometry_msgs::Pose2D error = computeError(now,it->second);
                    pose2d_pub_.publish(error);
                    geometry_msgs::Twist twist;
                    if(error.x>0.5){
						delay=delay+rate.cycleTime();
					}
                    if (final && (error.x < 0.05)) {
                        // Finished
                        traj_.clear();
                        twist.linear.x = 0.0;
                        twist.angular.z = 0.0;
												
                        reached.stamp = ros::Time::now();
                        reached.frame_id = frame_id_;
						reached_pub_.publish(reached);
						ROS_INFO("Goal Reached!!  Error = %.2f", error.x);
                    } else {				
                        twist.linear.x = it->second.twist.linear.x + Kx_ * error.x;
                        twist.linear.x = std::min(twist.linear.x,max_velocity_);
                        error.y = sat(error.y,max_y_error_);
                        twist.angular.z = it->second.twist.angular.z + Ktheta_ * error.theta 
                            + Ky_*it->second.twist.linear.x*exp(-error.theta*error.theta)*error.y;
                        twist.angular.z = sat(twist.angular.z,max_rot_speed_);
                    }
                    twist_pub_.publish(twist);
                } else {
                    geometry_msgs::Twist twist;
                    twist_pub_.publish(twist);
                }
                rate.sleep();
            }
        }
};


int main(int argc, char * argv[]) {
    ros::init(argc,argv,"path_follower");
    PathFollower pf;
    pf.run();
}


