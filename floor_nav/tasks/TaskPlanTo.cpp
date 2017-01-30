#include <math.h>
#include "TaskPlanTo.h"
#include "floor_nav/TaskPlanToConfig.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_PlanTo
#ifdef DEBUG_PlanTo
#warning Debugging task PlanTo
#endif


TaskIndicator TaskPlanTo::initialise() 
{
    error.x = NAN;
    publish_time = ros::Time::now();
    ROS_INFO("Going to %.2f %.2f",cfg.goal_x,cfg.goal_y);
    goalPub = env->getNodeHandle().advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
    trackSub = env->getNodeHandle().subscribe("/path_follower/error",1,
            &TaskPlanTo::error_cb, this);
    return TaskStatus::TASK_INITIALISED;
}

void TaskPlanTo::error_cb(geometry_msgs::Pose2DConstPtr msg) {
    error = *msg;
}


TaskIndicator TaskPlanTo::iterate()
{
    if ((ros::Time::now() - publish_time).toSec() > 1.0) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = env->getReferenceFrame();
        msg.pose.position.x = cfg.goal_x;
        msg.pose.position.y = cfg.goal_y;
        msg.pose.orientation = tf::createQuaternionMsgFromYaw(cfg.goal_theta);
        goalPub.publish(msg);
        publish_time = ros::Time::now();
    }

    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    double r = hypot(cfg.goal_y-tpose.y,cfg.goal_x-tpose.x);
    if ((r < cfg.dist_threshold) && (error.x < cfg.dist_threshold)) {
		return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}


DYNAMIC_TASK(TaskFactoryPlanTo);
