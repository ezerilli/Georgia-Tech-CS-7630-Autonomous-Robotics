#include <math.h>
#include "TaskGoToPose.h"
#include "floor_nav/TaskGoToPoseConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

#define DEBUG_GOTOPOSE
#ifdef DEBUG_GOTOPOSE
#warning Debugging task GOTOPOSE
#endif


TaskIndicator TaskGoToPose::initialise() 
{
    ROS_INFO("Going to %.2f %.2f, with angle ",cfg.goal_x,cfg.goal_y,cfg.goal_theta*180./M_PI);
    if (cfg.relative) {
        const geometry_msgs::Pose2D & tpose = env->getPose2D();
        x_init = tpose.x;
        y_init = tpose.y;
        theta_init = tpose.theta;
    } else {
        x_init = 0.0;
        y_init = 0.0;
        theta_init=0.0;
    }
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskGoToPose::iterate()
{
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    
    if (!cfg.holonomic_mode){
		if(!cfg.smart_mode){
			double r = hypot(y_init + cfg.goal_y-tpose.y,x_init + cfg.goal_x-tpose.x);
			double alpha = remainder(atan2((y_init + cfg.goal_y-tpose.y),x_init + cfg.goal_x-tpose.x)-tpose.theta,2*M_PI);
			if (r < cfg.dist_threshold) {
				double alpha = remainder(theta_init+cfg.goal_theta-tpose.theta,2*M_PI);
				if (fabs(alpha) < cfg.angle_threshold) return TaskStatus::TASK_COMPLETED;
				double rot = std::max(std::min(cfg.k_alpha*alpha,cfg.max_angular_velocity),-cfg.max_angular_velocity);
				if (fabs(alpha) > M_PI/9) rot = ((alpha>0)?+1:-1)*cfg.max_angular_velocity;
				env->publishVelocity(0.0, rot);
				return TaskStatus::TASK_RUNNING;		
			}
			#ifdef DEBUG_GOTOPOSE
			printf("c %.1f %.1f %.1f g %.1f %.1f r %.3f alpha %.1f\n",
					tpose.x, tpose.y, tpose.theta*180./M_PI,
					cfg.goal_x,cfg.goal_y,r,alpha*180./M_PI);
			#endif
			if (fabs(alpha) > M_PI/9) {
				double rot = ((alpha>0)?+1:-1)*cfg.max_angular_velocity;
			#ifdef DEBUG_GOTOPOSE
				printf("Cmd v %.2f r %.2f\n",0.,rot);
			#endif
				env->publishVelocity(0,rot);
			} else {
				double vel = cfg.k_v * r;
				double rot = std::max(std::min(cfg.k_alpha*alpha,cfg.max_angular_velocity),-cfg.max_angular_velocity);
				if (vel > cfg.max_velocity) vel = cfg.max_velocity;
			#ifdef DEBUG_GOTOPOSE
				printf("Cmd v %.2f r %.2f\n",vel,rot);
			#endif
				env->publishVelocity(vel,rot);
			}
			return TaskStatus::TASK_RUNNING;
		} else {
			double r= hypot(x_init+cfg.goal_x-tpose.x, y_init+cfg.goal_y-tpose.y);
			double alpha = remainder(-tpose.theta + atan2(y_init+cfg.goal_y-tpose.y,x_init+cfg.goal_x-tpose.x),2*M_PI);
			double beta =-(alpha +tpose.theta-cfg.goal_theta-theta_init);
			#ifdef DEBUG_GOTOPOSE
				printf("Cmd x %.2f y %.2f r %.2f a %.2f b %.2f\n",tpose.x,tpose.y,r,alpha,beta);
			#endif
			if (fabs(beta) < cfg.angle_threshold && r < cfg.dist_threshold) {
				return TaskStatus::TASK_COMPLETED;
			}
			double vel = cfg.k_v * r;	
			if (vel > cfg.max_velocity) vel = cfg.max_velocity;
			
			double rot = std::max(std::min(cfg.k_alpha*alpha+ cfg.k_beta*beta,cfg.max_angular_velocity),-cfg.max_angular_velocity);
			env->publishVelocity(vel,rot);
			return TaskStatus::TASK_RUNNING;
		}
	} else {
		double dx=x_init+cfg.goal_x-tpose.x;
		double dy=y_init+cfg.goal_y-tpose.y;
		double dtheta=remainder(theta_init+cfg.goal_theta-tpose.theta,2*M_PI);
		
		#ifdef DEBUG_GOTOPOSE
				printf("Cmd x %.2f y %.2f dx %.2f dy %.2f dtheta %.2f\n",tpose.x,tpose.y,dx,dy,dtheta);
		#endif
		
		if (fabs(dtheta) < cfg.angle_threshold && hypot(dx,dy) < cfg.dist_threshold) {
			return TaskStatus::TASK_COMPLETED;
		}
		
		double vel_x= cfg.k_v*(cos(cfg.goal_theta)*dx+sin(cfg.goal_theta)*dy);
		double vel_y= cfg.k_v*(-cos(cfg.goal_theta)*dx+sin(cfg.goal_theta)*dy);
		double rot= cfg.k_gamma*dtheta;
		
		vel_x = std::max(std::min(vel_x,cfg.max_velocity),-cfg.max_velocity);
		vel_y = std::max(std::min(vel_y,cfg.max_velocity),-cfg.max_velocity);
		rot = std::max(std::min(rot,cfg.max_angular_velocity),-cfg.max_angular_velocity);
		
		env->publishVelocity(vel_x, vel_y, rot);
		return TaskStatus::TASK_RUNNING;
		}
}

TaskIndicator TaskGoToPose::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoToPose);
