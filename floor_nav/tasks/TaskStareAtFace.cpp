#include <math.h>
#include "TaskStareAtFace.h"
#include "floor_nav/TaskStareAtFaceConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO

TaskIndicator TaskStareAtFace::initialise() 
{
    ROS_INFO("Setting heading to %.2f deg", cfg.target*180./M_PI);
    if (cfg.relative) {
        const geometry_msgs::Pose2D & tpose = env->getPose2D();
        initial_heading = tpose.theta;
    } else {
        initial_heading = 0.0;
    }
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskStareAtFace::iterate()
{
	face_detect_base::roiVect ROIs = env->getROIs();
	sensor_msgs::RegionOfInterest currROI;
	
	if(ROIs.ROI.size()>0 ){
	//for(int i = 0; i< ROIs.ROI.size(); i++){
		currROI=ROIs.ROI[0];
		ROS_INFO("ROI STARE!");
			//double delta = remainder(98-currROI.x_offset,2*M_PI);
		double delta=-currROI.x_offset+(256-currROI.height)/2.;
		ROS_INFO("x ROI = %d ",currROI.x_offset);
		ROS_INFO("delta = %.2f ", delta);
		if(fabs(delta)<3)
			return TaskStatus::TASK_COMPLETED;	
		/*double rot = cfg.k_theta*delta;
		ROS_INFO("rot = %.2f ", rot);
		if (rot > cfg.max_angular_velocity) rot = cfg.max_angular_velocity;
		if (rot <-cfg.max_angular_velocity) rot =-cfg.max_angular_velocity;
		env->publishVelocity(0.0, rot);
		return TaskStatus::TASK_RUNNING;*/
		if(delta<0)
		 env->publishVelocity(0.0, +0.3);
		else
		 env->publishVelocity(0.0, -0.3);

	//	}	
	}else
		return TaskStatus::TASK_COMPLETED;

	

	/*
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    double alpha = remainder(initial_heading+cfg.target-tpose.theta,2*M_PI);
    if (fabs(alpha) < cfg.angle_threshold) {
		return TaskStatus::TASK_COMPLETED;
    }
    double rot = cfg.k_theta*alpha;
    if (rot > cfg.max_angular_velocity) rot = cfg.max_angular_velocity;
    if (rot <-cfg.max_angular_velocity) rot =-cfg.max_angular_velocity;
    env->publishVelocity(0.0, rot);
	return TaskStatus::TASK_RUNNING;*/
}

TaskIndicator TaskStareAtFace::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryStareAtFace);
