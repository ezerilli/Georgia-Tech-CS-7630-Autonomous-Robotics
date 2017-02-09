#include <math.h>
#include "TaskStareAtFace.h"
#include "floor_nav/TaskStareAtFaceConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO

TaskIndicator TaskStareAtFace::initialise() 
{
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskStareAtFace::iterate()
{
	face_detect_base::roiVect ROIs = env->getROIs();
	sensor_msgs::RegionOfInterest currROI;
	
	if(ROIs.ROI.size()>0){
		int biggest=0;
		int biggest_width=ROIs.ROI[0].width;
		for(int i = 0; i< ROIs.ROI.size(); i++){
			currROI=ROIs.ROI[i];
			if(currROI.width>biggest_width){
				biggest_width=currROI.width;
				biggest=i;
			}
		}
		currROI=ROIs.ROI[biggest];
		ROS_INFO("STARE at ROI!");
		double delta = -(currROI.x_offset + currROI.width/2 - cfg.frame_width/2);
		ROS_INFO("x ROI = %d ",currROI.x_offset);
		ROS_INFO("delta = %.2f ", delta);
		
		if(fabs(delta)<cfg.delta_threshold)
			return TaskStatus::TASK_COMPLETED;	
		
		double rot = cfg.k_theta*delta;
		ROS_INFO("rot = %.2f ", rot);
		if (rot > cfg.max_angular_velocity) rot = cfg.max_angular_velocity;
		if (rot <-cfg.max_angular_velocity) rot =-cfg.max_angular_velocity;
		env->publishVelocity(0.0, rot);
		return TaskStatus::TASK_RUNNING;		
	}
}

TaskIndicator TaskStareAtFace::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryStareAtFace);
