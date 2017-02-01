#include <math.h>
#include "TaskWaitForFace.h"
#include "floor_nav/TaskWaitForFaceConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;


TaskIndicator TaskWaitForFace::iterate()
{   
	face_detect_base::roiVect ROIs = env->getROIs();
	if(ROIs.ROI.size()>0){
		ROS_INFO("ROI");
		return TaskStatus::TASK_COMPLETED;
	}else
		ROS_INFO("NO ROI");
	
    /*const geometry_msgs::Pose2D & tpose = env->getPose2D();
    ROS_INFO("NO ROI");
    double r = hypot(cfg.roi_y-tpose.y,cfg.roi_x-tpose.x);
    if (r < cfg.roi_radius) {
        ROS_INFO("Detected ROI at %.2f %.2f",tpose.x, tpose.y);
		return TaskStatus::TASK_COMPLETED;
    }*/
	return TaskStatus::TASK_RUNNING;
}

DYNAMIC_TASK(TaskFactoryWaitForFace);
