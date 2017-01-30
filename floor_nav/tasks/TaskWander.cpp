#include <math.h>
#include "TaskWander.h"
#include "floor_nav/TaskWanderConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_Wander
#ifdef DEBUG_Wander
#warning Debugging task Wander
#endif


TaskIndicator TaskWander::initialise() 
{
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskWander::iterate()
{
    const pcl::PointCloud<pcl::PointXYZ>& pc = env->getPointCloud2D();
    std::map<int,double> sensor_map;
    std::map<int,double>::iterator sensor_it;

    // Convert the pointcloud into a discrete set of range measurement
    // separated by ANGULAR_RANGE
    sensor_map.clear();
    unsigned int n = pc.size();
    for (unsigned int i=0;i<n;i++) {
        double r = hypot(pc[i].y,pc[i].x);
        if (r < 1e-2) {
            // bogus point, the laser did not return
            continue;
        }
        double alpha = atan2(pc[i].y,pc[i].x);
        // With this line, sensor_map[0] will contain the point in
        // front of the robot
        // int ialpha = round(alpha / angular_range);

        // With this one, there is no sector just in front of the robot
        int ialpha = 0;
        if (cfg.front_sector) {
            ialpha = round(alpha / cfg.angular_range);
        } else {
            ialpha = ceil(alpha / cfg.angular_range);
        }
        sensor_it = sensor_map.find(ialpha);
        if (sensor_it == sensor_map.end()) {
            sensor_map[ialpha] = r;
        } else {
            sensor_it->second = std::min(r,sensor_it->second);
        }
    }
    // At this stage sensor_map[0] corresponds to the closest range in
    // [-30,0], sensor_map[1] corresponds to the closest range in
    // [0 : 30] degrees, etc
    try {
        // Use the value stored in sensor_map to implement the desired
        // behaviors
        double min_forward = std::min(sensor_map[0],sensor_map[1]);
        double linear = std::min(1.0, std::max((min_forward-cfg.safety_range) 
                    / (cfg.dont_care_range-cfg.safety_range),0.0))*cfg.max_linear_speed;
        double angular = cfg.max_angular_speed*(std::min(sensor_map[1],
                    cfg.dont_care_range)-std::min(sensor_map[0],cfg.dont_care_range))
            / cfg.dont_care_range;
        if (angular < -cfg.max_angular_speed) {
            angular = -cfg.max_angular_speed;
        } 
        if (angular > cfg.max_angular_speed) {
            angular = cfg.max_angular_speed;
        } 
        // safety to make sure we're not stuck
        if ((fabs(linear)<5e-2)) {
            angular = cfg.max_angular_speed;
        }
        env->publishVelocity(linear, angular);
    } catch (std::exception & e) {
        ROS_ERROR("Exception in speed computation: %s",e.what());
        env->publishVelocity(0.0, 0.0);
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskWander::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryWander);
